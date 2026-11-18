/**************************************************************************/
/*  rope_solver.cpp                                                       */
/**************************************************************************/
/*                         This file is part of:                          */
/*                             GODOT ENGINE                               */
/*                        https://godotengine.org                         */
/**************************************************************************/
/* Copyright (c) 2014-present Godot Engine contributors (see AUTHORS.md). */
/* Copyright (c) 2007-2014 Juan Linietsky, Ariel Manzur.                  */
/*                                                                        */
/* Permission is hereby granted, free of charge, to any person obtaining  */
/* a copy of this software and associated documentation files (the        */
/* "Software"), to deal in the Software without restriction, including    */
/* without limitation the rights to use, copy, modify, merge, publish,    */
/* distribute, sublicense, and/or sell copies of the Software, and to     */
/* permit persons to whom the Software is furnished to do so, subject to  */
/* the following conditions:                                              */
/*                                                                        */
/* The above copyright notice and this permission notice shall be         */
/* included in all copies or substantial portions of the Software.        */
/*                                                                        */
/* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,        */
/* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF     */
/* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. */
/* IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY   */
/* CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,   */
/* TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE      */
/* SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.                 */
/**************************************************************************/

#include "rope_solver.h"

#include "core/math/geometry_3d.h"
#include "core/object/class_db.h"

void RopeSolver::set_particle_inv_mass(int p_index, double p_inv_mass) {
	ERR_FAIL_INDEX(p_index, inv_mass.size());
	inv_mass.set(p_index, p_inv_mass);
}

void RopeSolver::set_links(const PackedInt32Array &p_links, const PackedFloat32Array &p_lengths) {
	ERR_FAIL_COND(p_links.size() != 2 * p_lengths.size());
	links = p_links;
	link_lengths = p_lengths;
}

void RopeSolver::set_pins(const PackedInt32Array &p_indices, const PackedVector3Array &p_points) {
	ERR_FAIL_COND(p_indices.size() != p_points.size());
	pins = p_indices;
	pin_points = p_points;
}

void RopeSolver::set_colliders(const PackedVector3Array &p_a, const PackedVector3Array &p_b, const PackedFloat32Array &p_radius, const PackedVector3Array &p_centre, const PackedFloat32Array &p_reach) {
	const int n = p_radius.size();
	ERR_FAIL_COND(p_a.size() != n || p_b.size() != n || p_centre.size() != n || p_reach.size() != n);
	collider_a = p_a;
	collider_b = p_b;
	collider_radius = p_radius;
	collider_centre = p_centre;
	collider_reach = p_reach;
}

void RopeSolver::_apply_pins(Vector3 *p_p) {
	const int *index = pins.ptr();
	const Vector3 *point = pin_points.ptr();
	for (int k = 0; k < pins.size(); k++) {
		p_p[index[k]] = point[k];
	}
}

// Link-collider pairs within pair_margin of touching, once per step. A link outside a collider's
// bounding sphere skips the exact test.
void RopeSolver::_find_pairs(const Vector3 *p_p) {
	pairs.clear();
	const int *l_ptr = links.ptr();
	const float *len = link_lengths.ptr();
	const float *w = inv_mass.ptr();
	const Vector3 *a = collider_a.ptr();
	const Vector3 *b = collider_b.ptr();
	const float *cr = collider_radius.ptr();
	const Vector3 *centre = collider_centre.ptr();
	const float *reaches = collider_reach.ptr();
	for (int c = 0; c < collider_radius.size(); c++) {
		const double reach = cr[c] + radius + pair_margin;
		for (int l = 0; l < link_lengths.size(); l++) {
			const int i = l_ptr[2 * l];
			const int j = l_ptr[2 * l + 1];
			if ((double)w[i] + (double)w[j] <= 0.0) {
				continue;
			}
			const double bound = reaches[c] + len[l] * 0.5 + radius + pair_margin;
			if ((double)((p_p[i] + p_p[j]) * 0.5).distance_squared_to(centre[c]) >= bound * bound) {
				continue;
			}
			Vector3 q0, q1;
			Geometry3D::get_closest_points_between_segments(p_p[i], p_p[j], a[c], b[c], q0, q1);
			if ((double)q0.distance_squared_to(q1) < reach * reach) {
				pairs.push_back(l);
				pairs.push_back(c);
			}
		}
	}
}

void RopeSolver::_solve_links(Vector3 *p_p) {
	const int *l_ptr = links.ptr();
	const float *len = link_lengths.ptr();
	const float *w = inv_mass.ptr();
	for (int k = 0; k < link_lengths.size(); k++) {
		const int i = l_ptr[2 * k];
		const int j = l_ptr[2 * k + 1];
		const double wi = w[i];
		const double wj = w[j];
		if (wi + wj <= 0.0) {
			continue;
		}
		const Vector3 d = p_p[j] - p_p[i];
		const double length = d.length();
		if (length < 1e-9) {
			continue;
		}
		const Vector3 correction = d * (real_t)((length - len[k]) / (length * (wi + wj)));
		p_p[i] += correction * (real_t)wi;
		p_p[j] -= correction * (real_t)wj;
	}
}

// Link against capsule, not particle against capsule: a finger fits between two particles. The push
// lands on the link's two particles by where along it the contact is and by their inverse masses, so
// the contact point itself moves out.
void RopeSolver::_solve_colliders(Vector3 *p_p) {
	const int *pr = pairs.ptr();
	const int *l_ptr = links.ptr();
	const float *w = inv_mass.ptr();
	const Vector3 *a = collider_a.ptr();
	const Vector3 *b = collider_b.ptr();
	const float *cr = collider_radius.ptr();
	for (int k = 0; k < pairs.size(); k += 2) {
		const int l = pr[k];
		const int c = pr[k + 1];
		const int i = l_ptr[2 * l];
		const int j = l_ptr[2 * l + 1];
		Vector3 q0, q1;
		Geometry3D::get_closest_points_between_segments(p_p[i], p_p[j], a[c], b[c], q0, q1);
		const Vector3 gap = q0 - q1;
		const double r = cr[c] + radius;
		const double dist_sq = gap.length_squared();
		if (dist_sq >= r * r || dist_sq < 1e-12) {
			continue;
		}
		const Vector3 seg = p_p[j] - p_p[i];
		const double s = CLAMP((double)(q0 - p_p[i]).dot(seg) / MAX((double)seg.length_squared(), 1e-12), 0.0, 1.0);
		const double wi = (1.0 - s) * w[i];
		const double wj = s * w[j];
		Vector3 push = gap * (real_t)((r - Math::sqrt(dist_sq)) / Math::sqrt(dist_sq));
		push /= (real_t)MAX((1.0 - s) * wi + s * wj, min_contact_weight);
		p_p[i] += push * (real_t)wi;
		p_p[j] += push * (real_t)wj;
	}
}

// One step of p_delta seconds: Verlet with damping and a speed limit, the pins, then `iterations`
// passes of links, colliders, the floor at p_floor_y and the pins, then floor friction (a particle on
// the floor keeps only 1 - friction of its sideways motion).
void RopeSolver::simulate(double p_delta, double p_floor_y) {
	const int n = positions.size();
	ERR_FAIL_COND(previous.size() != n || inv_mass.size() != n);
	Vector3 *p = positions.ptrw();
	Vector3 *prev = previous.ptrw();
	const real_t decay = Math::exp(-damping * p_delta);
	const real_t limit = max_speed * p_delta;
	const real_t dt = p_delta;
	for (int i = 0; i < n; i++) {
		const Vector3 v = ((p[i] - prev[i]) * decay).limit_length(limit);
		prev[i] = p[i];
		p[i] = p[i] + v + gravity * dt * dt;
	}
	_apply_pins(p);
	_find_pairs(p);
	for (int pass = 0; pass < iterations; pass++) {
		_solve_links(p);
		_solve_colliders(p);
		for (int i = 0; i < n; i++) {
			if ((double)p[i].y < p_floor_y) {
				p[i].y = p_floor_y;
			}
		}
		_apply_pins(p);
	}
	for (int i = 0; i < n; i++) {
		if ((double)p[i].y <= p_floor_y + 0.001) {
			const Vector3 slide = p[i] - prev[i];
			prev[i] += Vector3(slide.x, 0.0, slide.z) * (real_t)friction;
		}
	}
}

void RopeSolver::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_positions", "positions"), &RopeSolver::set_positions);
	ClassDB::bind_method(D_METHOD("get_positions"), &RopeSolver::get_positions);
	ClassDB::bind_method(D_METHOD("set_previous", "previous"), &RopeSolver::set_previous);
	ClassDB::bind_method(D_METHOD("get_previous"), &RopeSolver::get_previous);
	ClassDB::bind_method(D_METHOD("set_inv_mass", "inv_mass"), &RopeSolver::set_inv_mass);
	ClassDB::bind_method(D_METHOD("get_inv_mass"), &RopeSolver::get_inv_mass);
	ClassDB::bind_method(D_METHOD("set_particle_inv_mass", "index", "inv_mass"), &RopeSolver::set_particle_inv_mass);
	ClassDB::bind_method(D_METHOD("set_links", "links", "lengths"), &RopeSolver::set_links);
	ClassDB::bind_method(D_METHOD("set_pins", "indices", "points"), &RopeSolver::set_pins);
	ClassDB::bind_method(D_METHOD("set_colliders", "a", "b", "radius", "centre", "reach"), &RopeSolver::set_colliders);
	ClassDB::bind_method(D_METHOD("get_pairs"), &RopeSolver::get_pairs);
	ClassDB::bind_method(D_METHOD("set_gravity", "gravity"), &RopeSolver::set_gravity);
	ClassDB::bind_method(D_METHOD("get_gravity"), &RopeSolver::get_gravity);
	ClassDB::bind_method(D_METHOD("set_damping", "damping"), &RopeSolver::set_damping);
	ClassDB::bind_method(D_METHOD("get_damping"), &RopeSolver::get_damping);
	ClassDB::bind_method(D_METHOD("set_max_speed", "max_speed"), &RopeSolver::set_max_speed);
	ClassDB::bind_method(D_METHOD("get_max_speed"), &RopeSolver::get_max_speed);
	ClassDB::bind_method(D_METHOD("set_friction", "friction"), &RopeSolver::set_friction);
	ClassDB::bind_method(D_METHOD("get_friction"), &RopeSolver::get_friction);
	ClassDB::bind_method(D_METHOD("set_radius", "radius"), &RopeSolver::set_radius);
	ClassDB::bind_method(D_METHOD("get_radius"), &RopeSolver::get_radius);
	ClassDB::bind_method(D_METHOD("set_iterations", "iterations"), &RopeSolver::set_iterations);
	ClassDB::bind_method(D_METHOD("get_iterations"), &RopeSolver::get_iterations);
	ClassDB::bind_method(D_METHOD("set_pair_margin", "pair_margin"), &RopeSolver::set_pair_margin);
	ClassDB::bind_method(D_METHOD("get_pair_margin"), &RopeSolver::get_pair_margin);
	ClassDB::bind_method(D_METHOD("set_min_contact_weight", "min_contact_weight"), &RopeSolver::set_min_contact_weight);
	ClassDB::bind_method(D_METHOD("get_min_contact_weight"), &RopeSolver::get_min_contact_weight);
	ClassDB::bind_method(D_METHOD("simulate", "delta", "floor_y"), &RopeSolver::simulate);

	ADD_PROPERTY(PropertyInfo(Variant::PACKED_VECTOR3_ARRAY, "positions"), "set_positions", "get_positions");
	ADD_PROPERTY(PropertyInfo(Variant::PACKED_VECTOR3_ARRAY, "previous"), "set_previous", "get_previous");
	ADD_PROPERTY(PropertyInfo(Variant::PACKED_FLOAT32_ARRAY, "inv_mass"), "set_inv_mass", "get_inv_mass");
	ADD_PROPERTY(PropertyInfo(Variant::VECTOR3, "gravity"), "set_gravity", "get_gravity");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "damping"), "set_damping", "get_damping");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "max_speed"), "set_max_speed", "get_max_speed");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "friction"), "set_friction", "get_friction");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "radius"), "set_radius", "get_radius");
	ADD_PROPERTY(PropertyInfo(Variant::INT, "iterations"), "set_iterations", "get_iterations");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "pair_margin"), "set_pair_margin", "get_pair_margin");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "min_contact_weight"), "set_min_contact_weight", "get_min_contact_weight");
}
