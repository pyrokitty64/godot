/**************************************************************************/
/*  rope_solver.h                                                         */
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

#pragma once

#include "core/object/ref_counted.h"
#include "core/variant/variant.h"

// A chain of particles joined by fixed-length links, stepped in native code (Verlet, then passes of
// link, capsule and floor corrections). The pony game's reins (avatar/scripts/Reins.gd) own one; its
// cart belt and tail can later. The caller keeps its own rules (holding, leading, breaking): each step
// it sets the pins and colliders, then calls simulate().
class RopeSolver : public RefCounted {
	GDCLASS(RopeSolver, RefCounted);

	PackedVector3Array positions;
	PackedVector3Array previous;
	PackedFloat32Array inv_mass;
	PackedInt32Array links;
	PackedFloat32Array link_lengths;
	PackedInt32Array pins;
	PackedVector3Array pin_points;
	PackedVector3Array collider_a;
	PackedVector3Array collider_b;
	PackedFloat32Array collider_radius;
	PackedVector3Array collider_centre;
	PackedFloat32Array collider_reach;
	PackedInt32Array pairs;

	Vector3 gravity = Vector3(0, -9.8, 0);
	double damping = 1.5;
	double max_speed = 8.0;
	double friction = 0.3;
	double radius = 0.012;
	int iterations = 8;
	double pair_margin = 0.05;
	double min_contact_weight = 0.25;

	void _apply_pins(Vector3 *p_p);
	void _find_pairs(const Vector3 *p_p);
	void _solve_links(Vector3 *p_p);
	void _solve_colliders(Vector3 *p_p);

protected:
	static void _bind_methods();

public:
	void set_positions(const PackedVector3Array &p_positions) { positions = p_positions; }
	PackedVector3Array get_positions() const { return positions; }
	void set_previous(const PackedVector3Array &p_previous) { previous = p_previous; }
	PackedVector3Array get_previous() const { return previous; }
	void set_inv_mass(const PackedFloat32Array &p_inv_mass) { inv_mass = p_inv_mass; }
	PackedFloat32Array get_inv_mass() const { return inv_mass; }
	void set_particle_inv_mass(int p_index, double p_inv_mass);
	void set_links(const PackedInt32Array &p_links, const PackedFloat32Array &p_lengths);
	void set_pins(const PackedInt32Array &p_indices, const PackedVector3Array &p_points);
	void set_colliders(const PackedVector3Array &p_a, const PackedVector3Array &p_b, const PackedFloat32Array &p_radius, const PackedVector3Array &p_centre, const PackedFloat32Array &p_reach);
	PackedInt32Array get_pairs() const { return pairs; }

	void set_gravity(const Vector3 &p_gravity) { gravity = p_gravity; }
	Vector3 get_gravity() const { return gravity; }
	void set_damping(double p_damping) { damping = p_damping; }
	double get_damping() const { return damping; }
	void set_max_speed(double p_max_speed) { max_speed = p_max_speed; }
	double get_max_speed() const { return max_speed; }
	void set_friction(double p_friction) { friction = p_friction; }
	double get_friction() const { return friction; }
	void set_radius(double p_radius) { radius = p_radius; }
	double get_radius() const { return radius; }
	void set_iterations(int p_iterations) { iterations = p_iterations; }
	int get_iterations() const { return iterations; }
	void set_pair_margin(double p_margin) { pair_margin = p_margin; }
	double get_pair_margin() const { return pair_margin; }
	void set_min_contact_weight(double p_weight) { min_contact_weight = p_weight; }
	double get_min_contact_weight() const { return min_contact_weight; }

	void simulate(double p_delta, double p_floor_y);
};
