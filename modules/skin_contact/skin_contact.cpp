/**************************************************************************/
/*  skin_contact.cpp                                                      */
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

#include "skin_contact.h"

#include "scene/3d/mesh_instance_3d.h"
#include "scene/3d/skeleton_3d.h"
#include "scene/resources/3d/importer_mesh.h"
#include "scene/resources/3d/skin.h"

#include "scene/scene_string_names.h"
#include "core/object/callable_mp.h"
#include "core/object/class_db.h"
#include "core/os/os.h"
#include "core/templates/hash_map.h"
#include "core/templates/hash_set.h"

// How far her skin may move under a still hand in a frame.
static const real_t SKIN_MOTION = 0.01;
// How far outside her skin a touching hand point rests.
static const real_t GAP = 0.001;
// How far a touching point may slide in a frame (1.8 m/s at 90 Hz): the skin is gathered that far round it.
static const real_t SLIDE = 0.02;
// A vertex strays this far from where its heaviest bind alone would put it (joint blends, keys).
static const real_t GROUP_SLACK = 0.05;

// A skinning transform's turn as skeleton.glsl reads it (quat_from_rows).
static Quaternion quat_from_rows(const Basis &p_basis) {
	const Vector3 &r0 = p_basis.rows[0];
	const Vector3 &r1 = p_basis.rows[1];
	const Vector3 &r2 = p_basis.rows[2];
	const real_t trace = r0.x + r1.y + r2.z;
	Quaternion q;
	if (trace > 0.0) {
		const real_t s = 0.5 / Math::sqrt(trace + 1.0);
		q = Quaternion((r2.y - r1.z) * s, (r0.z - r2.x) * s, (r1.x - r0.y) * s, 0.25 / s);
	} else if (r0.x > r1.y && r0.x > r2.z) {
		const real_t s = 2.0 * Math::sqrt(1.0 + r0.x - r1.y - r2.z);
		q = Quaternion(0.25 * s, (r0.y + r1.x) / s, (r0.z + r2.x) / s, (r2.y - r1.z) / s);
	} else if (r1.y > r2.z) {
		const real_t s = 2.0 * Math::sqrt(1.0 + r1.y - r0.x - r2.z);
		q = Quaternion((r0.y + r1.x) / s, 0.25 * s, (r1.z + r2.y) / s, (r0.z - r2.x) / s);
	} else {
		const real_t s = 2.0 * Math::sqrt(1.0 + r2.z - r0.x - r1.y);
		q = Quaternion((r0.z + r2.x) / s, (r1.z + r2.y) / s, 0.25 * s, (r1.x - r0.y) / s);
	}
	return q.normalized();
}

// Turns v by the unit quaternion q, as the shader writes it.
static Vector3 turn(const Quaternion &q, const Vector3 &v) {
	const Vector3 u(q.x, q.y, q.z);
	return v + 2.0 * u.cross(u.cross(v) + q.w * v);
}

static Vector3 closest_on_segment(const Vector3 &p_a, const Vector3 &p_b, const Vector3 &p_point) {
	const Vector3 ab = p_b - p_a;
	const real_t len2 = ab.length_squared();
	if (len2 <= CMP_EPSILON2) {
		return p_a;
	}
	return p_a + ab * CLAMP((p_point - p_a).dot(ab) / len2, 0.0, 1.0);
}

Skeleton3D *SkinBody::_skeleton() const {
	return ObjectDB::get_instance<Skeleton3D>(skeleton);
}

struct SplitItem {
	real_t key = 0.0;
	uint32_t vertex = 0;
	bool operator<(const SplitItem &p_other) const { return key < p_other.key; }
};

// `p_vertices` as chunks of at most CHUNK, halved at the median of their longest side.
void SkinBody::_split(int p_bind, const LocalVector<uint32_t> &p_vertices, LocalVector<Group> &r_chunks) const {
	static const uint32_t CHUNK = 128;
	if (p_vertices.is_empty()) {
		return;
	}
	AABB box(rest[p_vertices[0]], Vector3());
	for (uint32_t v : p_vertices) {
		box.expand_to(rest[v]);
	}
	if (p_vertices.size() <= CHUNK) {
		Group g;
		g.bind = p_bind;
		g.centre = box.get_center();
		g.radius = box.size.length() * 0.5;
		g.vertices = p_vertices;
		r_chunks.push_back(g);
		return;
	}
	const int axis = box.get_longest_axis_index();
	LocalVector<SplitItem> items;
	items.resize(p_vertices.size());
	for (uint32_t i = 0; i < p_vertices.size(); i++) {
		items[i].key = rest[p_vertices[i]][axis];
		items[i].vertex = p_vertices[i];
	}
	items.sort();
	const uint32_t half = items.size() / 2;
	LocalVector<uint32_t> low;
	LocalVector<uint32_t> high;
	for (uint32_t i = 0; i < items.size(); i++) {
		(i < half ? low : high).push_back(items[i].vertex);
	}
	_split(p_bind, low, r_chunks);
	_split(p_bind, high, r_chunks);
}

// Adds surfaces of a mesh skinned to one skeleton (every body mesh of an avatar shares it).
// Empty `p_surfaces`: all of them.
bool SkinBody::add_mesh(MeshInstance3D *p_mesh, const PackedInt32Array &p_surfaces, const Ref<ImporterMesh> &p_source) {
	ERR_FAIL_NULL_V(p_mesh, false);
	Ref<Mesh> mesh = p_mesh->get_mesh();
	ERR_FAIL_COND_V(mesh.is_null(), false);
	Skeleton3D *skel = Object::cast_to<Skeleton3D>(p_mesh->get_node_or_null(p_mesh->get_skeleton_path()));
	ERR_FAIL_NULL_V_MSG(skel, false, "The mesh isn't skinned to a skeleton.");
	if (skeleton.is_null()) {
		skeleton = skel->get_instance_id();
		skel->connect(SceneStringName(skeleton_updated), callable_mp(this, &SkinBody::_take_pose));
	}
	ERR_FAIL_COND_V_MSG(skel->get_instance_id() != skeleton, false, "Every mesh of a body must share its skeleton.");
	Ref<Skin> skin = p_mesh->get_skin();
	if (skin.is_null() && p_mesh->get_skin_reference().is_valid()) {
		skin = p_mesh->get_skin_reference()->get_skin();
	}
	ERR_FAIL_COND_V_MSG(skin.is_null(), false, "The mesh has no skin.");

	// This skin's binds in the body's table.
	const uint32_t bind_base = binds.size();
	const uint32_t first_shape = shapes.size();
	const uint32_t first_vertex = rest.size();
	for (int i = 0; i < skin->get_bind_count(); i++) {
		Bind b;
		const StringName name = skin->get_bind_name(i);
		b.bone = name != StringName() ? skel->find_bone(name) : skin->get_bind_bone(i);
		b.pose = skin->get_bind_pose(i);
		binds.push_back(b);
	}

	for (uint32_t i = bind_base; i < binds.size(); i++) {
		binds[i].surface = surfaces.size(); // this mesh's first surface: all share its node transform
	}
	const bool from_source = p_source.is_valid();
	ERR_FAIL_COND_V_MSG(from_source && p_source->get_surface_count() != mesh->get_surface_count(), false, "The ImporterMesh isn't the mesh's.");
	Ref<ArrayMesh> array_mesh = mesh;
	const bool relative = from_source ? p_source->get_blend_shape_mode() == Mesh::BLEND_SHAPE_MODE_RELATIVE : array_mesh.is_valid() && array_mesh->get_blend_shape_mode() == Mesh::BLEND_SHAPE_MODE_RELATIVE;
	PackedInt32Array wanted = p_surfaces;
	if (wanted.is_empty()) {
		for (int s = 0; s < mesh->get_surface_count(); s++) {
			wanted.push_back(s);
		}
	}
	HashMap<int, uint32_t> group_of; // bind -> into groups
	for (const Group &g : groups) {
		group_of.insert(g.bind, &g - groups.ptr());
	}
	for (int s : wanted) {
		ERR_CONTINUE(s < 0 || s >= mesh->get_surface_count());
		const Array arrays = from_source ? p_source->get_surface_arrays(s) : mesh->surface_get_arrays(s);
		if (arrays[Mesh::ARRAY_BONES].get_type() == Variant::NIL || arrays[Mesh::ARRAY_WEIGHTS].get_type() == Variant::NIL) {
			continue;
		}
		const PackedVector3Array vertices = arrays[Mesh::ARRAY_VERTEX];
		const PackedVector3Array vertex_normals = arrays[Mesh::ARRAY_NORMAL];
		const PackedInt32Array bones = arrays[Mesh::ARRAY_BONES];
		const PackedFloat32Array vertex_weights = arrays[Mesh::ARRAY_WEIGHTS];
		const uint32_t count = vertices.size();
		if (count == 0 || bones.size() != vertex_weights.size() || bones.size() % count != 0) {
			continue;
		}
		const uint32_t stride = bones.size() / count;
		// One vertex per `spacing` cell and normal direction (both sides of an ear stay): a finger
		// needs no more, and studs and rings carry vertices a fraction of a millimetre apart.
		LocalVector<int> kept_as; // source vertex -> kept vertex, or -1
		kept_as.resize(count);
		LocalVector<uint32_t> into; // source vertex -> the kept vertex of its cell
		into.resize(count);
		LocalVector<uint32_t> kept;
		HashMap<uint64_t, uint32_t> cells;
		for (uint32_t v = 0; v < count; v++) {
			const Vector3 normal = v < (uint32_t)vertex_normals.size() ? vertex_normals[v] : Vector3(0, 1, 0);
			const int axis = normal.abs().max_axis_index();
			const int face = axis * 2 + (normal[axis] < 0.0 ? 1 : 0);
			const Vector3i cell = spacing > 0.0 ? Vector3i((vertices[v] / spacing).floor()) : Vector3i(v, 0, 0);
			const uint64_t key = (uint64_t(cell.x & 0xFFFFF) << 44) | (uint64_t(cell.y & 0xFFFFF) << 24) | (uint64_t(cell.z & 0xFFFFF) << 4) | uint64_t(face);
			if (cells.has(key)) {
				kept_as[v] = -1;
				into[v] = cells[key];
				continue;
			}
			cells.insert(key, kept.size());
			into[v] = kept.size();
			kept_as[v] = kept.size();
			kept.push_back(v);
		}
		Surface surface;
		surface.mesh = p_mesh->get_instance_id();
		surface.first = rest.size();
		surface.count = kept.size();
		for (uint32_t v : kept) {
			surface.source.push_back(v);
		}
		const uint32_t surface_index = surfaces.size();
		surfaces.push_back(surface);
		for (uint32_t i = 0; i < kept.size(); i++) {
			const uint32_t v = kept[i];
			rest.push_back(vertices[v]);
			rest_normals.push_back(v < (uint32_t)vertex_normals.size() ? vertex_normals[v] : Vector3(0, 1, 0));
			surface_of.push_back(surface_index);
			int heaviest = -1;
			float heaviest_weight = 0.0;
			for (uint32_t k = 0; k < 8; k++) {
				// The GPU keeps weights as 16-bit unorm.
				const float w = k < stride ? Math::round(CLAMP(vertex_weights[v * stride + k], 0.0f, 1.0f) * 65535.0f) / 65535.0f : 0.0f;
				const int b = k < stride && w > 0.0f ? bones[v * stride + k] : -1;
				joints.push_back(b >= 0 && b < skin->get_bind_count() ? int(bind_base + b) : -1);
				weights.push_back(joints[joints.size() - 1] >= 0 ? w : 0.0f);
				if (joints[joints.size() - 1] >= 0 && w > heaviest_weight) {
					heaviest_weight = w;
					heaviest = bind_base + b;
				}
			}
			if (heaviest < 0 || binds[heaviest].bone < 0) {
				continue;
			}
			if (!group_of.has(heaviest)) {
				group_of.insert(heaviest, groups.size());
				Group g;
				g.bind = heaviest;
				groups.push_back(g);
			}
			groups[group_of[heaviest]].vertices.push_back(surface.first + i);
		}
		// Triangles, on the kept vertices (those merged into one cell drop out).
		const PackedInt32Array indices = arrays[Mesh::ARRAY_INDEX];
		const uint32_t corners = indices.is_empty() ? count : indices.size();
		for (uint32_t i = 0; i + 2 < corners; i += 3) {
			uint32_t t[3];
			for (int c = 0; c < 3; c++) {
				t[c] = surface.first + into[indices.is_empty() ? i + c : indices[i + c]];
			}
			if (t[0] == t[1] || t[1] == t[2] || t[0] == t[2]) {
				continue;
			}
			for (int c = 0; c < 3; c++) {
				tris.push_back(t[c]);
			}
		}
		// Keys, sparse: only the vertices each moves.
		TypedArray<Array> blend;
		if (from_source) {
			for (int k = 0; k < p_source->get_blend_shape_count(); k++) {
				blend.push_back(p_source->get_surface_blend_shape_arrays(s, k));
			}
		} else {
			blend = mesh->surface_get_blend_shape_arrays(s);
		}
		for (int k = 0; k < blend.size(); k++) {
			const Array shape_arrays = blend[k];
			if (shape_arrays.is_empty()) {
				continue;
			}
			const PackedVector3Array shape_vertices = shape_arrays[Mesh::ARRAY_VERTEX];
			if ((uint32_t)shape_vertices.size() != count) {
				continue;
			}
			Shape shape;
			shape.surface = surface_index;
			shape.index = k;
			for (uint32_t v = 0; v < count; v++) {
				const Vector3 delta = relative ? shape_vertices[v] : shape_vertices[v] - vertices[v];
				if (kept_as[v] >= 0 && delta.length_squared() > 1e-14) {
					shape.vertices.push_back(surface.first + kept_as[v]);
					shape.deltas.push_back(delta);
				}
			}
			if (!shape.vertices.is_empty()) {
				shapes.push_back(shape);
			}
		}
	}
	// Split into chunks of nearby vertices, each with its bounds (in the mesh's space, which its
	// bind carries rigidly), so a hand skins only the chunks around it.
	LocalVector<Group> chunks;
	for (Group &group : groups) {
		_split(group.bind, group.vertices, chunks);
	}
	groups = chunks;
	const uint32_t n = rest.size();
	// The new vertices' keys, per vertex (a vertex's keys are what skinning it needs).
	if (key_start.is_empty()) {
		key_start.push_back(0);
	}
	LocalVector<uint32_t> cursor;
	cursor.resize(n - first_vertex);
	for (uint32_t &c : cursor) {
		c = 0;
	}
	for (uint32_t k = first_shape; k < shapes.size(); k++) {
		for (uint32_t v : shapes[k].vertices) {
			cursor[v - first_vertex]++;
		}
	}
	for (uint32_t i = 0; i < cursor.size(); i++) {
		const uint32_t at = key_start[key_start.size() - 1];
		key_start.push_back(at + cursor[i]);
		cursor[i] = at;
	}
	key_shape.resize(key_start[key_start.size() - 1]);
	key_delta.resize(key_start[key_start.size() - 1]);
	for (uint32_t k = first_shape; k < shapes.size(); k++) {
		Shape &shape = shapes[k];
		for (uint32_t i = 0; i < shape.vertices.size(); i++) {
			const uint32_t slot = cursor[shape.vertices[i] - first_vertex]++;
			key_shape[slot] = k;
			key_delta[slot] = shape.deltas[i];
		}
		shape.vertices.reset();
		shape.deltas.reset();
	}
	// Each vertex's triangles.
	tri_start.resize(n + 1);
	for (uint32_t &c : tri_start) {
		c = 0;
	}
	for (uint32_t v : tris) {
		tri_start[v + 1]++;
	}
	for (uint32_t v = 0; v < n; v++) {
		tri_start[v + 1] += tri_start[v];
	}
	tri_of.resize(tris.size());
	vertex_edge.resize(n);
	for (real_t &e : vertex_edge) {
		e = 0.0;
	}
	for (uint32_t i = 0; i < tris.size(); i++) {
		const uint32_t v = tris[i];
		const uint32_t w = tris[i - i % 3 + (i + 1) % 3];
		const real_t e = rest[v].distance_to(rest[w]);
		vertex_edge[v] = MAX(vertex_edge[v], e);
		vertex_edge[w] = MAX(vertex_edge[w], e);
	}
	for (Group &g : groups) {
		g.edge = 0.0;
		for (uint32_t v : g.vertices) {
			g.edge = MAX(g.edge, vertex_edge[v]);
		}
	}
	LocalVector<uint32_t> fill;
	fill.resize(n);
	for (uint32_t v = 0; v < n; v++) {
		fill[v] = tri_start[v];
	}
	for (uint32_t i = 0; i < tris.size(); i++) {
		tri_of[fill[tris[i]]++] = i / 3;
	}
	skinned_frame.resize(n);
	morph.resize(n);
	world.resize(n);
	world_normals.resize(n);
	for (uint32_t v = 0; v < n; v++) {
		skinned_frame[v] = 0;
	}
	// Taken again, with these binds, when next drawn.
	MutexLock lock(pose_mutex);
	taken = Pose();
	taken.version = now.version + 1;
	now = Pose();
	now.version = taken.version;
	return true;
}

// The binds' skinning transforms and turns, each surface's node transform and the key weights, as drawn.
void SkinBody::_take_pose() {
	Skeleton3D *skel = _skeleton();
	if (skel == nullptr || !skel->is_inside_tree()) {
		return;
	}
	Pose p;
	p.dual_quaternion = skel->is_dual_quaternion_skinning(); // may be switched on after the meshes are added
	p.bind_now.resize(binds.size());
	p.bind_turn.resize(binds.size());
	for (uint32_t i = 0; i < binds.size(); i++) {
		p.bind_now[i] = binds[i].bone >= 0 ? skel->get_bone_global_pose(binds[i].bone) * binds[i].pose : binds[i].pose;
		p.bind_turn[i] = quat_from_rows(p.bind_now[i].basis);
	}
	p.mesh_now.resize(surfaces.size());
	for (uint32_t s = 0; s < surfaces.size(); s++) {
		MeshInstance3D *mi = ObjectDB::get_instance<MeshInstance3D>(surfaces[s].mesh);
		p.mesh_now[s] = mi != nullptr ? mi->get_global_transform() : skel->get_global_transform();
	}
	// Each bind in the world, through its mesh's node, for culling.
	p.bind_world.resize(binds.size());
	p.bind_scale.resize(binds.size());
	for (uint32_t i = 0; i < binds.size(); i++) {
		p.bind_world[i] = (binds[i].surface < p.mesh_now.size() ? p.mesh_now[binds[i].surface] : skel->get_global_transform()) * p.bind_now[i];
		p.bind_scale[i] = p.bind_world[i].basis.get_scale_abs().x;
	}
	p.shape_weight.resize(shapes.size());
	for (uint32_t k = 0; k < shapes.size(); k++) {
		MeshInstance3D *mi = ObjectDB::get_instance<MeshInstance3D>(surfaces[shapes[k].surface].mesh);
		p.shape_weight[k] = mi != nullptr ? mi->get_blend_shape_value(shapes[k].index) : 0.0f;
	}
	MutexLock lock(pose_mutex);
	p.version = taken.version + 1;
	taken = p;
}

void SkinBody::sync() {
	MutexLock lock(pose_mutex);
	if (now.version != taken.version) {
		now = taken;
	}
}

// One vertex at `p_at` (its keyed bind position) through its binds, in the mesh's space.
Vector3 SkinBody::_skin(uint32_t p_vertex, const Vector3 &p_at, Vector3 &r_normal) const {
	const int *j = &joints[p_vertex * 8];
	const float *w = &weights[p_vertex * 8];
	const Vector3 &normal = rest_normals[p_vertex];
	if (now.dual_quaternion) {
		int lead = 0;
		for (int i = 1; i < 8; i++) {
			if (w[i] > w[lead]) {
				lead = i;
			}
		}
		Quaternion real(0, 0, 0, 0);
		Quaternion dual(0, 0, 0, 0);
		Quaternion lead_real;
		for (int k = 0; k < 8; k++) {
			const int i = k == 0 ? lead : (k == lead ? 0 : k);
			if (w[i] == 0.0f || j[i] < 0) {
				continue;
			}
			const Quaternion &q = now.bind_turn[j[i]];
			if (k == 0) {
				lead_real = q;
			}
			const real_t weight = q.dot(lead_real) < 0.0 ? -w[i] : w[i];
			const Vector3 moved = now.bind_now[j[i]].xform(p_at);
			const Vector3 d = moved - turn(q, p_at);
			real += q * weight;
			dual += Quaternion(d.x, d.y, d.z, 0.0) * q * (0.5 * weight);
		}
		const real_t n = real.length();
		if (n <= CMP_EPSILON) {
			r_normal = normal;
			return p_at;
		}
		real /= n;
		dual /= n;
		const Vector3 rv(real.x, real.y, real.z);
		const Vector3 dv(dual.x, dual.y, dual.z);
		const Vector3 translation = 2.0 * (real.w * dv - dual.w * rv + rv.cross(dv));
		r_normal = turn(real, normal).normalized();
		return turn(real, p_at) + translation;
	}
	Basis basis = Basis(Vector3(), Vector3(), Vector3());
	Vector3 origin;
	for (int i = 0; i < 8; i++) {
		if (w[i] == 0.0f || j[i] < 0) {
			continue;
		}
		const Transform3D &m = now.bind_now[j[i]];
		basis.rows[0] += m.basis.rows[0] * w[i];
		basis.rows[1] += m.basis.rows[1] * w[i];
		basis.rows[2] += m.basis.rows[2] * w[i];
		origin += m.origin * w[i];
	}
	r_normal = basis.xform(normal).normalized();
	return basis.xform(p_at) + origin;
}

// How far p lies outside the nearest of the segments, each grown by its reach (negative: inside).
static real_t outside_segments(const Vector3 &p_point, const LocalVector<Vector3> &p_a, const LocalVector<Vector3> &p_b, const LocalVector<real_t> &p_reach) {
	real_t best = INFINITY;
	for (uint32_t k = 0; k < p_a.size(); k++) {
		best = MIN(best, closest_on_segment(p_a[k], p_b[k], p_point).distance_to(p_point) - p_reach[k]);
	}
	return best;
}

void SkinBody::gather(const LocalVector<Vector3> &p_a, const LocalVector<Vector3> &p_b, const LocalVector<real_t> &p_reach, LocalVector<Vector3> &r_positions, LocalVector<Vector3> &r_normals, LocalVector<uint32_t> *r_near) {
	const bool edges = r_near != nullptr; // whole triangles: each vertex reaches as far as its longest edge
	if (rest.is_empty() || now.bind_now.size() != binds.size()) {
		return; // not drawn yet
	}
	const uint64_t frame = now.version;
	const uint64_t started = OS::get_singleton()->get_ticks_usec();
	stats = Stats();
	LocalVector<uint32_t> fresh;
	LocalVector<uint32_t> near;
	// One sphere round every segment first: most chunks are nowhere near.
	AABB box(p_a[0], Vector3());
	real_t widest = 0.0;
	for (uint32_t k = 0; k < p_a.size(); k++) {
		box.expand_to(p_a[k]);
		box.expand_to(p_b[k]);
		widest = MAX(widest, p_reach[k]);
	}
	const Vector3 centre = box.get_center();
	const real_t radius = box.size.length() * 0.5 + widest;
	for (const Group &g : groups) {
		const Transform3D &xf = now.bind_world[g.bind];
		const Vector3 at = xf.xform(g.centre);
		const real_t extent = (g.radius + (edges ? g.edge : 0.0)) * now.bind_scale[g.bind] + GROUP_SLACK;
		if (at.distance_to(centre) > radius + extent || outside_segments(at, p_a, p_b, p_reach) > extent) {
			continue;
		}
		stats.chunks++;
		stats.tested += g.vertices.size();
		for (uint32_t v : g.vertices) {
			if (skinned_frame[v] == frame) {
				near.push_back(v);
				continue;
			}
			// Where its heaviest bind alone puts it; only those near enough are skinned.
			const Vector3 rough = xf.xform(rest[v]);
			const real_t slack = GROUP_SLACK + (edges ? vertex_edge[v] * now.bind_scale[g.bind] : 0.0);
			if (rough.distance_to(centre) > radius + slack || outside_segments(rough, p_a, p_b, p_reach) > slack) {
				continue;
			}
			near.push_back(v);
			fresh.push_back(v);
			morph[v] = rest[v];
		}
	}
	// Keys at this frame's weights, on the vertices not yet skinned.
	if (!fresh.is_empty()) {
		for (uint32_t v : fresh) {
			for (uint32_t e = key_start[v]; e < key_start[v + 1]; e++) {
				const float w = now.shape_weight[key_shape[e]];
				if (w != 0.0f) {
					morph[v] += key_delta[e] * w;
				}
			}
			stats.shape_entries += key_start[v + 1] - key_start[v];
		}
	}
	const uint64_t skinning = OS::get_singleton()->get_ticks_usec();
	stats.skinned = fresh.size();
	for (uint32_t v : fresh) {
		Vector3 normal;
		const Vector3 p = _skin(v, morph[v], normal);
		const Transform3D &mesh_xf = now.mesh_now[surface_of[v]];
		world[v] = mesh_xf.xform(p);
		world_normals[v] = mesh_xf.basis.xform(normal).normalized();
		skinned_frame[v] = frame;
	}
	stats.skin_usec = OS::get_singleton()->get_ticks_usec() - skinning;
	stats.usec = OS::get_singleton()->get_ticks_usec() - started;
	if (r_near != nullptr) {
		*r_near = near;
		return;
	}
	for (uint32_t v : near) {
		if (outside_segments(world[v], p_a, p_b, p_reach) <= 0.0) {
			r_positions.push_back(world[v]);
			r_normals.push_back(world_normals[v]);
		}
	}
}

void SkinBody::gather_triangles(const Vector3 &p_from, const Vector3 &p_to, real_t p_around, LocalVector<Triangle> &r_triangles) {
	LocalVector<Vector3> a;
	LocalVector<Vector3> b;
	LocalVector<real_t> reach;
	a.push_back(p_from);
	b.push_back(p_to);
	reach.push_back(0.001);
	a.push_back(p_from);
	b.push_back(p_from);
	reach.push_back(p_around);
	LocalVector<Vector3> unused;
	LocalVector<uint32_t> near;
	gather(a, b, reach, unused, unused, &near);
	HashSet<uint32_t> seen;
	for (uint32_t v : near) {
		for (uint32_t i = tri_start[v]; i < tri_start[v + 1]; i++) {
			const uint32_t t = tri_of[i];
			if (seen.has(t)) {
				continue;
			}
			seen.insert(t);
			const uint32_t *c = &tris[t * 3];
			if (skinned_frame[c[0]] != now.version || skinned_frame[c[1]] != now.version || skinned_frame[c[2]] != now.version) {
				continue;
			}
			Triangle tri;
			tri.id = t;
			tri.a = world[c[0]];
			tri.b = world[c[1]];
			tri.c = world[c[2]];
			// Outward: the side its vertex normals face (the winding is the mesh's affair).
			const Vector3 face = (tri.b - tri.a).cross(tri.c - tri.a);
			if (face.length_squared() < 1e-14) {
				continue;
			}
			tri.normal = face.normalized();
			if (tri.normal.dot(world_normals[c[0]] + world_normals[c[1]] + world_normals[c[2]]) < 0.0) {
				tri.normal = -tri.normal;
			}
			r_triangles.push_back(tri);
		}
	}
}

// [positions, normals] near the sphere, for checks and tools.
Array SkinBody::gather_near(const Vector3 &p_centre, real_t p_radius) {
	sync();
	LocalVector<Vector3> p;
	LocalVector<Vector3> n;
	LocalVector<Vector3> at;
	at.push_back(p_centre);
	LocalVector<real_t> reach;
	reach.push_back(p_radius);
	gather(at, at, reach, p, n);
	PackedVector3Array positions;
	PackedVector3Array out_normals;
	for (uint32_t i = 0; i < p.size(); i++) {
		positions.push_back(p[i]);
		out_normals.push_back(n[i]);
	}
	Array out;
	out.push_back(positions);
	out.push_back(out_normals);
	return out;
}

PackedVector3Array SkinBody::debug_skin_surface(int p_surface) {
	PackedVector3Array out;
	ERR_FAIL_INDEX_V(p_surface, (int)surfaces.size(), out);
	sync();
	if (now.bind_now.size() != binds.size()) {
		return out;
	}
	const Surface &surface = surfaces[p_surface];
	LocalVector<Vector3> at;
	at.resize(surface.count);
	for (uint32_t v = 0; v < surface.count; v++) {
		at[v] = rest[surface.first + v];
	}
	for (uint32_t v = 0; v < surface.count; v++) {
		for (uint32_t e = key_start[surface.first + v]; e < key_start[surface.first + v + 1]; e++) {
			at[v] += key_delta[e] * now.shape_weight[key_shape[e]];
		}
	}
	out.resize(surface.count);
	for (uint32_t v = 0; v < surface.count; v++) {
		Vector3 normal;
		out.write[v] = _skin(surface.first + v, at[v], normal);
	}
	return out;
}

Dictionary SkinBody::get_stats() const {
	Dictionary d;
	d["chunks"] = stats.chunks;
	d["tested"] = stats.tested;
	d["skinned"] = stats.skinned;
	d["shape_entries"] = stats.shape_entries;
	d["usec"] = stats.usec;
	d["skin_usec"] = stats.skin_usec;
	return d;
}

PackedInt32Array SkinBody::debug_surface_vertices(int p_surface) const {
	PackedInt32Array out;
	ERR_FAIL_INDEX_V(p_surface, (int)surfaces.size(), out);
	for (uint32_t v : surfaces[p_surface].source) {
		out.push_back(v);
	}
	return out;
}

void SkinBody::_bind_methods() {
	ClassDB::bind_method(D_METHOD("add_mesh", "mesh", "surfaces", "source"), &SkinBody::add_mesh, DEFVAL(PackedInt32Array()), DEFVAL(Ref<ImporterMesh>()));
	ClassDB::bind_method(D_METHOD("get_vertex_count"), &SkinBody::get_vertex_count);
	ClassDB::bind_method(D_METHOD("gather_near", "centre", "radius"), &SkinBody::gather_near);
	ClassDB::bind_method(D_METHOD("debug_skin_surface", "surface"), &SkinBody::debug_skin_surface);
	ClassDB::bind_method(D_METHOD("debug_surface_vertices", "surface"), &SkinBody::debug_surface_vertices);
	ClassDB::bind_method(D_METHOD("get_stats"), &SkinBody::get_stats);
	ClassDB::bind_method(D_METHOD("set_spacing", "spacing"), &SkinBody::set_spacing);
	ClassDB::bind_method(D_METHOD("get_spacing"), &SkinBody::get_spacing);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "spacing"), "set_spacing", "get_spacing");
}

// Where the segment from -> to first crosses the triangle, as a fraction of it (Moller-Trumbore).
// Geometry3D::segment_intersects_triangle takes a millimetre step over centimetre triangles as parallel.
static bool crosses(const Vector3 &p_from, const Vector3 &p_to, const SkinBody::Triangle &p_t, real_t &r_along) {
	const Vector3 rel = p_to - p_from;
	const Vector3 e1 = p_t.b - p_t.a;
	const Vector3 e2 = p_t.c - p_t.a;
	const Vector3 h = rel.cross(e2);
	const real_t det = e1.dot(h);
	if (Math::abs(det) <= 1e-6 * rel.length() * e1.length() * e2.length()) {
		return false;
	}
	const Vector3 s = p_from - p_t.a;
	const real_t u = s.dot(h) / det;
	if (u < 0.0 || u > 1.0) {
		return false;
	}
	const Vector3 q = s.cross(e1);
	const real_t v = rel.dot(q) / det;
	if (v < 0.0 || u + v > 1.0) {
		return false;
	}
	r_along = e2.dot(q) / det;
	return r_along >= 0.0 && r_along <= 1.0;
}

// Where p lies on triangle t, as weights of its corners.
static Vector3 barycentric(const SkinBody::Triangle &p_t, const Vector3 &p_point) {
	const Vector3 e0 = p_t.b - p_t.a;
	const Vector3 e1 = p_t.c - p_t.a;
	const Vector3 d = p_point - p_t.a;
	const real_t d00 = e0.dot(e0);
	const real_t d01 = e0.dot(e1);
	const real_t d11 = e1.dot(e1);
	const real_t den = d00 * d11 - d01 * d01;
	if (Math::abs(den) < 1e-20) {
		return Vector3(1, 0, 0);
	}
	const real_t v = (d11 * d.dot(e0) - d01 * d.dot(e1)) / den;
	const real_t w = (d00 * d.dot(e1) - d01 * d.dot(e0)) / den;
	return Vector3(1.0 - v - w, v, w);
}

// The hand point follows the controller until it meets a triangle from its outer side, then slides
// over the skin with the controller, less the deeper it is pressed (the "god object" of haptics): up
// to two faces at once, a face or a crease. Faces stop it only from outside, so a hand that is inside (started
// there, or let go) comes out freely. While touching, the face it rests on carries it as she moves.
Vector3 HandContact::resolve(const Array &p_bodies, const Vector3 &p_point) {
	contacts = 0;
	if (!started) {
		started = true;
		proxy = p_point;
		last_point = p_point;
		touching = false;
		offset = Vector3();
		return offset;
	}
	LocalVector<SkinBody::Triangle> triangles;
	LocalVector<ObjectID> owner; // per triangle, its body
	for (int i = 0; i < p_bodies.size(); i++) {
		Ref<SkinBody> body = p_bodies[i];
		if (body.is_valid()) {
			const uint32_t before = triangles.size();
			body->sync();
			body->gather_triangles(proxy, p_point, SKIN_MOTION + SLIDE, triangles);
			for (uint32_t k = before; k < triangles.size(); k++) {
				owner.push_back(body->get_instance_id());
			}
		}
	}
	Vector3 goal = p_point;
	Vector3 planes[2];
	int count = 0;
	real_t depth = 0.0; // how far the controller is in past the face the point rests on
	// Moves the point along a face it is pressed into; false when it stays put. On the first face it
	// slides by the controller's move along her skin, scaled down by depth: slide_depth / (slide_depth
	// + depth), a half at slide_depth in. Pulled back out, it catches up the ground it lost in the same
	// proportion as the depth shrinks, so it has caught up when the controller leaves her skin.
	auto slide_on = [&](const Vector3 &p_normal, const Vector3 &p_move) -> bool {
		Vector3 step;
		if (count == 0) {
			depth = (proxy - p_point).dot(p_normal);
			const Vector3 move = p_move - p_normal * p_move.dot(p_normal);
			step = move * (slide_depth / (slide_depth + MAX(depth, (real_t)0.0)));
			// The controller's own move out, not the change in depth, which jumps from face to face.
			const real_t out = p_move.dot(p_normal);
			if (out > 0.0 && last_depth > 0.0) {
				const Vector3 lag = last_point - proxy;
				step += (lag - p_normal * lag.dot(p_normal)) * MIN(out / last_depth, (real_t)1.0);
			}
		} else {
			const Vector3 left = goal - proxy;
			step = left - p_normal * left.dot(p_normal);
		}
		if (count == 2 || step.length_squared() < 1e-12) {
			goal = proxy;
			return false;
		}
		goal = proxy + step.limit_length(SLIDE);
		planes[count++] = p_normal;
		if (count == 2) {
			const Vector3 crease = planes[0].cross(planes[1]);
			goal = crease.length_squared() > 1e-12 ? proxy + crease.normalized() * (goal - proxy).dot(crease.normalized()) : proxy;
		}
		return true;
	};
	bool moving = true;
	if (touching) {
		// Carried by the face it rests on; still pressed into it, it starts on that face.
		touching = false;
		for (uint32_t k = 0; k < triangles.size(); k++) {
			if (owner[k] == touch_body && triangles[k].id == touch_triangle) {
				const SkinBody::Triangle &t = triangles[k];
				proxy = t.a * touch_weights.x + t.b * touch_weights.y + t.c * touch_weights.z + t.normal * GAP;
				if ((goal - proxy).dot(t.normal) < 0.0) {
					touching = true;
					contacts = 1;
					moving = slide_on(t.normal, p_point - last_point);
				}
				break;
			}
		}
	}
	// The face the segment first enters from outside, and how far along it; -1 for none.
	auto first_entry = [&](const Vector3 &p_from, const Vector3 &p_to, real_t &r_along) -> int {
		const Vector3 d = p_to - p_from;
		int hit = -1;
		r_along = INFINITY;
		for (uint32_t k = 0; k < triangles.size(); k++) {
			real_t along;
			if (d.dot(triangles[k].normal) < 0.0 && crosses(p_from, p_to, triangles[k], along) && along < r_along) {
				r_along = along;
				hit = k;
			}
		}
		return hit;
	};
	Vector3 rest_normal; // of the face it rests on
	auto rest_on = [&](int p_hit, const Vector3 &p_at) {
		const SkinBody::Triangle &t = triangles[p_hit];
		proxy = p_at + t.normal * GAP;
		rest_normal = t.normal;
		touching = true;
		touch_body = owner[p_hit];
		touch_triangle = t.id;
		touch_weights = barycentric(t, p_at);
		contacts = 1;
	};
	bool slid = moving;
	for (int step = 0; moving && step < 4; step++) {
		if (proxy.distance_squared_to(goal) < 1e-12) {
			break;
		}
		real_t along;
		const int hit = first_entry(proxy, goal, along);
		if (hit < 0) {
			proxy = goal;
			break;
		}
		const Vector3 hit_at = proxy + (goal - proxy) * along;
		const Vector3 rest = goal - hit_at; // what is left of this step
		rest_on(hit, hit_at);
		if (!slide_on(rest_normal, rest)) {
			break;
		}
	}
	if (touching && slid) {
		// Slid along her skin: back down onto it, on the face now under it (her arm curves away).
		if (rest_normal.is_zero_approx()) {
			rest_normal = planes[0];
		}
		real_t along;
		const Vector3 from = proxy + rest_normal * GAP;
		const Vector3 to = proxy - rest_normal * (GAP + SLIDE);
		const int hit = first_entry(from, to, along);
		if (hit >= 0) {
			rest_on(hit, from + (to - from) * along);
		} else {
			touching = false; // slid off her
		}
	}
	if (proxy.distance_to(p_point) > max_offset) {
		proxy = p_point; // too far behind the controller: let go, and pass through
		touching = false;
		contacts = 0;
	}
	last_point = p_point;
	last_depth = touching ? depth : 0.0;
	offset = proxy - p_point;
	return offset;
}

void HandContact::reset() {
	started = false;
	touching = false;
	last_depth = 0.0;
	offset = Vector3();
	contacts = 0;
}

void HandContact::_bind_methods() {
	ClassDB::bind_method(D_METHOD("resolve", "bodies", "point"), &HandContact::resolve);
	ClassDB::bind_method(D_METHOD("reset"), &HandContact::reset);
	ClassDB::bind_method(D_METHOD("get_contact_count"), &HandContact::get_contact_count);
	ClassDB::bind_method(D_METHOD("get_offset"), &HandContact::get_offset);
	ClassDB::bind_method(D_METHOD("set_max_offset", "max_offset"), &HandContact::set_max_offset);
	ClassDB::bind_method(D_METHOD("get_max_offset"), &HandContact::get_max_offset);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "max_offset"), "set_max_offset", "get_max_offset");
	ClassDB::bind_method(D_METHOD("set_slide_depth", "depth"), &HandContact::set_slide_depth);
	ClassDB::bind_method(D_METHOD("get_slide_depth"), &HandContact::get_slide_depth);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "slide_depth"), "set_slide_depth", "get_slide_depth");
}
