/**************************************************************************/
/*  skin_contact.h                                                        */
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
#include "core/os/mutex.h"
#include "core/templates/local_vector.h"
#include "core/variant/typed_array.h"

class ImporterMesh;
class MeshInstance3D;
class Skeleton3D;

// A body's skin as the GPU draws it, skinned on the CPU only where asked (near a hand), at most
// once a frame per vertex. Same math as skeleton.glsl: blend shapes first (base + sum w delta),
// then dual quaternions (heaviest bone's hemisphere) or the matrix blend, with 16-bit weights.
class SkinBody : public RefCounted {
	GDCLASS(SkinBody, RefCounted);

	struct Bind {
		int bone = -1;
		Transform3D pose;
		uint32_t surface = 0; // one of its mesh's, for the node transform
	};

	struct Surface {
		ObjectID mesh;
		uint32_t first = 0;
		uint32_t count = 0;
		LocalVector<uint32_t> source; // the mesh's vertex each kept one is
	};

	struct Shape {
		uint32_t surface = 0;
		int index = -1; // the mesh's blend shape
		LocalVector<uint32_t> vertices; // while adding; then per vertex in key_*
		LocalVector<Vector3> deltas;
	};

	// Nearby vertices skinned mostly by one bind: their centre in the mesh's space and how far they reach.
	struct Group {
		int bind = -1;
		Vector3 centre;
		real_t radius = 0.0;
		real_t edge = 0.0; // its vertices' longest triangle edge
		LocalVector<uint32_t> vertices;
	};

	ObjectID skeleton;
	real_t spacing = 0.003;
	LocalVector<Bind> binds;
	LocalVector<Surface> surfaces;
	LocalVector<Shape> shapes;
	LocalVector<Group> groups;
	// Triangles on the kept vertices, 3 per; each vertex's: tri_start[v] to tri_start[v + 1] into tri_of.
	LocalVector<uint32_t> tris;
	LocalVector<uint32_t> tri_start;
	LocalVector<uint32_t> tri_of;
	LocalVector<real_t> vertex_edge; // per vertex, its longest triangle edge, in the mesh's space

	// Per vertex: bind-space position and normal, 8 binds and weights, its surface.
	LocalVector<Vector3> rest;
	LocalVector<Vector3> rest_normals;
	LocalVector<int> joints;
	LocalVector<float> weights;
	LocalVector<uint32_t> surface_of;
	// Per vertex, its keys: key_start[v] to key_start[v + 1] into key_shape (into shapes) and key_delta.
	LocalVector<uint32_t> key_start;
	LocalVector<uint32_t> key_shape;
	LocalVector<Vector3> key_delta;

	// The pose as last drawn (taken on skeleton_updated, while the modifiers' pose is in place:
	// get_bone_global_pose afterwards has it restored to the pose before them): bind transforms and
	// turns, node transforms, key weights. Taken on the main thread into `taken`; a hand's work, on
	// a worker, copies it into `now` first (sync), and reads nothing else of the scene.
	struct Pose {
		uint64_t version = 0;
		bool dual_quaternion = false;
		LocalVector<Transform3D> bind_now;
		LocalVector<Quaternion> bind_turn;
		LocalVector<Transform3D> mesh_now;
		LocalVector<Transform3D> bind_world;
		LocalVector<real_t> bind_scale;
		LocalVector<float> shape_weight;
	};
	Pose taken;
	Pose now;
	Mutex pose_mutex;
	// Per vertex, skinned on demand: the pose version it was skinned at, world position and normal.
	LocalVector<uint64_t> skinned_frame;
	LocalVector<Vector3> morph;
	LocalVector<Vector3> world;
	LocalVector<Vector3> world_normals;

	struct Stats {
		int chunks = 0;
		int tested = 0;
		int skinned = 0;
		int shape_entries = 0;
		int usec = 0;
		int skin_usec = 0;
	} stats; // the last gather's

	Skeleton3D *_skeleton() const;
	void _take_pose();
	void _split(int p_bind, const LocalVector<uint32_t> &p_vertices, LocalVector<Group> &r_chunks) const;
	Vector3 _skin(uint32_t p_vertex, const Vector3 &p_at, Vector3 &r_normal) const;

protected:
	static void _bind_methods();

public:
	struct Triangle {
		uint32_t id = 0;
		Vector3 a, b, c;
		Vector3 normal; // outward
	};

	// From `p_source` when given (the load's ImporterMesh, so a loader thread need not read the
	// mesh back from the RenderingServer); its surfaces are the mesh's.
	bool add_mesh(MeshInstance3D *p_mesh, const PackedInt32Array &p_surfaces, const Ref<ImporterMesh> &p_source = Ref<ImporterMesh>());
	// The pose last drawn, for the gathers that follow; on the thread doing them.
	void sync();
	int get_vertex_count() const { return rest.size(); }
	// Set before add_mesh: one vertex is kept per cell this wide (and normal direction); 0 keeps all.
	void set_spacing(real_t p_spacing) { spacing = p_spacing; }
	real_t get_spacing() const { return spacing; }

	// World positions and normals of the vertices within reach of any segment, appended.
	// With r_near, the indices of every vertex of a triangle that may come within reach, instead.
	void gather(const LocalVector<Vector3> &p_a, const LocalVector<Vector3> &p_b, const LocalVector<real_t> &p_reach, LocalVector<Vector3> &r_positions, LocalVector<Vector3> &r_normals, LocalVector<uint32_t> *r_near = nullptr);
	// The triangles that may cross the segment or lie within p_around of p_from, in the world, appended.
	void gather_triangles(const Vector3 &p_from, const Vector3 &p_to, real_t p_around, LocalVector<Triangle> &r_triangles);
	Array gather_near(const Vector3 &p_centre, real_t p_radius);
	// Every vertex of one added surface skinned, in the mesh's space (no node transform), as the
	// GPU's buffer holds it, for checks against RenderingServer.instance_debug_get_skinned_vertices.
	// The kept vertices of a surface, in the mesh's order (the mesh's vertex each is).
	PackedVector3Array debug_skin_surface(int p_surface);
	PackedInt32Array debug_surface_vertices(int p_surface) const;
	Dictionary get_stats() const;
};

// Keeps a hand point on the outside of the skin it touches. Given where the controller puts the
// point, returns how far to move the hand: the point slides over her triangles, held on their
// outer side, and a point already inside comes out freely.
class HandContact : public RefCounted {
	GDCLASS(HandContact, RefCounted);

	bool started = false;
	Vector3 proxy; // where the point is drawn
	bool touching = false;
	ObjectID touch_body; // the SkinBody, triangle and spot on it the point rests on
	uint32_t touch_triangle = 0;
	Vector3 touch_weights;
	real_t max_offset = 0.3;
	real_t slide_depth = 0.02; // pressed this far in, the point slides half as far as the controller
	Vector3 last_point; // the controller's point, and its depth, last frame
	real_t last_depth = 0.0;
	int contacts = 0;
	Vector3 offset;

protected:
	static void _bind_methods();

public:
	void set_max_offset(real_t p_max) { max_offset = p_max; }
	real_t get_max_offset() const { return max_offset; }
	void set_slide_depth(real_t p_depth) { slide_depth = MAX(p_depth, (real_t)1e-4); }
	real_t get_slide_depth() const { return slide_depth; }
	int get_contact_count() const { return contacts; }
	Vector3 get_offset() const { return offset; }

	Vector3 resolve(const Array &p_bodies, const Vector3 &p_point);
	void reset();
};
