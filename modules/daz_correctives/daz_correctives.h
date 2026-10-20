/**************************************************************************/
/*  daz_correctives.h                                                     */
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

#include "core/templates/hash_map.h"
#include "core/templates/local_vector.h"
#include "core/templates/pair.h"
#include "scene/3d/skeleton_modifier_3d.h"

class MeshInstance3D;

// DAZ's pose-driven correctives. The last modifier on a DAZ figure's skeleton: each frame it reads
// each driving bone's DAZ channels from the pose, runs DAZ's formulas from the spec given
// to setup(), sets the corrective keys' weights, and turns and moves the bones DAZ drives from
// other bones and from the expressions on top of the pose.
//
// The bind is DAZ's unbent pose, so a bone's skinning transform (global pose x bind) is DAZ's move
// from it in the mesh's space. DAZ's angles: a bone's turn from its DAZ parent, E = O^T (R_p^T R) O
// with O its joint orientation, as Euler angles in its rotation order (DAZ composes them
// extrinsically in the listed order). Its translation v (cm): the bone sits at
// t = t_p + R_p (c2 - c2_p + S_p v), c2 the scaled centres.
//
// The spec:
//   bones     {dsf id: {node, parent, orientation [x,y,z,w], order, limits, parent_scale}}; node and
//             parent are the skeleton's bone names (parent: DAZ's parent); limits DAZ's [min, max]
//             per axis x y z (null where free), which pick the turn's reading; parent_scale (3 rows
//             of 3) only on bones whose translation the formulas set
//   channels  {channel: {own, sum: [program], mult: [program], clamp: [min, max] | null, rest}}:
//             (own + sums) x mults, clamped. A bone channel ("<id>?rotation/x" degrees,
//             "<id>?translation/x" cm) with `rest` is one the formulas set: its own gets the
//             animation's change since the saved pose, (the pose's value) - rest. A bone channel
//             not listed: the pose's. A program: a channel name, a number, "mult", or
//             {"spline": [[x, y], ...]} (DAZ's spline_tcb with t = c = b = 0).
//   keys      {shape key: channel}
//   inputs    {channel: {offset, terms: [[shape key, scale], ...]}}: an expression's controller,
//             offset + sum(scale x the key's weight this frame) in place of own.
//   centres   {shape key: {node: [x, y, z]}}: how far the key at 1 moves that bone's pivot.
//   followers [{bone, target, refs: [{at, joints: [[bone, weight]], keys: [[shape key, delta]]}], saved}]:
//             rigid follow nodes: each bone rides its target mesh's reference vertices (skinned as
//             the GPU does, dual quaternions, with their keys) by their rigid turn from where they
//             sit at the saved pose. After set_followers_after(m), while m is active, they move
//             only on update_followers() (call it once m has run).
class DazCorrectives : public SkeletonModifier3D {
	GDCLASS(DazCorrectives, SkeletonModifier3D);

	struct Op {
		enum Kind {
			PUSH_CHANNEL,
			PUSH_NUMBER,
			MULT,
			SPLINE,
		};
		Kind kind = PUSH_NUMBER;
		int index = 0; // channel or spline
		double number = 0.0;
	};
	typedef LocalVector<Op> Program;

	struct Spline {
		LocalVector<double> xs;
		LocalVector<double> ys;
	};

	struct Term {
		int shape = -1;
		double scale = 0.0;
	};

	struct Channel {
		String name;
		double own = NAN; // NAN: the pose's
		LocalVector<Program> sum;
		LocalVector<Program> mult;
		bool clamped = false;
		double clamp_min = 0.0;
		double clamp_max = 0.0;
		double rest = NAN; // NAN: not one the formulas set
		bool input = false;
		double offset = 0.0;
		LocalVector<Term> terms; // on the body
		int bone = -1; // into bones, when it is a bone channel of a listed bone
		int axis = 0;
		int kind = 0; // 0 rotation, 1 translation
	};

	struct Bone {
		int bone = -1;
		int parent = -1; // DAZ's parent (-1: the mesh's space)
		Basis o;
		Basis o_inv;
		EulerOrder order = EulerOrder::XYZ;
		Basis parent_scale;
		Basis parent_scale_inv;
		Vector3 c2;
		Vector3 c2_parent;
		bool limited[3] = { false, false, false };
		Vector2 limits[3];
		int middle = 0;
	};

	struct Key {
		ObjectID mesh;
		int shape = -1;
		int channel = -1;
	};

	struct Output {
		int bone = -1; // into bones
		int rotations[3] = { -1, -1, -1 };
		int translations[3] = { -1, -1, -1 };
	};

	struct Centre {
		int channel = -1;
		int bone = -1;
		Vector3 shift;
	};

	struct Anchor {
		Vector3 at;
		LocalVector<int> joints; // heaviest first
		LocalVector<float> weights;
	};

	struct Follower {
		int bone = -1;
		Transform3D saved_global;
		LocalVector<Anchor> refs;
		LocalVector<Pair<int, LocalVector<Vector3>>> keys; // blend shape -> delta per ref
		LocalVector<Vector3> saved;
		Vector3 centre;
		ObjectID target;
	};

	ObjectID skeleton;
	ObjectID body;
	ObjectID followers_after;
	bool ready = false;

	LocalVector<Channel> channels;
	HashMap<String, int> channel_index;
	LocalVector<Spline> splines;
	LocalVector<Bone> bones;
	HashMap<String, int> bone_index;
	LocalVector<bool> has_bind; // per skeleton bone
	LocalVector<Transform3D> binds;
	LocalVector<Transform3D> bind_inverses;
	LocalVector<Key> keys;
	LocalVector<Output> outputs;
	LocalVector<Centre> centres;
	LocalVector<Follower> followers;

	// This frame's: values per channel, DAZ angles (degrees) and translation (cm) per bone as posed now.
	uint32_t frame = 1;
	LocalVector<double> values;
	LocalVector<uint32_t> value_frames;
	LocalVector<Vector3> posed_angles;
	LocalVector<Vector3> posed_moves;
	LocalVector<uint32_t> posed_frames;

	int _channel(const String &p_name);
	bool _compile(const Array &p_programs, LocalVector<Program> &r_out);
	bool _add_follower(Node *p_avatar, Skeleton3D *p_skeleton, const Dictionary &p_f);
	void _stay_last();
	void _child_entered(Node *p_node);

	Transform3D _skinning(Skeleton3D *p_skeleton, int p_bone) const;
	void _channels_of(Skeleton3D *p_skeleton, int p_bone);
	double _value(Skeleton3D *p_skeleton, int p_channel);
	double _run(Skeleton3D *p_skeleton, const Program &p_program);
	void _set_bone(Skeleton3D *p_skeleton, const Output &p_output);
	void _apply_centres(Skeleton3D *p_skeleton);
	void _follow(Skeleton3D *p_skeleton);
	void _refs_now(Skeleton3D *p_skeleton, const Follower &p_f, HashMap<int, Pair<Quaternion, Transform3D>> &r_dq, LocalVector<Vector3> &r_now) const;

	static Vector3 _nearer_reading(const Vector3 &p_angles, const Bone &p_bone);
	static double _outside(const Vector3 &p_angles, const Bone &p_bone);
	static double _spline(double p_x, const Spline &p_spline);
	static Quaternion _quat_from_rows(const Basis &p_basis);
	static Quaternion _horn(const Vector3 &p_x, const Vector3 &p_y, const Vector3 &p_z);

protected:
	static void _bind_methods();
	virtual void _process_modification(double p_delta) override;

public:
	static MeshInstance3D *find_body(Node *p_avatar, const Dictionary &p_keys);

	bool setup(Node *p_avatar, Skeleton3D *p_skeleton, const Dictionary &p_spec);
	Dictionary get_values() const;
	PackedInt32Array get_driven_bones() const;
	Array get_driven_keys() const;
	void set_followers_after(SkeletonModifier3D *p_modifier);
	void update_followers();
};
