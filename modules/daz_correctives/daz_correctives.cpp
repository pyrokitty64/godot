/**************************************************************************/
/*  daz_correctives.cpp                                                   */
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

#include "daz_correctives.h"

#include "scene/3d/mesh_instance_3d.h"
#include "scene/3d/skeleton_3d.h"
#include "scene/resources/3d/skin.h"

#include "core/object/callable_mp.h"
#include "core/object/class_db.h"
#include "core/templates/sort_array.h"

// DAZ's order "XYZ" is E = Rz Ry Rx (extrinsic X, then Y, then Z); Godot's ZYX composes Rz Ry Rx.
// So Godot's order is DAZ's string reversed.
static bool daz_order(const String &p_order, EulerOrder &r_order) {
	static const char *names[6] = { "XYZ", "XZY", "YXZ", "YZX", "ZXY", "ZYX" };
	static const EulerOrder orders[6] = { EulerOrder::ZYX, EulerOrder::YZX, EulerOrder::ZXY, EulerOrder::XZY, EulerOrder::YXZ, EulerOrder::XYZ };
	for (int i = 0; i < 6; i++) {
		if (p_order == names[i]) {
			r_order = orders[i];
			return true;
		}
	}
	return false;
}

// "lForeArm?rotation/y" -> lForeArm, 1, 0; "lowerJaw?translation/z" -> lowerJaw, 2, 1.
static bool bone_axis(const String &p_channel, String &r_id, int &r_axis, int &r_kind) {
	static const char *tags[2] = { "?rotation/", "?translation/" };
	bool found = false;
	for (int kind = 0; kind < 2; kind++) {
		const String tag = tags[kind];
		const int at = p_channel.find(tag);
		if (at >= 0) {
			r_id = p_channel.substr(0, at);
			r_axis = String("xyz").find(p_channel.substr(at + tag.length()));
			r_kind = kind;
			found = true;
		}
	}
	return found;
}

static Vector3 vec3(const Variant &p_array) {
	const Array a = p_array;
	return Vector3(real_t(a[0]), real_t(a[1]), real_t(a[2]));
}

static int skeleton_depth(const Skeleton3D *p_skeleton, int p_bone) {
	int d = 0;
	while (p_bone >= 0) {
		p_bone = p_skeleton->get_bone_parent(p_bone);
		d++;
	}
	return d;
}

static int bind_bone(const Skeleton3D *p_skeleton, const Ref<Skin> &p_skin, int p_index) {
	const StringName name = p_skin->get_bind_name(p_index);
	return name != StringName() ? p_skeleton->find_bone(name) : p_skin->get_bind_bone(p_index);
}

int DazCorrectives::_channel(const String &p_name) {
	if (const int *index = channel_index.getptr(p_name)) {
		return *index;
	}
	const int index = channels.size();
	channels.push_back(Channel());
	channels[index].name = p_name;
	channel_index.insert(p_name, index);
	return index;
}

// The spec's programs as ops once, not strings each frame.
bool DazCorrectives::_compile(const Array &p_programs, LocalVector<Program> &r_out) {
	for (const Variant &program_v : p_programs) {
		const Array program = program_v;
		Program ops;
		int depth = 0;
		for (const Variant &v : program) {
			Op op;
			if (v.get_type() == Variant::STRING && String(v) == "mult") {
				op.kind = Op::MULT;
				depth--;
			} else if (v.get_type() == Variant::STRING) {
				op.kind = Op::PUSH_CHANNEL;
				op.index = _channel(v);
				depth++;
			} else if (v.get_type() == Variant::DICTIONARY) {
				Spline spline;
				const Array knots = Dictionary(v).get("spline", Array());
				for (const Variant &knot : knots) {
					spline.xs.push_back(double(Array(knot)[0]));
					spline.ys.push_back(double(Array(knot)[1]));
				}
				ERR_FAIL_COND_V_MSG(spline.xs.is_empty(), false, "[correctives] a spline without knots");
				op.kind = Op::SPLINE;
				op.index = splines.size();
				splines.push_back(spline);
			} else {
				op.kind = Op::PUSH_NUMBER;
				op.number = v;
				depth++;
			}
			ERR_FAIL_COND_V_MSG(depth < 1 || depth > 64, false, "[correctives] a program's stack is out of bounds");
			ops.push_back(op);
		}
		ERR_FAIL_COND_V_MSG(depth < 1, false, "[correctives] a program leaves nothing");
		r_out.push_back(ops);
	}
	return true;
}

// The body: the mesh with the most of the spec's keys (worn items carry some under its names).
MeshInstance3D *DazCorrectives::find_body(Node *p_avatar, const Dictionary &p_keys) {
	ERR_FAIL_NULL_V(p_avatar, nullptr);
	MeshInstance3D *body = nullptr;
	int most = 0;
	const Array names = p_keys.keys();
	for (const Variant &node : p_avatar->find_children("*", "MeshInstance3D", true, false)) {
		MeshInstance3D *mesh = Object::cast_to<MeshInstance3D>(node);
		if (mesh == nullptr || mesh->get_mesh().is_null()) {
			continue;
		}
		int count = 0;
		for (const Variant &key : names) {
			if (mesh->find_blend_shape_by_name(key) >= 0) {
				count++;
			}
		}
		if (count > most) {
			body = mesh;
			most = count;
		}
	}
	return body;
}

// False (and a warning) when the spec or the avatar doesn't fit; the node then does nothing.
bool DazCorrectives::setup(Node *p_avatar, Skeleton3D *p_skeleton, const Dictionary &p_spec) {
	ERR_FAIL_NULL_V(p_avatar, false);
	ERR_FAIL_NULL_V(p_skeleton, false);
	ERR_FAIL_COND_V_MSG(ready, false, "[correctives] set up once");
	skeleton = p_skeleton->get_instance_id();
	const String who = p_avatar->get_name();
	const Dictionary key_spec = p_spec.get("keys", Dictionary());
	const Array key_names = key_spec.keys();
	MeshInstance3D *b = find_body(p_avatar, key_spec);
	for (const Variant &node : p_avatar->find_children("*", "MeshInstance3D", true, false)) {
		MeshInstance3D *mesh = Object::cast_to<MeshInstance3D>(node);
		if (mesh == nullptr || mesh->get_mesh().is_null()) {
			continue;
		}
		for (const Variant &key : key_names) {
			const int shape = mesh->find_blend_shape_by_name(key);
			if (shape >= 0) {
				keys.push_back({ mesh->get_instance_id(), shape, _channel(key_spec[key]) });
			}
		}
	}
	if (b == nullptr || b->get_skin().is_null()) {
		WARN_PRINT(vformat("[correctives] %s: no skinned mesh has the spec's keys", who));
		return false;
	}
	body = b->get_instance_id();

	const Dictionary inputs = p_spec.get("inputs", Dictionary());
	for (const Variant &name : inputs.keys()) {
		const Dictionary input = inputs[name];
		LocalVector<Term> terms;
		for (const Variant &term_v : Array(input.get("terms", Array()))) {
			const Array term = term_v;
			const int shape = b->find_blend_shape_by_name(term[0]);
			if (shape < 0) {
				WARN_PRINT(vformat("[correctives] %s: no control key %s for %s", who, term[0], name));
				return false;
			}
			terms.push_back({ shape, double(term[1]) });
		}
		Channel &c = channels[_channel(name)];
		c.input = true;
		c.offset = input.get("offset", 0.0);
		c.terms = terms;
	}

	const int bone_count = p_skeleton->get_bone_count();
	has_bind.resize(bone_count);
	binds.resize(bone_count);
	bind_inverses.resize(bone_count);
	for (int i = 0; i < bone_count; i++) {
		has_bind[i] = false;
	}
	const Ref<Skin> skin = b->get_skin();
	for (int i = 0; i < skin->get_bind_count(); i++) {
		const int bone = bind_bone(p_skeleton, skin, i);
		if (bone >= 0 && bone < bone_count) {
			has_bind[bone] = true;
			binds[bone] = skin->get_bind_pose(i);
			bind_inverses[bone] = binds[bone].affine_inverse();
		}
	}

	const Dictionary bone_spec = p_spec.get("bones", Dictionary());
	for (const Variant &id : bone_spec.keys()) {
		const Dictionary d = bone_spec[id];
		const String node = d.get("node", "");
		const String parent_name = d.get("parent", "");
		const String order = d.get("order", "");
		Bone bone;
		bone.bone = p_skeleton->find_bone(node);
		bone.parent = parent_name.is_empty() ? -1 : p_skeleton->find_bone(parent_name);
		if (!parent_name.is_empty() && bone.parent < 0) {
			WARN_PRINT(vformat("[correctives] %s: no parent %s for %s (a bone the .vrm dropped?)", who, parent_name, id));
			return false;
		}
		if (bone.bone < 0 || !has_bind[bone.bone] || (bone.parent >= 0 && !has_bind[bone.parent]) || !daz_order(order, bone.order)) {
			WARN_PRINT(vformat("[correctives] %s: no bone, bind or order for %s (%s)", who, id, node));
			return false;
		}
		const Array o = d["orientation"];
		bone.o = Basis(Quaternion(real_t(o[0]), real_t(o[1]), real_t(o[2]), real_t(o[3])));
		bone.o_inv = bone.o.inverse();
		if (d.has("parent_scale")) {
			const Array m = d["parent_scale"]; // rows
			for (int i = 0; i < 3; i++) {
				bone.parent_scale.rows[i] = vec3(m[i]);
			}
		}
		bone.parent_scale_inv = bone.parent_scale.inverse();
		bone.c2 = bind_inverses[bone.bone].origin;
		bone.c2_parent = bone.parent >= 0 ? bind_inverses[bone.parent].origin : Vector3();
		const Array limits = d.get("limits", Array());
		for (int i = 0; i < 3 && i < limits.size(); i++) {
			if (limits[i].get_type() == Variant::ARRAY) {
				bone.limited[i] = true;
				bone.limits[i] = Vector2(real_t(Array(limits[i])[0]), real_t(Array(limits[i])[1]));
			}
		}
		bone.middle = String("xyz").find(order.substr(1, 1).to_lower());
		bone_index.insert(id, bones.size());
		bones.push_back(bone);
	}

	HashMap<int, Output> by_bone;
	LocalVector<int> output_bones;
	const Dictionary channel_spec = p_spec.get("channels", Dictionary());
	for (const Variant &name : channel_spec.keys()) {
		const Dictionary d = channel_spec[name];
		const int index = _channel(name);
		LocalVector<Program> sum;
		LocalVector<Program> mult;
		if (!_compile(d.get("sum", Array()), sum) || !_compile(d.get("mult", Array()), mult)) {
			return false;
		}
		Channel &c = channels[index];
		c.sum = sum;
		c.mult = mult;
		const Variant own = d.get("own", Variant());
		c.own = own.get_type() == Variant::NIL ? NAN : double(own);
		const Variant rest = d.get("rest", Variant());
		c.rest = rest.get_type() == Variant::NIL ? NAN : double(rest);
		const Variant clamp = d.get("clamp", Variant());
		if (clamp.get_type() == Variant::ARRAY) {
			c.clamped = true;
			c.clamp_min = Array(clamp)[0];
			c.clamp_max = Array(clamp)[1];
		}
		String id;
		int axis = 0;
		int kind = 0;
		if (bone_axis(name, id, axis, kind) && !Math::is_nan(c.rest)) {
			const int *bone = bone_index.getptr(id);
			if (bone == nullptr || axis < 0) {
				WARN_PRINT(vformat("[correctives] %s: %s sets a bone the spec's bones don't list", who, name));
				return false;
			}
			if (!by_bone.has(*bone)) {
				by_bone.insert(*bone, Output());
				by_bone[*bone].bone = *bone;
				output_bones.push_back(*bone);
			}
			(kind == 0 ? by_bone[*bone].rotations : by_bone[*bone].translations)[axis] = index;
		}
	}
	// Parents first: a bone's DAZ parent has to be final before its own channels are set.
	struct ByDepth {
		const Skeleton3D *skeleton;
		const LocalVector<Bone> *bones;
		bool operator()(int a, int b) const {
			return skeleton_depth(skeleton, (*bones)[a].bone) < skeleton_depth(skeleton, (*bones)[b].bone);
		}
	};
	SortArray<int, ByDepth> sorter;
	sorter.compare = { p_skeleton, &bones };
	sorter.sort(output_bones.ptr(), output_bones.size());
	for (int bone : output_bones) {
		outputs.push_back(by_bone[bone]);
	}

	const Dictionary centre_spec = p_spec.get("centres", Dictionary());
	for (const Variant &key : centre_spec.keys()) {
		const Dictionary shifts = centre_spec[key];
		for (const Variant &node : shifts.keys()) {
			const int bone = p_skeleton->find_bone(node);
			if (bone >= 0 && key_spec.has(key)) {
				centres.push_back({ _channel(key_spec[key]), bone, vec3(shifts[node]) });
			}
		}
	}

	for (const Variant &f : Array(p_spec.get("followers", Array()))) {
		if (!_add_follower(p_avatar, p_skeleton, f)) {
			WARN_PRINT(vformat("[correctives] %s: follower %s doesn't fit the avatar; it rides its parent bone", who, Dictionary(f).get("bone", "")));
		}
	}

	// A channel names a listed bone's channel: it reads the pose (all channels are known now).
	for (Channel &c : channels) {
		String id;
		if (bone_axis(c.name, id, c.axis, c.kind) && c.axis >= 0) {
			const int *bone = bone_index.getptr(id);
			c.bone = bone != nullptr ? *bone : -1;
		}
	}
	values.resize(channels.size());
	value_frames.resize(channels.size());
	for (uint32_t &f : value_frames) {
		f = 0;
	}
	posed_angles.resize(bones.size());
	posed_moves.resize(bones.size());
	posed_frames.resize(bones.size());
	for (uint32_t &f : posed_frames) {
		f = 0;
	}

	// Last among the skeleton's modifiers, which run in child order: keep moving behind any added later.
	p_skeleton->connect("child_entered_tree", callable_mp(this, &DazCorrectives::_child_entered));
	_stay_last();
	ready = true;
	return true;
}

void DazCorrectives::_stay_last() {
	Skeleton3D *s = ObjectDB::get_instance<Skeleton3D>(skeleton);
	if (s != nullptr && is_inside_tree() && get_parent() == s && get_index(false) != s->get_child_count(false) - 1) {
		s->move_child(this, -1);
	}
}

void DazCorrectives::_child_entered(Node *p_node) {
	callable_mp(this, &DazCorrectives::_stay_last).call_deferred();
}

// Followers wait for this modifier while it's active (one that runs after this one and moves what
// they ride); then update_followers() moves them. Null: they move in this modifier's pass.
void DazCorrectives::set_followers_after(SkeletonModifier3D *p_modifier) {
	followers_after = p_modifier != nullptr ? p_modifier->get_instance_id() : ObjectID();
}

void DazCorrectives::update_followers() {
	Skeleton3D *s = ObjectDB::get_instance<Skeleton3D>(skeleton);
	if (ready && s != nullptr) {
		_follow(s);
	}
}

void DazCorrectives::_process_modification(double p_delta) {
	Skeleton3D *s = ObjectDB::get_instance<Skeleton3D>(skeleton);
	if (!ready || s == nullptr || keys.is_empty()) {
		return;
	}
	frame = frame == UINT32_MAX ? 1 : frame + 1;
	for (const Key &k : keys) {
		const double v = _value(s, k.channel);
		if (MeshInstance3D *mesh = ObjectDB::get_instance<MeshInstance3D>(k.mesh)) {
			mesh->set_blend_shape_value(k.shape, v);
		}
	}
	for (const Output &output : outputs) {
		_set_bone(s, output);
	}
	_apply_centres(s);
	SkeletonModifier3D *sp = ObjectDB::get_instance<SkeletonModifier3D>(followers_after);
	if (sp == nullptr || !sp->is_active()) {
		_follow(s);
	}
}

// A `followers` entry made ready; false if its bone, bind, target or a joint is missing.
bool DazCorrectives::_add_follower(Node *p_avatar, Skeleton3D *p_skeleton, const Dictionary &p_f) {
	Follower f;
	f.bone = p_skeleton->find_bone(p_f.get("bone", ""));
	const String target_name = p_f.get("target", "");
	MeshInstance3D *target = target_name.is_empty() ? nullptr : Object::cast_to<MeshInstance3D>(p_avatar->find_child(target_name, true, false));
	if (f.bone < 0 || target == nullptr || target->get_mesh().is_null()) {
		return false;
	}
	f.target = target->get_instance_id();
	bool found = false;
	for (const Variant &node : p_avatar->find_children("*", "MeshInstance3D", true, false)) { // its bind is its saved pose
		MeshInstance3D *mesh = Object::cast_to<MeshInstance3D>(node);
		const Ref<Skin> skin = mesh != nullptr ? mesh->get_skin() : Ref<Skin>();
		for (int i = 0; skin.is_valid() && i < skin->get_bind_count(); i++) {
			if (bind_bone(p_skeleton, skin, i) == f.bone) {
				f.saved_global = skin->get_bind_pose(i).affine_inverse();
				found = true;
			}
		}
	}
	if (!found) {
		return false;
	}
	const Array refs = p_f.get("refs", Array());
	for (int i = 0; i < refs.size(); i++) {
		const Dictionary r = refs[i];
		Anchor ref;
		ref.at = vec3(r["at"]);
		for (const Variant &j : Array(r["joints"])) {
			const int b = p_skeleton->find_bone(Array(j)[0]);
			if (b < 0 || !has_bind[b]) {
				return false;
			}
			ref.joints.push_back(b);
			ref.weights.push_back(float(Array(j)[1]));
		}
		f.refs.push_back(ref);
		for (const Variant &k : Array(r.get("keys", Array()))) {
			const int shape = target->find_blend_shape_by_name(Array(k)[0]);
			if (shape < 0) {
				return false;
			}
			int at = -1;
			for (uint32_t n = 0; n < f.keys.size(); n++) {
				if (f.keys[n].first == shape) {
					at = n;
				}
			}
			if (at < 0) {
				at = f.keys.size();
				f.keys.push_back(Pair<int, LocalVector<Vector3>>(shape, LocalVector<Vector3>()));
				f.keys[at].second.resize_initialized(refs.size());
			}
			f.keys[at].second[i] = vec3(Array(k)[1]);
		}
	}
	for (const Variant &v : Array(p_f.get("saved", Array()))) {
		f.saved.push_back(vec3(v));
		f.centre += f.saved[f.saved.size() - 1];
	}
	if (refs.is_empty() || int(f.saved.size()) != refs.size()) {
		return false;
	}
	f.centre /= f.saved.size();
	followers.push_back(f);
	return true;
}

// Each follower's bone onto its target's reference vertices as skinned now: their rigid turn from
// the saved pose (_horn()).
void DazCorrectives::_follow(Skeleton3D *p_skeleton) {
	HashMap<int, Pair<Quaternion, Transform3D>> dq;
	LocalVector<Pair<int, Transform3D>> poses; // set after every read, as a set makes the next read redo the skeleton
	LocalVector<Vector3> now;
	for (const Follower &f : followers) {
		_refs_now(p_skeleton, f, dq, now);
		Vector3 centre;
		for (const Vector3 &v : now) {
			centre += v;
		}
		centre /= now.size();
		Vector3 cols[3]; // sum of (now - centre)(saved - c0)^T
		for (uint32_t i = 0; i < now.size(); i++) {
			const Vector3 a = f.saved[i] - f.centre;
			const Vector3 b = now[i] - centre;
			cols[0] += b * a.x;
			cols[1] += b * a.y;
			cols[2] += b * a.z;
		}
		const Quaternion turn = _horn(cols[0], cols[1], cols[2]);
		const Transform3D parent = p_skeleton->get_bone_global_pose(p_skeleton->get_bone_parent(f.bone)); // one of hers, never a follower
		poses.push_back(Pair<int, Transform3D>(f.bone, parent.affine_inverse() * Transform3D(Basis(turn), centre - turn.xform(f.centre)) * f.saved_global));
	}
	for (const Pair<int, Transform3D> &p : poses) {
		p_skeleton->set_bone_pose(p.first, p.second);
	}
}

// A follower's reference vertices as the GPU skins its target now: keys as set this frame, then dual
// quaternions in the heaviest joint's hemisphere. `r_dq`: bone -> [real, skinning], filled once a frame.
void DazCorrectives::_refs_now(Skeleton3D *p_skeleton, const Follower &p_f, HashMap<int, Pair<Quaternion, Transform3D>> &r_dq, LocalVector<Vector3> &r_now) const {
	r_now.resize(p_f.refs.size());
	for (uint32_t i = 0; i < p_f.refs.size(); i++) {
		r_now[i] = p_f.refs[i].at;
	}
	MeshInstance3D *target = ObjectDB::get_instance<MeshInstance3D>(p_f.target);
	for (const Pair<int, LocalVector<Vector3>> &k : p_f.keys) {
		const double w = target != nullptr ? target->get_blend_shape_value(k.first) : 0.0;
		if (w != 0.0) {
			for (uint32_t i = 0; i < r_now.size(); i++) {
				r_now[i] += k.second[i] * w;
			}
		}
	}
	for (uint32_t i = 0; i < r_now.size(); i++) {
		const Anchor &ref = p_f.refs[i];
		Quaternion real(0, 0, 0, 0);
		Quaternion dual(0, 0, 0, 0);
		Quaternion lead;
		for (uint32_t k = 0; k < ref.joints.size(); k++) {
			const int j = ref.joints[k];
			if (!r_dq.has(j)) {
				const Transform3D m = _skinning(p_skeleton, j);
				r_dq.insert(j, Pair<Quaternion, Transform3D>(_quat_from_rows(m.basis), m));
			}
			const Pair<Quaternion, Transform3D> &e = r_dq[j];
			const Quaternion q = e.first;
			if (k == 0) {
				lead = q;
			}
			const double w = lead.dot(q) >= 0.0 ? ref.weights[k] : -ref.weights[k];
			// Its move: where this bone alone puts the ref, less its turn (a squash's scale lands here).
			const Vector3 d = e.second.xform(r_now[i]) - q.xform(r_now[i]);
			real += q * w;
			dual += Quaternion(d.x, d.y, d.z, 0.0) * q * (0.5 * w);
		}
		const double n = real.length();
		real /= n;
		const Quaternion t = ((dual / n) * real.inverse()) * 2.0;
		r_now[i] = real.xform(r_now[i]) + Vector3(t.x, t.y, t.z);
	}
}

// The rotation that best takes the saved refs onto now's, given cols[j] = sum (now - centre) (saved -
// c0)[j]: Horn's (1987) quaternion, the eigenvector of the largest eigenvalue of his 4x4 matrix, here
// by Jacobi rotations. Exact however thin the refs (a cuff's 4 lie in a strip 7 x 0.7 mm).
Quaternion DazCorrectives::_horn(const Vector3 &p_x, const Vector3 &p_y, const Vector3 &p_z) {
	const double xx = p_x.x, xy = p_x.y, xz = p_x.z; // Horn's S_ij = sum saved_i now_j = cols[i][j]
	const double yx = p_y.x, yy = p_y.y, yz = p_y.z;
	const double zx = p_z.x, zy = p_z.y, zz = p_z.z;
	double n[16] = { // row-major 4x4, symmetric
		xx + yy + zz, yz - zy, zx - xz, xy - yx,
		yz - zy, xx - yy - zz, xy + yx, zx + xz,
		zx - xz, xy + yx, -xx + yy - zz, yz + zy,
		xy - yx, zx + xz, yz + zy, -xx - yy + zz
	};
	double v[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1 };
	double size = 0.0;
	for (double e : n) {
		size += e * e;
	}
	static const int upper[6] = { 1, 2, 3, 6, 7, 11 };
	for (int sweep = 0; sweep < 12; sweep++) { // 2-3 do
		double off = 0.0;
		for (int pq : upper) {
			off += n[pq] * n[pq];
		}
		if (off <= 1e-24 * size) { // off the diagonal 1e-12 of it: the turn to ~1e-12 rad
			break;
		}
		for (int p = 0; p < 3; p++) {
			for (int q = p + 1; q < 4; q++) {
				const double npq = n[4 * p + q];
				if (Math::abs(npq) < 1e-300) {
					continue;
				}
				const double theta = (n[5 * q] - n[5 * p]) / (2.0 * npq);
				const double t = theta != 0.0 ? SIGN(theta) / (Math::abs(theta) + Math::sqrt(theta * theta + 1.0)) : 1.0;
				const double c = 1.0 / Math::sqrt(t * t + 1.0);
				const double sn = t * c;
				for (int k = 0; k < 4; k++) { // N <- J^T N J, columns then rows; V <- V J
					const double a = n[4 * k + p];
					const double b = n[4 * k + q];
					n[4 * k + p] = c * a - sn * b;
					n[4 * k + q] = sn * a + c * b;
				}
				for (int k = 0; k < 4; k++) {
					const double a = n[4 * p + k];
					const double b = n[4 * q + k];
					n[4 * p + k] = c * a - sn * b;
					n[4 * q + k] = sn * a + c * b;
				}
				for (int k = 0; k < 4; k++) {
					const double a = v[4 * k + p];
					const double b = v[4 * k + q];
					v[4 * k + p] = c * a - sn * b;
					v[4 * k + q] = sn * a + c * b;
				}
			}
		}
	}
	int best = 0;
	for (int k = 0; k < 4; k++) {
		if (n[5 * k] > n[5 * best]) {
			best = k;
		}
	}
	return Quaternion(v[4 + best], v[8 + best], v[12 + best], v[best]).normalized();
}

// The bone's skinning transform: DAZ's move from the unbent pose, in the mesh's space.
Transform3D DazCorrectives::_skinning(Skeleton3D *p_skeleton, int p_bone) const {
	return p_skeleton->get_bone_global_pose(p_bone) * binds[p_bone];
}

// A skinning transform's turn as the shader reads it (skeleton.glsl quat_from_rows): for a scaled
// bone this differs from get_rotation_quaternion(), and the refs must land where the GPU draws them.
Quaternion DazCorrectives::_quat_from_rows(const Basis &p_basis) {
	const Vector3 &r0 = p_basis.rows[0];
	const Vector3 &r1 = p_basis.rows[1];
	const Vector3 &r2 = p_basis.rows[2];
	const double trace = double(r0.x) + r1.y + r2.z;
	Quaternion q;
	if (trace > 0.0) {
		const double s = 0.5 / Math::sqrt(trace + 1.0);
		q = Quaternion((r2.y - r1.z) * s, (r0.z - r2.x) * s, (r1.x - r0.y) * s, 0.25 / s);
	} else if (r0.x > r1.y && r0.x > r2.z) {
		const double s = 2.0 * Math::sqrt(1.0 + r0.x - r1.y - r2.z);
		q = Quaternion(0.25 * s, (r0.y + r1.x) / s, (r0.z + r2.x) / s, (r2.y - r1.z) / s);
	} else if (r1.y > r2.z) {
		const double s = 2.0 * Math::sqrt(1.0 + r1.y - r0.x - r2.z);
		q = Quaternion((r0.y + r1.x) / s, 0.25 * s, (r1.z + r2.y) / s, (r0.z - r2.x) / s);
	} else {
		const double s = 2.0 * Math::sqrt(1.0 + r2.z - r0.x - r1.y);
		q = Quaternion((r0.z + r2.x) / s, (r1.z + r2.y) / s, 0.25 * s, (r1.x - r0.y) / s);
	}
	return q.normalized();
}

// DAZ's channels of a bone in the pose as it is now: angles (degrees), translation (cm), x y z.
// From t = t_p + R_p (c2 - c2_p + S_p v): v = S_p^-1 (R_p^T (t - t_p) - (c2 - c2_p)).
void DazCorrectives::_channels_of(Skeleton3D *p_skeleton, int p_bone) {
	if (posed_frames[p_bone] == frame) {
		return;
	}
	const Bone &b = bones[p_bone];
	const Transform3D skinning = _skinning(p_skeleton, b.bone);
	const Transform3D parent = b.parent >= 0 ? _skinning(p_skeleton, b.parent) : Transform3D();
	const Basis parent_inv = parent.basis.inverse();
	const Vector3 a = (b.o_inv * (parent_inv * skinning.basis) * b.o).get_euler(b.order);
	const Vector3 t = skinning.xform(b.c2);
	const Vector3 v = b.parent_scale_inv.xform(parent_inv.xform(t - parent.xform(b.c2_parent)) - (b.c2 - b.c2_parent));
	posed_angles[p_bone] = _nearer_reading(Vector3(Math::rad_to_deg(double(a.x)), Math::rad_to_deg(double(a.y)), Math::rad_to_deg(double(a.z))), b);
	posed_moves[p_bone] = v * 100.0;
	posed_frames[p_bone] = frame;
}

// The reading of a turn nearer the bone's DAZ limits: every turn is both (a, b, c) and
// (a + 180, 180 - b, c + 180) in its order's sequence, and get_euler keeps the middle angle within
// +-90 where DAZ's limits go past it (lShldr y to -110). On a tie, get_euler's.
Vector3 DazCorrectives::_nearer_reading(const Vector3 &p_angles, const Bone &p_bone) {
	Vector3 other = p_angles + Vector3(180.0, 180.0, 180.0);
	other[p_bone.middle] = 360.0 - double(other[p_bone.middle]);
	for (int i = 0; i < 3; i++) {
		other[i] = Math::fposmod(double(other[i]) + 180.0, 360.0) - 180.0;
	}
	return _outside(other, p_bone) < _outside(p_angles, p_bone) ? other : p_angles;
}

// How far angles lie outside the limits, summed over the axes (degrees).
double DazCorrectives::_outside(const Vector3 &p_angles, const Bone &p_bone) {
	double out = 0.0;
	for (int i = 0; i < 3; i++) {
		if (p_bone.limited[i]) {
			out += MAX(0.0, MAX(double(p_bone.limits[i].x) - double(p_angles[i]), double(p_angles[i]) - double(p_bone.limits[i].y)));
		}
	}
	return out;
}

double DazCorrectives::_value(Skeleton3D *p_skeleton, int p_channel) {
	if (value_frames[p_channel] == frame) {
		return values[p_channel];
	}
	const Channel &c = channels[p_channel];
	double value = c.own;
	double posed = 0.0; // the bone's value in the pose before this driver sets it
	if (c.bone >= 0) {
		_channels_of(p_skeleton, c.bone);
		posed = (c.kind == 0 ? posed_angles : posed_moves)[c.bone][c.axis];
	}
	if (c.input) {
		value = c.offset;
		MeshInstance3D *b = ObjectDB::get_instance<MeshInstance3D>(body);
		for (const Term &term : c.terms) {
			value += (b != nullptr ? b->get_blend_shape_value(term.shape) : 0.0) * term.scale;
		}
	} else if (Math::is_nan(value)) {
		value = posed;
	}
	if (!Math::is_nan(c.rest)) {
		value += posed - c.rest; // the animation's change since the saved pose
	}
	for (const Program &program : c.sum) {
		value += _run(p_skeleton, program);
	}
	for (const Program &program : c.mult) {
		value *= _run(p_skeleton, program);
	}
	if (c.clamped) {
		value = CLAMP(value, c.clamp_min, c.clamp_max);
	}
	values[p_channel] = value;
	value_frames[p_channel] = frame;
	return value;
}

double DazCorrectives::_run(Skeleton3D *p_skeleton, const Program &p_program) {
	double stack[64]; // _compile() keeps programs within it
	int top = 0;
	for (const Op &op : p_program) {
		switch (op.kind) {
			case Op::PUSH_CHANNEL:
				stack[top++] = _value(p_skeleton, op.index);
				break;
			case Op::PUSH_NUMBER:
				stack[top++] = op.number;
				break;
			case Op::MULT:
				top--;
				stack[top - 1] *= stack[top];
				break;
			case Op::SPLINE:
				stack[top - 1] = _spline(stack[top - 1], splines[op.index]);
				break;
		}
	}
	return stack[0];
}

// DAZ's spline_tcb (tension, continuity and bias all 0 in Genesis 8): Kochanek-Bartels, 0 slope at
// the end knots, an inner knot's slope the mean of its segments' rises scaled by 2 h / (h_in + h_out),
// flat outside (_daz_pipeline/formulas.py spline()).
double DazCorrectives::_spline(double p_x, const Spline &p_spline) {
	const LocalVector<double> &xs = p_spline.xs;
	const LocalVector<double> &ys = p_spline.ys;
	const uint32_t n = xs.size();
	if (p_x <= xs[0]) {
		return ys[0];
	}
	if (p_x >= xs[n - 1]) {
		return ys[n - 1];
	}
	uint32_t i = 0;
	while (xs[i + 1] < p_x) {
		i++;
	}
	const double h = xs[i + 1] - xs[i];
	const double u = (p_x - xs[i]) / h;
	const double t0 = i == 0 ? 0.0 : 0.5 * (ys[i + 1] - ys[i - 1]) * 2.0 * h / (xs[i] - xs[i - 1] + h);
	const double t1 = i + 1 == n - 1 ? 0.0 : 0.5 * (ys[i + 2] - ys[i]) * 2.0 * h / (h + xs[i + 2] - xs[i + 1]);
	const double u2 = u * u;
	const double u3 = u2 * u;
	return (2 * u3 - 3 * u2 + 1) * ys[i] + (u3 - 2 * u2 + u) * t0 + (-2 * u3 + 3 * u2) * ys[i + 1] + (u3 - u2) * t1;
}

// Turns and moves a bone so its DAZ channels on the given axes become their values; its other axes
// keep the pose's. A turn keeps the pivot (t, the centre's posed position) where the channels put it.
void DazCorrectives::_set_bone(Skeleton3D *p_skeleton, const Output &p_output) {
	const int i = p_output.bone;
	_channels_of(p_skeleton, i);
	const Vector3 now_angles = posed_angles[i];
	const Vector3 now_moves = posed_moves[i];
	Vector3 angles = now_angles;
	Vector3 moves = now_moves;
	for (int axis = 0; axis < 3; axis++) {
		if (p_output.rotations[axis] >= 0) {
			angles[axis] = _value(p_skeleton, p_output.rotations[axis]);
		}
		if (p_output.translations[axis] >= 0) {
			moves[axis] = _value(p_skeleton, p_output.translations[axis]);
		}
	}
	if (angles.is_equal_approx(now_angles) && moves.is_equal_approx(now_moves)) {
		return;
	}
	const Bone &b = bones[i];
	const Transform3D parent = b.parent >= 0 ? _skinning(p_skeleton, b.parent) : Transform3D();
	const Vector3 radians(Math::deg_to_rad(double(angles.x)), Math::deg_to_rad(double(angles.y)), Math::deg_to_rad(double(angles.z)));
	const Basis turn = b.o * Basis::from_euler(radians, b.order) * b.o_inv;
	const Basis r = parent.basis * turn;
	const Vector3 t = parent.xform(b.c2_parent) + parent.basis.xform(b.c2 - b.c2_parent + b.parent_scale.xform(moves / 100.0));
	p_skeleton->set_bone_global_pose(b.bone, Transform3D(r, t - r.xform(b.c2)) * bind_inverses[b.bone]);
	posed_frames[i] = 0;
}

// DAZ turns a bone about its pivot as the keys have moved it: in its parent's frame,
// x -> L (x - p - d) + p + d, which is today's turn plus (I - L) d. Zero while the bone isn't turned.
void DazCorrectives::_apply_centres(Skeleton3D *p_skeleton) {
	LocalVector<Pair<int, Vector3>> shift;
	for (const Centre &c : centres) {
		const double w = _value(p_skeleton, c.channel);
		if (w == 0.0) {
			continue;
		}
		bool found = false;
		for (Pair<int, Vector3> &s : shift) {
			if (s.first == c.bone) {
				s.second += c.shift * w;
				found = true;
			}
		}
		if (!found) {
			shift.push_back(Pair<int, Vector3>(c.bone, c.shift * w));
		}
	}
	for (const Pair<int, Vector3> &s : shift) {
		Transform3D parent_skin;
		int parent = p_skeleton->get_bone_parent(s.first);
		while (parent >= 0 && !has_bind[parent]) {
			parent = p_skeleton->get_bone_parent(parent);
		}
		if (parent >= 0) {
			parent_skin = _skinning(p_skeleton, parent);
		}
		Transform3D local = parent_skin.affine_inverse() * _skinning(p_skeleton, s.first);
		local.origin += s.second - local.basis.xform(s.second);
		p_skeleton->set_bone_global_pose(s.first, parent_skin * local * bind_inverses[s.first]);
	}
}

// This frame's channel values (the ones evaluated), by name.
Dictionary DazCorrectives::get_values() const {
	Dictionary out;
	for (uint32_t c = 0; c < channels.size(); c++) {
		if (value_frames[c] == frame) {
			out[channels[c].name] = values[c];
		}
	}
	return out;
}

// The skeleton bones the formulas set.
PackedInt32Array DazCorrectives::get_driven_bones() const {
	PackedInt32Array out;
	for (const Output &o : outputs) {
		out.push_back(bones[o.bone].bone);
	}
	return out;
}

// The shape keys the driver sets: [mesh, blend shape] each.
Array DazCorrectives::get_driven_keys() const {
	Array out;
	for (const Key &k : keys) {
		out.push_back(Array{ ObjectDB::get_instance(k.mesh), k.shape });
	}
	return out;
}

void DazCorrectives::_bind_methods() {
	ClassDB::bind_method(D_METHOD("setup", "avatar", "skeleton", "spec"), &DazCorrectives::setup);
	ClassDB::bind_method(D_METHOD("get_values"), &DazCorrectives::get_values);
	ClassDB::bind_method(D_METHOD("get_driven_bones"), &DazCorrectives::get_driven_bones);
	ClassDB::bind_method(D_METHOD("get_driven_keys"), &DazCorrectives::get_driven_keys);
	ClassDB::bind_method(D_METHOD("set_followers_after", "modifier"), &DazCorrectives::set_followers_after);
	ClassDB::bind_method(D_METHOD("update_followers"), &DazCorrectives::update_followers);
	ClassDB::bind_static_method("DazCorrectives", D_METHOD("find_body", "avatar", "keys"), &DazCorrectives::find_body);
}
