#[compute]

#version 450

#VERSION_DEFINES

layout(local_size_x = 64, local_size_y = 1, local_size_z = 1) in;

layout(set = 0, binding = 1, std430) buffer restrict writeonly DstVertexData {
	uint data[];
}
dst_vertices;

layout(set = 0, binding = 2, std430) buffer restrict readonly BlendShapeWeights {
	float data[];
}
blend_shape_weights;

layout(set = 1, binding = 0, std430) buffer restrict readonly SrcVertexData {
	uint data[];
}
src_vertices;

layout(set = 1, binding = 1, std430) buffer restrict readonly BoneWeightData {
	uint data[];
}
src_bone_weights;

layout(set = 1, binding = 2, std430) buffer restrict readonly BlendShapeData {
	uint data[];
}
src_blend_shapes;

layout(set = 2, binding = 0, std430) buffer restrict readonly SkeletonData {
	vec4 data[];
}
bone_transforms;

layout(push_constant, std430) uniform Params {
	bool has_normal;
	bool has_tangent;
	bool has_skeleton;
	bool has_blend_shape;

	uint vertex_count;
	uint vertex_stride;
	uint skin_stride;
	uint skin_weight_offset;

	uint blend_shape_count;
	bool normalized_blend_shapes;
	uint normal_tangent_stride;
	bool dual_quaternion;

	vec2 skeleton_transform_x;
	vec2 skeleton_transform_y;

	vec2 skeleton_transform_offset;
	vec2 inverse_transform_x;

	vec2 inverse_transform_y;
	vec2 inverse_transform_offset;
}
params;

vec2 uint_to_vec2(uint base) {
	uvec2 decode = (uvec2(base) >> uvec2(0, 16)) & uvec2(0xFFFF, 0xFFFF);
	return vec2(decode) / vec2(65535.0, 65535.0) * 2.0 - 1.0;
}

vec3 oct_to_vec3(vec2 oct) {
	vec3 v = vec3(oct.xy, 1.0 - abs(oct.x) - abs(oct.y));
	float t = max(-v.z, 0.0);
	v.xy += t * -sign(v.xy);
	return normalize(v);
}

vec3 decode_uint_oct_to_norm(uint base) {
	return oct_to_vec3(uint_to_vec2(base));
}

vec4 decode_uint_oct_to_tang(uint base) {
	vec2 oct_sign_encoded = uint_to_vec2(base);
	// Binormal sign encoded in y component
	vec2 oct = vec2(oct_sign_encoded.x, abs(oct_sign_encoded.y) * 2.0 - 1.0);
	return vec4(oct_to_vec3(oct), sign(oct_sign_encoded.y));
}

vec2 signNotZero(vec2 v) {
	return mix(vec2(-1.0), vec2(1.0), greaterThanEqual(v.xy, vec2(0.0)));
}

uint vec2_to_uint(vec2 base) {
	uvec2 enc = uvec2(clamp(ivec2(base * vec2(65535, 65535)), ivec2(0), ivec2(0xFFFF, 0xFFFF))) << uvec2(0, 16);
	return enc.x | enc.y;
}

vec2 vec3_to_oct(vec3 e) {
	e /= abs(e.x) + abs(e.y) + abs(e.z);
	vec2 oct = e.z >= 0.0f ? e.xy : (vec2(1.0f) - abs(e.yx)) * signNotZero(e.xy);
	return oct * 0.5f + 0.5f;
}

uint encode_norm_to_uint_oct(vec3 base) {
	return vec2_to_uint(vec3_to_oct(base));
}

uint encode_tang_to_uint_oct(vec4 base) {
	vec2 oct = vec3_to_oct(base.xyz);
	// Encode binormal sign in y component
	oct.y = oct.y * 0.5f + 0.5f;
	oct.y = base.w >= 0.0f ? oct.y : 1 - oct.y;

	if (oct.x == 0.0 && oct.y == 1.0) {
		// (1, 1) and (0, 1) decode to the same value, but (0, 1) messes with our compression detection.
		// So we sanitize here.
		oct.x = 1.0;
	}

	return vec2_to_uint(oct);
}

#ifndef MODE_2D
// A bone's transform: 3 rows of [rotation | translation].
mat4 bone_matrix(uint bone) {
	return mat4(bone_transforms.data[bone], bone_transforms.data[bone + 1], bone_transforms.data[bone + 2], vec4(0.0, 0.0, 0.0, 1.0));
}

// A rotation matrix's quaternion (x, y, z, w), from its rows.
vec4 quat_from_rows(vec3 r0, vec3 r1, vec3 r2) {
	float trace = r0.x + r1.y + r2.z;
	vec4 q;
	if (trace > 0.0) {
		float s = 0.5 / sqrt(trace + 1.0);
		q = vec4((r2.y - r1.z) * s, (r0.z - r2.x) * s, (r1.x - r0.y) * s, 0.25 / s);
	} else if (r0.x > r1.y && r0.x > r2.z) {
		float s = 2.0 * sqrt(1.0 + r0.x - r1.y - r2.z);
		q = vec4(0.25 * s, (r0.y + r1.x) / s, (r0.z + r2.x) / s, (r2.y - r1.z) / s);
	} else if (r1.y > r2.z) {
		float s = 2.0 * sqrt(1.0 + r1.y - r0.x - r2.z);
		q = vec4((r0.y + r1.x) / s, 0.25 * s, (r1.z + r2.y) / s, (r0.z - r2.x) / s);
	} else {
		float s = 2.0 * sqrt(1.0 + r2.z - r0.x - r1.y);
		q = vec4((r0.z + r2.x) / s, (r1.z + r2.y) / s, 0.25 * s, (r1.x - r0.y) / s);
	}
	return normalize(q);
}

vec4 quat_mul(vec4 a, vec4 b) {
	return vec4(a.w * b.xyz + b.w * a.xyz + cross(a.xyz, b.xyz), a.w * b.w - dot(a.xyz, b.xyz));
}
#endif

void main() {
	uint index = gl_GlobalInvocationID.x;
	if (index >= params.vertex_count) {
		return;
	}

	uint src_offset = index * params.vertex_stride;

#ifdef MODE_2D
	vec2 vertex = uintBitsToFloat(uvec2(src_vertices.data[src_offset + 0], src_vertices.data[src_offset + 1]));

	if (params.has_blend_shape) {
		float blend_total = 0.0;
		vec2 blend_vertex = vec2(0.0);

		for (uint i = 0; i < params.blend_shape_count; i++) {
			float w = blend_shape_weights.data[i];
			if (abs(w) > 0.0001) {
				uint base_offset = (params.vertex_count * i + index) * params.vertex_stride;

				blend_vertex += uintBitsToFloat(uvec2(src_blend_shapes.data[base_offset + 0], src_blend_shapes.data[base_offset + 1])) * w;

				base_offset += 2;

				blend_total += w;
			}
		}

		if (params.normalized_blend_shapes) {
			vertex = (1.0 - blend_total) * vertex;
		}

		vertex += blend_vertex;
	}

	if (params.has_skeleton) {
		uint skin_offset = params.skin_stride * index;

		uvec2 bones = uvec2(src_bone_weights.data[skin_offset + 0], src_bone_weights.data[skin_offset + 1]);
		uvec2 bones_01 = uvec2(bones.x & 0xFFFF, bones.x >> 16) * 2; //pre-add xform offset
		uvec2 bones_23 = uvec2(bones.y & 0xFFFF, bones.y >> 16) * 2;

		skin_offset += params.skin_weight_offset;

		uvec2 weights = uvec2(src_bone_weights.data[skin_offset + 0], src_bone_weights.data[skin_offset + 1]);

		vec2 weights_01 = unpackUnorm2x16(weights.x);
		vec2 weights_23 = unpackUnorm2x16(weights.y);

		mat4 m = mat4(bone_transforms.data[bones_01.x], bone_transforms.data[bones_01.x + 1], vec4(0.0, 0.0, 1.0, 0.0), vec4(0.0, 0.0, 0.0, 1.0)) * weights_01.x;
		m += mat4(bone_transforms.data[bones_01.y], bone_transforms.data[bones_01.y + 1], vec4(0.0, 0.0, 1.0, 0.0), vec4(0.0, 0.0, 0.0, 1.0)) * weights_01.y;
		m += mat4(bone_transforms.data[bones_23.x], bone_transforms.data[bones_23.x + 1], vec4(0.0, 0.0, 1.0, 0.0), vec4(0.0, 0.0, 0.0, 1.0)) * weights_23.x;
		m += mat4(bone_transforms.data[bones_23.y], bone_transforms.data[bones_23.y + 1], vec4(0.0, 0.0, 1.0, 0.0), vec4(0.0, 0.0, 0.0, 1.0)) * weights_23.y;

		mat4 skeleton_matrix = mat4(vec4(params.skeleton_transform_x, 0.0, 0.0), vec4(params.skeleton_transform_y, 0.0, 0.0), vec4(0.0, 0.0, 1.0, 0.0), vec4(params.skeleton_transform_offset, 0.0, 1.0));
		mat4 inverse_matrix = mat4(vec4(params.inverse_transform_x, 0.0, 0.0), vec4(params.inverse_transform_y, 0.0, 0.0), vec4(0.0, 0.0, 1.0, 0.0), vec4(params.inverse_transform_offset, 0.0, 1.0));

		m = skeleton_matrix * transpose(m) * inverse_matrix;

		vertex = (m * vec4(vertex, 0.0, 1.0)).xy;
	}

	uint dst_offset = index * params.vertex_stride;

	uvec2 uvertex = floatBitsToUint(vertex);
	dst_vertices.data[dst_offset + 0] = uvertex.x;
	dst_vertices.data[dst_offset + 1] = uvertex.y;

#else
	vec3 vertex;
	vec3 normal;
	vec4 tangent;

	vertex = uintBitsToFloat(uvec3(src_vertices.data[src_offset + 0], src_vertices.data[src_offset + 1], src_vertices.data[src_offset + 2]));

	uint src_normal = params.vertex_count * params.vertex_stride + index * params.normal_tangent_stride;

	if (params.has_normal) {
		normal = decode_uint_oct_to_norm(src_vertices.data[src_normal]);
		src_normal++;
	}

	if (params.has_tangent) {
		tangent = decode_uint_oct_to_tang(src_vertices.data[src_normal]);
	}

	if (params.has_blend_shape) {
		float blend_total = 0.0;
		vec3 blend_vertex = vec3(0.0);
		vec3 blend_normal = vec3(0.0);
		vec3 blend_tangent = vec3(0.0);

		for (uint i = 0; i < params.blend_shape_count; i++) {
			float w = blend_shape_weights.data[i];
			if (abs(w) > 0.0001) {
				uint base_offset = params.vertex_count * i * (params.vertex_stride + params.normal_tangent_stride) + index * params.vertex_stride;

				// Normalized shapes are stored whole: (1 - sum w) base + sum w shape. Summed as
				// base + sum w (shape - base), the same thing without float32 cancelling numbers the
				// size of sum w times the mesh's extent (60 keys on a body: 0.02 mm at head height).
				vec3 shape_vertex = uintBitsToFloat(uvec3(src_blend_shapes.data[base_offset + 0], src_blend_shapes.data[base_offset + 1], src_blend_shapes.data[base_offset + 2]));
				blend_vertex += (params.normalized_blend_shapes ? shape_vertex - vertex : shape_vertex) * w;

				uint base_normal = params.vertex_count * i * (params.vertex_stride + params.normal_tangent_stride) + params.vertex_count * params.vertex_stride + index * params.normal_tangent_stride;

				if (params.has_normal) {
					vec3 shape_normal = decode_uint_oct_to_norm(src_blend_shapes.data[base_normal]);
					blend_normal += (params.normalized_blend_shapes ? shape_normal - normal : shape_normal) * w;
					base_normal++;
				}

				if (params.has_tangent) {
					vec3 shape_tangent = decode_uint_oct_to_tang(src_blend_shapes.data[base_normal]).rgb;
					blend_tangent += (params.normalized_blend_shapes ? shape_tangent - tangent.rgb : shape_tangent) * w;
				}

				blend_total += w;
			}
		}

		vertex += blend_vertex;
		normal = normalize(normal + blend_normal);
		tangent.rgb = normalize(tangent.rgb + blend_tangent);
	}

	if (params.has_skeleton) {
		// The vertex's bones (offsets into bone_transforms) and weights: 4, or 8.
		uint bone_ids[8];
		float bone_weights[8];
		uint count = params.skin_weight_offset == 4 ? 8 : 4;
		for (uint set = 0; set < count / 4; set++) {
			uint skin_offset = params.skin_stride * index + set * 2;
			uvec2 bones = uvec2(src_bone_weights.data[skin_offset + 0], src_bone_weights.data[skin_offset + 1]);
			uvec2 weights = uvec2(src_bone_weights.data[skin_offset + params.skin_weight_offset + 0], src_bone_weights.data[skin_offset + params.skin_weight_offset + 1]);
			bone_ids[set * 4 + 0] = (bones.x & 0xFFFF) * 3; //pre-add xform offset
			bone_ids[set * 4 + 1] = (bones.x >> 16) * 3;
			bone_ids[set * 4 + 2] = (bones.y & 0xFFFF) * 3;
			bone_ids[set * 4 + 3] = (bones.y >> 16) * 3;
			vec2 weights_01 = unpackUnorm2x16(weights.x);
			vec2 weights_23 = unpackUnorm2x16(weights.y);
			bone_weights[set * 4 + 0] = weights_01.x;
			bone_weights[set * 4 + 1] = weights_01.y;
			bone_weights[set * 4 + 2] = weights_23.x;
			bone_weights[set * 4 + 3] = weights_23.y;
		}

		// Dual quaternions (Kavan et al.), as DAZ skins Genesis figures: each bone's rigid move
		// blended as a dual quaternion, in the same hemisphere as the vertex's heaviest bone.
		// Each bone's move is where it alone puts the vertex, less its turn: a rigid bone's translation,
		// and for a scaled one (BreastInertia's squash) its scale lands there too, so a vertex is
		// exact on one bone and the blend stays continuous as a scale comes and goes.
		if (params.dual_quaternion) {
			vec4 real = vec4(0.0);
			vec4 dual = vec4(0.0);
			uint lead = 0;
			for (uint i = 1; i < count; i++) {
				if (bone_weights[i] > bone_weights[lead]) {
					lead = i;
				}
			}
			vec4 lead_real = vec4(0.0);
			for (uint k = 0; k < count; k++) {
				uint i = k == 0 ? lead : (k == lead ? 0 : k); // the heaviest first: it sets the hemisphere
				if (bone_weights[i] == 0.0) {
					continue;
				}
				vec4 r0 = bone_transforms.data[bone_ids[i]];
				vec4 r1 = bone_transforms.data[bone_ids[i] + 1];
				vec4 r2 = bone_transforms.data[bone_ids[i] + 2];
				vec4 q = quat_from_rows(r0.xyz, r1.xyz, r2.xyz);
				if (k == 0) {
					lead_real = q;
				}
				float w = dot(q, lead_real) < 0.0 ? -bone_weights[i] : bone_weights[i];
				vec3 moved = vec3(dot(r0.xyz, vertex), dot(r1.xyz, vertex), dot(r2.xyz, vertex)) + vec3(r0.w, r1.w, r2.w);
				vec3 turned = vertex + 2.0 * cross(q.xyz, cross(q.xyz, vertex) + q.w * vertex);
				real += q * w;
				dual += quat_mul(vec4(moved - turned, 0.0), q) * (0.5 * w);
			}
			float norm = length(real);
			real /= norm;
			dual /= norm;
			vec3 translation = 2.0 * (real.w * dual.xyz - dual.w * real.xyz + cross(real.xyz, dual.xyz));
			vertex += 2.0 * cross(real.xyz, cross(real.xyz, vertex) + real.w * vertex);
			vertex += translation;
			normal = normalize(normal + 2.0 * cross(real.xyz, cross(real.xyz, normal) + real.w * normal));
			tangent.xyz = normalize(tangent.xyz + 2.0 * cross(real.xyz, cross(real.xyz, tangent.xyz) + real.w * tangent.xyz));
		} else {
			mat4 m = mat4(0.0);
			for (uint i = 0; i < count; i++) {
				m += bone_matrix(bone_ids[i]) * bone_weights[i];
			}

			//reverse order because its transposed
			vertex = (vec4(vertex, 1.0) * m).xyz;
			normal = normalize((vec4(normal, 0.0) * m).xyz);
			tangent.xyz = normalize((vec4(tangent.xyz, 0.0) * m).xyz);
		}
	}

	uint dst_offset = index * params.vertex_stride;

	uvec3 uvertex = floatBitsToUint(vertex);
	dst_vertices.data[dst_offset + 0] = uvertex.x;
	dst_vertices.data[dst_offset + 1] = uvertex.y;
	dst_vertices.data[dst_offset + 2] = uvertex.z;

	uint dst_normal = params.vertex_count * params.vertex_stride + index * params.normal_tangent_stride;

	if (params.has_normal) {
		dst_vertices.data[dst_normal] = encode_norm_to_uint_oct(normal);
		dst_normal++;
	}

	if (params.has_tangent) {
		dst_vertices.data[dst_normal] = encode_tang_to_uint_oct(tangent);
	}

#endif
}
