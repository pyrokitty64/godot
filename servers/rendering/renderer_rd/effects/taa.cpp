/**************************************************************************/
/*  taa.cpp                                                               */
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

#include "taa.h"

#include "core/config/project_settings.h"

#include "servers/rendering/renderer_rd/effects/copy_effects.h"
#include "servers/rendering/renderer_rd/storage_rd/material_storage.h"
#include "servers/rendering/renderer_rd/uniform_set_cache_rd.h"

using namespace RendererRD;

TAA::TAA() {
	Vector<String> taa_modes;
	taa_modes.push_back("\n#define MODE_TAA_RESOLVE");
	taa_modes.push_back("\n#define MODE_TAA_RESOLVE\n#define MSAA_INPUT\n");
	taa_shader.initialize(taa_modes);
	shader_version = taa_shader.version_create();
	for (int i = 0; i < 2; i++) {
		pipelines[i] = RD::get_singleton()->compute_pipeline_create(taa_shader.version_get_shader(shader_version, i));
	}
}

TAA::~TAA() {
	taa_shader.version_free(shader_version);
}

bool TAA::has_history(Ref<RenderSceneBuffersRD> p_render_buffers) const {
	return p_render_buffers->has_texture(SNAME("taa"), SNAME("history0"));
}

void TAA::resolve(RID p_frame, RID p_dest, RID p_internal_dest, RID p_depth, RID p_velocity, RID p_prev_velocity, RID p_history, const Projection &p_reprojection, Size2 p_resolution, float p_z_near, float p_z_far) {
	UniformSetCacheRD *uniform_set_cache = UniformSetCacheRD::get_singleton();
	ERR_FAIL_NULL(uniform_set_cache);
	MaterialStorage *material_storage = MaterialStorage::get_singleton();
	ERR_FAIL_NULL(material_storage);

	const int mode = p_internal_dest.is_valid() ? 1 : 0;
	RID shader = taa_shader.version_get_shader(shader_version, mode);
	ERR_FAIL_COND(shader.is_null());

	RID default_sampler = material_storage->sampler_rd_get_default(RSE::CANVAS_ITEM_TEXTURE_FILTER_LINEAR, RSE::CANVAS_ITEM_TEXTURE_REPEAT_DISABLED);

	float base_variance = 1.1f;
	float base_variance_min = 0.75f;
	float base_variance_max = 1.00f;
	float variance_scale = 1080.0f / p_resolution.height; // 1080p taken as baseline for calculation, as this is most commonly used resolution

	TAAResolvePushConstant push_constant;
	memset(&push_constant, 0, sizeof(TAAResolvePushConstant));
	RendererRD::MaterialStorage::store_camera(p_reprojection, push_constant.reprojection_matrix);
	push_constant.resolution_width = p_resolution.width;
	push_constant.resolution_height = p_resolution.height;
	push_constant.disocclusion_threshold = 2.5f; // If velocity changes by less than this amount of texels we can retain the accumulation buffer.
	push_constant.variance_dynamic = CLAMP(base_variance * variance_scale, base_variance_min, base_variance_max); // Variance dynamically scales based on resolution
	// Project settings, read every frame: how wide the history clip box is, and how much the history is favored where
	// a pixel changed in brightness (1.0 each is the stock behavior).
	push_constant.variance_dynamic *= float(GLOBAL_GET("rendering/anti_aliasing/quality/taa_clip_scale"));
	push_constant.flicker_reduction = GLOBAL_GET("rendering/anti_aliasing/quality/taa_flicker_reduction");

	RD::ComputeListID compute_list = RD::get_singleton()->compute_list_begin();
	RD::get_singleton()->compute_list_bind_compute_pipeline(compute_list, pipelines[mode]);

	RD::Uniform u_frame_source = mode == 1 ? RD::Uniform(RD::UNIFORM_TYPE_SAMPLER_WITH_TEXTURE, 0, { default_sampler, p_frame }) : RD::Uniform(RD::UNIFORM_TYPE_IMAGE, 0, { p_frame });
	RD::Uniform u_depth(RD::UNIFORM_TYPE_SAMPLER_WITH_TEXTURE, 1, { default_sampler, p_depth });
	RD::Uniform u_velocity(RD::UNIFORM_TYPE_IMAGE, 2, { p_velocity });
	RD::Uniform u_prev_velocity(RD::UNIFORM_TYPE_IMAGE, 3, { p_prev_velocity });
	RD::Uniform u_history(RD::UNIFORM_TYPE_SAMPLER_WITH_TEXTURE, 4, { default_sampler, p_history });
	RD::Uniform u_frame_dest(RD::UNIFORM_TYPE_IMAGE, 5, { p_dest });

	if (mode == 1) {
		RD::Uniform u_internal_dest(RD::UNIFORM_TYPE_IMAGE, 6, { p_internal_dest });
		RD::get_singleton()->compute_list_bind_uniform_set(compute_list, uniform_set_cache->get_cache(shader, 0, u_frame_source, u_depth, u_velocity, u_prev_velocity, u_history, u_frame_dest, u_internal_dest), 0);
	} else {
		RD::get_singleton()->compute_list_bind_uniform_set(compute_list, uniform_set_cache->get_cache(shader, 0, u_frame_source, u_depth, u_velocity, u_prev_velocity, u_history, u_frame_dest), 0);
	}
	RD::get_singleton()->compute_list_set_push_constant(compute_list, &push_constant, sizeof(TAAResolvePushConstant));
	RD::get_singleton()->compute_list_dispatch_threads(compute_list, p_resolution.width, p_resolution.height, 1);
	RD::get_singleton()->compute_list_end();
}

void TAA::process(Ref<RenderSceneBuffersRD> p_render_buffers, const RenderSceneDataRD *p_scene_data, RD::DataFormat p_format, float p_z_near, float p_z_far, bool p_read_msaa_color) {
	CopyEffects *copy_effects = CopyEffects::get_singleton();
	ERR_FAIL_NULL(p_scene_data);

	uint32_t view_count = p_render_buffers->get_view_count();
	Size2i internal_size = p_render_buffers->get_internal_size();
	Size2i target_size = p_render_buffers->get_target_size();

	// Two history textures, swapped each frame: the resolve reads last frame's and writes the other, so the result
	// needs only one copy (into the internal texture) instead of two.
	if (!p_render_buffers->has_custom_data(SNAME("taa"))) {
		p_render_buffers->set_custom_data(SNAME("taa"), memnew(TAABuffers));
	}
	Ref<TAABuffers> buffers = p_render_buffers->get_custom_data(SNAME("taa"));
	static const StringName history_names[2] = { SNAME("history0"), SNAME("history1") };

	bool just_allocated = false;
	if (!p_render_buffers->has_texture(SNAME("taa"), history_names[0])) {
		uint32_t usage_bits = RD::TEXTURE_USAGE_SAMPLING_BIT | RD::TEXTURE_USAGE_STORAGE_BIT;

		p_render_buffers->create_texture(SNAME("taa"), history_names[0], p_format, usage_bits);
		p_render_buffers->create_texture(SNAME("taa"), history_names[1], p_format, usage_bits);

		p_render_buffers->create_texture(SNAME("taa"), SNAME("prev_velocity"), RD::DATA_FORMAT_R16G16_SFLOAT, usage_bits);

		buffers->last = 0;
		just_allocated = true;
	}
	// Reading the multisampled colour needs a history: the caller checks has_history() and resolves as usual without one.
	ERR_FAIL_COND(p_read_msaa_color && just_allocated);
	const uint32_t next = just_allocated ? buffers->last : 1 - buffers->last;

	RD::get_singleton()->draw_command_begin_label("TAA");

	for (uint32_t v = 0; v < view_count; v++) {
		// Get our (cached) slices
		RID internal_texture = p_render_buffers->get_internal_texture(v);
		RID velocity_buffer = p_render_buffers->get_velocity_buffer(false, v);
		RID taa_history = p_render_buffers->get_texture_slice(SNAME("taa"), history_names[buffers->last], v, 0);
		RID taa_next = p_render_buffers->get_texture_slice(SNAME("taa"), history_names[next], v, 0);
		RID taa_prev_velocity = p_render_buffers->get_texture_slice(SNAME("taa"), SNAME("prev_velocity"), v, 0);

		if (!just_allocated) {
			RID depth_texture = p_render_buffers->get_depth_texture(v);

			// For pixels left with invalid motion vectors (the sky), the resolve derives the camera's motion from depth
			// with this matrix: this view's clip space to its clip space last frame.
			Projection correction;
			correction.set_depth_correction(true, true, false);
			Projection reprojection = (correction * p_scene_data->prev_view_projection[v]) * p_scene_data->prev_cam_transform.affine_inverse() * p_scene_data->cam_transform * (correction * p_scene_data->view_projection[v]).inverse();

			if (p_read_msaa_color) {
				// The resolve writes the internal texture itself: no copy.
				resolve(p_render_buffers->get_color_msaa(v), taa_next, internal_texture, depth_texture, velocity_buffer, taa_prev_velocity, taa_history, reprojection, Size2(internal_size.x, internal_size.y), p_z_near, p_z_far);
			} else {
				resolve(internal_texture, taa_next, RID(), depth_texture, velocity_buffer, taa_prev_velocity, taa_history, reprojection, Size2(internal_size.x, internal_size.y), p_z_near, p_z_far);
				copy_effects->copy_to_rect(taa_next, internal_texture, Rect2(0, 0, internal_size.x, internal_size.y));
			}
		} else {
			// No history yet: this frame's image is the history.
			copy_effects->copy_to_rect(internal_texture, taa_next, Rect2(0, 0, internal_size.x, internal_size.y));
		}

		copy_effects->copy_to_rect(velocity_buffer, taa_prev_velocity, Rect2(0, 0, target_size.x, target_size.y));
	}

	buffers->last = next;

	RD::get_singleton()->draw_command_end_label();
}
