/**************************************************************************/
/*  taa.h                                                                 */
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

#include "servers/rendering/renderer_rd/shaders/effects/taa_resolve.glsl.gen.h"
#include "servers/rendering/renderer_rd/storage_rd/render_scene_buffers_rd.h"
#include "servers/rendering/renderer_rd/storage_rd/render_scene_data_rd.h"

namespace RendererRD {

class TAA {
public:
	TAA();
	~TAA();

	// p_read_msaa_color: read the multisampled colour instead of the internal texture, which the caller hasn't resolved
	// into (only with a history, see has_history()), and write the result straight into the internal texture.
	void process(Ref<RenderSceneBuffersRD> p_render_buffers, const RenderSceneDataRD *p_scene_data, RD::DataFormat p_format, float p_z_near, float p_z_far, bool p_read_msaa_color = false);
	bool has_history(Ref<RenderSceneBuffersRD> p_render_buffers) const;

private:
	struct TAAResolvePushConstant {
		float reprojection_matrix[16];
		float resolution_width;
		float resolution_height;
		float disocclusion_threshold;
		float variance_dynamic;
		float flicker_reduction;
		float pad[3];
	};

	TaaResolveShaderRD taa_shader;
	RID shader_version;
	RID pipelines[2]; // Internal texture input, multisampled colour input.

	// Which of the two history textures holds last frame's result; the resolve writes the other one.
	class TAABuffers : public RenderBufferCustomDataRD {
		GDCLASS(TAABuffers, RenderBufferCustomDataRD);

	public:
		uint32_t last = 0;

		virtual void configure(RenderSceneBuffersRD *p_render_buffers) override {}
		virtual void free_data() override {}
	};

	void resolve(RID p_frame, RID p_dest, RID p_internal_dest, RID p_depth, RID p_velocity, RID p_prev_velocity, RID p_history, const Projection &p_reprojection, Size2 p_resolution, float p_z_near, float p_z_far);
};

} // namespace RendererRD
