/**************************************************************************/
/*  test_renderer_scene_render.cpp                                        */
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

#include "tests/test_macros.h"

TEST_FORCE_LINK(test_renderer_scene_render)

#include "servers/rendering/renderer_scene_render.h"

namespace TestRendererSceneRender {

// A stereo pair shaped like a headset: two perspective eyes, 64 mm apart.
static void make_stereo_views(LocalVector<Transform3D> &r_offsets, LocalVector<Projection> &r_projections) {
	r_offsets.resize(2);
	r_projections.resize(2);
	for (int eye = 0; eye < 2; eye++) {
		r_offsets[eye] = Transform3D(Basis(), Vector3(eye == 0 ? -0.032 : 0.032, 0, 0));
		r_projections[eye] = Projection::create_perspective_hmd(90, 0.9, 0.05, 100, false, eye + 1, 0.064, 10);
	}
}

TEST_CASE("[RendererSceneRender] CameraData keeps TAA jitter for a single view") {
	RendererSceneRender::CameraData camera_data;
	camera_data.set_camera(Transform3D(), Projection::create_perspective(70, 16.0 / 9.0, 0.05, 100), false, false, Vector2(0.25, -0.5), 3.0f);

	CHECK(camera_data.view_count == 1);
	CHECK(camera_data.taa_jitter.is_equal_approx(Vector2(0.25, -0.5)));
	CHECK(camera_data.taa_frame_count == doctest::Approx(3.0f));
}

TEST_CASE("[RendererSceneRender] CameraData keeps TAA jitter for multiview") {
	LocalVector<Transform3D> offsets;
	LocalVector<Projection> projections;
	make_stereo_views(offsets, projections);

	RendererSceneRender::CameraData camera_data;
	camera_data.set_multiview_camera(Transform3D(), offsets, projections, false, false, Vector2(0.25, -0.5), 3.0f);

	CHECK(camera_data.view_count == 2);
	// Without jitter here, TAA in XR accumulates the same sample every frame and never antialiases.
	CHECK(camera_data.taa_jitter.is_equal_approx(Vector2(0.25, -0.5)));
	CHECK(camera_data.taa_frame_count == doctest::Approx(3.0f));
}

TEST_CASE("[RendererSceneRender] CameraData multiview defaults to no jitter") {
	LocalVector<Transform3D> offsets;
	LocalVector<Projection> projections;
	make_stereo_views(offsets, projections);

	RendererSceneRender::CameraData camera_data;
	camera_data.taa_jitter = Vector2(1, 1);
	camera_data.taa_frame_count = 7.0f;
	camera_data.set_multiview_camera(Transform3D(), offsets, projections, false, false);

	CHECK(camera_data.taa_jitter == Vector2());
	CHECK(camera_data.taa_frame_count == doctest::Approx(0.0f));
}

} // namespace TestRendererSceneRender
