/**************************************************************************/
/*  world_kernels.h                                                       */
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

#include "core/object/object.h"
#include "core/variant/array.h"
#include "core/variant/dictionary.h"

// The pony game's world generation loops that are too slow in GDScript (world_gen.gd).
class WorldKernels : public Object {
	GDCLASS(WorldKernels, Object);

protected:
	static void _bind_methods();

public:
	static Array priority_flood(const PackedFloat32Array &p_heights, int p_px, double p_sea_floor, const PackedInt32Array &p_gates, const PackedByteArray &p_rise, double p_eps);
	static Array island_heights(const Dictionary &p_params);
	static Dictionary biome_fields(const Dictionary &p_params);
	static Dictionary hydrology(const Dictionary &p_params);
	static PackedInt32Array ring_distance(const PackedFloat32Array &p_levels, int p_size, int p_rings);
	static Dictionary water_map(const Dictionary &p_params);
	static PackedFloat32Array resample_grid(const PackedFloat32Array &p_src, int p_src_px, int p_dst_px, double p_step);
	static PackedFloat32Array erode(const Dictionary &p_params);
	static PackedFloat32Array slope(const PackedFloat32Array &p_heights, int p_size, int p_o, double p_inv);
	static PackedFloat32Array floor_up(const PackedFloat32Array &p_src, int p_src_px, int p_dst_px, double p_k);
	static Dictionary control_map(const Dictionary &p_params);
	static Dictionary road_grid(const Dictionary &p_params);
	static PackedInt32Array road_astar(const Dictionary &p_params);
	static PackedInt32Array road_rasterize(const Dictionary &p_params);
	static Dictionary barn_find(const Dictionary &p_params);
	static Dictionary foliage(const Dictionary &p_params);
	static void chunk_insert(Dictionary p_store, Dictionary p_near_grid, int p_id, const Array &p_xforms, const PackedColorArray &p_colors, double p_chunk_m, double p_near_cell_m);
};
