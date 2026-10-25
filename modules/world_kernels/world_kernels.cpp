/**************************************************************************/
/*  world_kernels.cpp                                                     */
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

#include "world_kernels.h"

#include "core/object/class_db.h"
#include "core/templates/local_vector.h"

namespace {

// world_gen's MinHeap (godot/world/scripts/min_heap.gd), step for step: the same order among equal
// keys, so the flood visits cells in the same order and the world comes out the same.
struct MinHeap {
	LocalVector<float> k;
	LocalVector<int32_t> v;
	uint32_t n = 0;

	void push(float p_key, int32_t p_val) {
		if (n == k.size()) {
			k.resize(MAX(n * 2, 4096u));
			v.resize(k.size());
		}
		uint32_t i = n++;
		while (i > 0) {
			uint32_t p = (i - 1) >> 1;
			if (k[p] <= p_key) {
				break;
			}
			k[i] = k[p];
			v[i] = v[p];
			i = p;
		}
		k[i] = p_key;
		v[i] = p_val;
	}

	int32_t pop() {
		int32_t top = v[0];
		n--;
		if (n > 0) {
			float key = k[n];
			int32_t val = v[n];
			uint32_t i = 0;
			while (true) {
				uint32_t l = 2 * i + 1;
				if (l >= n) {
					break;
				}
				uint32_t r = l + 1;
				uint32_t m = (r >= n || k[l] <= k[r]) ? l : r;
				if (k[m] >= key) {
					break;
				}
				k[i] = k[m];
				v[i] = v[m];
				i = m;
			}
			k[i] = key;
			v[i] = val;
		}
		return top;
	}
};

} // namespace

// Priority flood (Barnes et al. 2014) from the border and the coast inward, filling every pit to
// its spill level plus a small rise per step, so filled flats drain toward their outlet. Returns
// [filled, spill]: the filled surface, and for each cell the level of the water that would stand
// there (the ground where none would). Cells at or below p_sea_floor are sea; border cells in
// p_gates are not outlets. The rise per step is p_eps times the step's length times
// 0.2 + 1.6 * p_rise / 255. Arithmetic is in doubles stored as floats, as GDScript does it.
Array WorldKernels::priority_flood(const PackedFloat32Array &p_heights, int p_px, double p_sea_floor, const PackedInt32Array &p_gates, const PackedByteArray &p_rise, double p_eps) {
	const int32_t px = p_px;
	const int64_t n = int64_t(px) * px;
	ERR_FAIL_COND_V(px < 2 || p_heights.size() != n || p_rise.size() != n, Array());
	const float *h = p_heights.ptr();
	const uint8_t *rise = p_rise.ptr();
	PackedFloat32Array filled = p_heights;
	PackedFloat32Array spill = p_heights;
	float *f = filled.ptrw();
	float *s = spill.ptrw();
	LocalVector<uint8_t> closed;
	closed.resize_initialized(n);
	LocalVector<uint8_t> gate;
	gate.resize_initialized(n);
	for (int32_t g : p_gates) {
		if (g >= 0 && g < n) {
			gate[g] = 1;
		}
	}
	LocalVector<int32_t> fifo;
	fifo.resize(n);
	int64_t fh = 0;
	int64_t ft = 0;
	MinHeap heap;
	const int32_t last = px - 1;

	int32_t i = 0;
	for (int32_t y = 0; y < px; y++) {
		const bool border_row = y == 0 || y == last;
		for (int32_t x = 0; x < px; x++, i++) {
			if (h[i] <= p_sea_floor) {
				closed[i] = 1;
			} else if ((border_row || x == 0 || x == last) && !gate[i]) {
				closed[i] = 1;
				heap.push(h[i], i);
			}
		}
	}
	// Sea cells on the coast start the flood ahead of the heap.
	i = 0;
	for (int32_t y = 0; y < px; y++) {
		const int32_t y0 = MAX(y - 1, 0);
		const int32_t y1 = MIN(y + 1, last);
		for (int32_t x = 0; x < px; x++, i++) {
			if (!(h[i] <= p_sea_floor)) {
				continue;
			}
			const int32_t x0 = MAX(x - 1, 0);
			const int32_t x1 = MIN(x + 1, last);
			bool coast = false;
			for (int32_t yy = y0; yy <= y1 && !coast; yy++) {
				for (int32_t xx = x0; xx <= x1; xx++) {
					if (h[yy * px + xx] > p_sea_floor) {
						coast = true;
						break;
					}
				}
			}
			if (coast) {
				fifo[ft++] = i;
			}
		}
	}

	const int32_t offs[8] = { -px - 1, -px, -px + 1, -1, 1, px - 1, px, px + 1 };
	const int32_t dxs[8] = { -1, 0, 1, -1, 1, -1, 0, 1 };
	const int32_t dys[8] = { -1, -1, -1, 0, 0, 1, 1, 1 };
	const float step_len[8] = { 1.4142f, 1.0f, 1.4142f, 1.0f, 1.0f, 1.4142f, 1.0f, 1.4142f };
	while (fh < ft || heap.n > 0) {
		const int32_t c = fh < ft ? fifo[fh++] : heap.pop();
		const double fc = f[c];
		const int32_t cx = c % px;
		const int32_t cy = c / px;
		const bool edge = cx == 0 || cy == 0 || cx == last || cy == last;
		for (int d = 0; d < 8; d++) {
			if (edge) {
				const int32_t x = cx + dxs[d];
				const int32_t y = cy + dys[d];
				if (x < 0 || y < 0 || x > last || y > last) {
					continue;
				}
			}
			const int32_t j = c + offs[d];
			if (closed[j]) {
				continue;
			}
			closed[j] = 1;
			if (f[j] <= fc) {
				f[j] = float(fc + p_eps * double(step_len[d]) * (0.2 + 1.6 * double(rise[j]) / 255.0));
				s[j] = s[c];
			} else {
				s[j] = f[j];
			}
			heap.push(f[j], j);
		}
	}
	Array out;
	out.push_back(filled);
	out.push_back(spill);
	return out;
}

void WorldKernels::_bind_methods() {
	ClassDB::bind_static_method("WorldKernels", D_METHOD("priority_flood", "heights", "px", "sea_floor", "gates", "rise", "eps"), &WorldKernels::priority_flood);
}
