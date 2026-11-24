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

#include "core/io/image.h"
#include "core/math/geometry_2d.h"
#include "core/math/random_number_generator.h"
#include "core/object/class_db.h"
#include "core/object/worker_thread_pool.h"
#include "core/os/os.h"
#include "core/templates/hash_map.h"
#include "core/templates/local_vector.h"
#include "core/variant/typed_array.h"
#include "core/variant/variant_internal.h"
#include "core/variant/variant_utility.h"
#include "modules/noise/fastnoise_lite.h"

namespace {

// world_gen's MinHeap (godot/world/scripts/min_heap.gd until 2026-11-05), step for step: the same order among equal
// keys, so the flood visits cells in the same order and the world comes out the same.
struct MinHeap {
	LocalVector<float> k;
	LocalVector<int32_t> v;
	uint32_t n = 0;

	// GDScript compares the stored float keys with the key it is given, a double in RoadBuilder's A*.
	template <typename K>
	void push(K p_key, int32_t p_val) {
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

// The heights and biome kernels below are world_gen.gd's per-cell loops ported line for line.
// GDScript does its float math in doubles and its Vector2 math in floats, so these do the same,
// through the same functions GDScript calls (VariantUtilityFunctions, Vector2, Geometry2D,
// FastNoiseLite), in the same order: the results match the scripts bit for bit. Rows run on
// WorkerThreadPool; each row writes only its own cells, so the thread order does not matter.

namespace {

typedef VariantUtilityFunctions VU;

template <typename F>
void for_each_row(int p_rows, const F &p_fn) {
	auto call = [](void *p_fn_ptr, uint32_t p_row) { (*static_cast<const F *>(p_fn_ptr))(int(p_row)); };
	WorkerThreadPool *pool = WorkerThreadPool::get_singleton();
	WorkerThreadPool::GroupID g = pool->add_native_group_task(call, const_cast<F *>(&p_fn), p_rows, -1, true, "WorldKernels rows");
	pool->wait_for_group_task_completion(g);
}

struct Ellipse {
	Vector2 c;
	Vector2 r;
	double a = 0.0;
};

struct Hollow {
	Vector2 c;
	double r = 0.0;
	double depth = 0.0;
};

// island.gd's plan and its per-point fields (land, spine_dist, lee, rain).
struct IslandShape {
	LocalVector<Ellipse> lobes;
	LocalVector<Ellipse> bites;
	PackedVector2Array spine;
	LocalVector<PackedVector2Array> spurs;
	double spine_h = 0.0;
	double spine_w = 0.0;
	Vector2 wind;
	LocalVector<Hollow> hollows;
	Ref<FastNoiseLite> coast_n;
	double margin = 0.0;

	static void read_ellipses(const Array &p_from, LocalVector<Ellipse> &r_to) {
		for (const Variant &v : p_from) {
			const Array l = v;
			r_to.push_back({ l[0], l[1], l[2] });
		}
	}

	explicit IslandShape(Object *p_island) {
		read_ellipses(p_island->get("lobes"), lobes);
		read_ellipses(p_island->get("bites"), bites);
		spine = p_island->get("spine");
		for (const Variant &v : Array(p_island->get("spurs"))) {
			spurs.push_back(v);
		}
		spine_h = p_island->get("spine_h");
		spine_w = p_island->get("spine_w");
		wind = p_island->get("wind");
		for (const Variant &v : Array(p_island->get("hollows"))) {
			const Array hw = v;
			hollows.push_back({ hw[0], hw[1], hw[2] });
		}
		coast_n = p_island->get("coast_n");
		margin = p_island->get("MARGIN");
	}

	static double ellipse(const Vector2 &p_p, const Ellipse &p_l) {
		const Vector2 q = (p_p - p_l.c).rotated(-p_l.a);
		return (q / p_l.r).length();
	}

	double land(const Vector2 &p) const {
		const Vector2 w = Vector2(coast_n->get_noise_2d(p.x, p.y), coast_n->get_noise_2d(p.x + 9.1, p.y - 3.7)) * real_t(0.05);
		const Vector2 q = p + w;
		double f = -1.0;
		for (const Ellipse &l : lobes) {
			f = VU::maxf(f, 1.0 - ellipse(q, l));
		}
		for (const Ellipse &b : bites) {
			f = VU::minf(f, ellipse(q, b) - 1.0);
		}
		f += 0.08 * coast_n->get_noise_2d(q.x * 2.0 + 31.0, q.y * 2.0);
		const double edge = VU::minf(VU::minf(p.x, 1.0 - p.x), VU::minf(p.y, 1.0 - p.y));
		return VU::minf(f, (edge - margin) / 0.03);
	}

	static double line_dist(const Vector2 &p, const PackedVector2Array &p_line) {
		double d = Math::INF;
		for (int k = 0; k < p_line.size() - 1; k++) {
			d = VU::minf(d, p.distance_to(Geometry2D::get_closest_point_to_segment(p, p_line[k], p_line[k + 1])));
		}
		return d;
	}

	double spine_dist(const Vector2 &p) const {
		double d = line_dist(p, spine);
		for (const PackedVector2Array &s : spurs) {
			d = VU::minf(d, line_dist(p, s) * 1.3);
		}
		return d;
	}

	double lee(const Vector2 &p) const {
		double best = Math::INF;
		double s = 0.0;
		for (int k = 0; k < spine.size() - 1; k++) {
			const Vector2 c = Geometry2D::get_closest_point_to_segment(p, spine[k], spine[k + 1]);
			const double d = p.distance_to(c);
			if (d < best) {
				best = d;
				s = (p - c).dot(wind);
			}
		}
		return s;
	}

	double rain(const Vector2 &p, const FastNoiseLite *p_noise, double p_gx, double p_gy) const {
		const double r = 0.64 - 0.56 * VU::smoothstep(-0.01, 0.13, lee(p));
		return r + 0.35 * ((p_noise->get_noise_2d(p_gx, p_gy) * 0.9 + 0.5) - 0.5); // Biomes.rain
	}
};

// foliage_placer.gd's blur_field: down with Lanczos, back up with cubic.
PackedFloat32Array blur_field(const PackedFloat32Array &p_f, int p_size, int p_radius) {
	if (p_radius <= 1) {
		return p_f;
	}
	Vector<uint8_t> bytes;
	bytes.resize(p_f.size() * sizeof(float));
	memcpy(bytes.ptrw(), p_f.ptr(), bytes.size());
	Ref<Image> img = Image::create_from_data(p_size, p_size, false, Image::FORMAT_RF, bytes);
	const int small = MAX(8, p_size / p_radius);
	img->resize(small, small, Image::INTERPOLATE_LANCZOS);
	img->resize(p_size, p_size, Image::INTERPOLATE_CUBIC);
	const Vector<uint8_t> out_bytes = img->get_data();
	PackedFloat32Array out;
	out.resize(out_bytes.size() / sizeof(float));
	memcpy(out.ptrw(), out_bytes.ptr(), out.size() * sizeof(float));
	return out;
}

} // namespace

// world_gen.gd's terrain before erosion, per cell of a px x px grid: the island's land and sea,
// the noise hills, the spine, the lake hollows, and desert mesas where the island's rain is low.
// p_params: px, ox, oy (the grid's first cell), nk (noise cells per grid cell), height_scale,
// sea_level, mesa_height, climate ("desert", "temperate", or anything else for the island's own),
// desert_core, desert_edge (Biomes'), island (an island.gd), and the noises base, ridge, mask,
// rain, mesa. Returns [heights, desert, rain].
Array WorldKernels::island_heights(const Dictionary &p_params) {
	const int px = p_params["px"];
	ERR_FAIL_COND_V(px < 1, Array());
	const int ox = p_params["ox"];
	const int oy = p_params["oy"];
	const double nk = p_params["nk"];
	const double scale = p_params["height_scale"];
	const double sea = p_params["sea_level"];
	const double mesa_height = p_params["mesa_height"];
	const double desert_core = p_params["desert_core"];
	const double desert_edge = p_params["desert_edge"];
	const String climate = p_params["climate"];
	const int climate_override = climate == "desert" ? 1 : (climate == "temperate" ? 2 : 0);
	Object *island_obj = p_params["island"];
	ERR_FAIL_NULL_V(island_obj, Array());
	const IslandShape isl(island_obj);
	const Ref<FastNoiseLite> base_r = p_params["base"];
	const Ref<FastNoiseLite> ridge_r = p_params["ridge"];
	const Ref<FastNoiseLite> mask_r = p_params["mask"];
	const Ref<FastNoiseLite> rain_r = p_params["rain"];
	const Ref<FastNoiseLite> mesa_r = p_params["mesa"];
	ERR_FAIL_COND_V(base_r.is_null() || ridge_r.is_null() || mask_r.is_null() || rain_r.is_null() || mesa_r.is_null() || isl.coast_n.is_null(), Array());
	const FastNoiseLite *base = base_r.ptr();
	const FastNoiseLite *ridge = ridge_r.ptr();
	const FastNoiseLite *mask = mask_r.ptr();
	const FastNoiseLite *rain_n = rain_r.ptr();
	const FastNoiseLite *mesa = mesa_r.ptr();

	PackedFloat32Array heights;
	PackedFloat32Array dry;
	PackedFloat32Array wet;
	heights.resize(int64_t(px) * px);
	dry.resize(heights.size());
	wet.resize(heights.size());
	float *out_h = heights.ptrw();
	float *out_dry = dry.ptrw();
	float *out_wet = wet.ptrw();

	for_each_row(px, [&](int y) {
		const double gy = (oy + y) * nk;
		int64_t k = int64_t(y) * px;
		for (int x = 0; x < px; x++, k++) {
			const double gx = (ox + x) * nk;
			const Vector2 p((x + 0.5) / px, (y + 0.5) / px);
			const double f = isl.land(p);
			const double inland = VU::smoothstep(-0.02, 0.22, f);
			// world_gen._base_height
			const double m = VU::clampf(mask->get_noise_2d(gx, gy) * 1.6 + 0.15, 0.0, 1.0);
			const double r = ridge->get_noise_2d(gx, gy) * 0.5 + 0.5;
			const double hb = base->get_noise_2d(gx, gy) * 0.4 + m * m * r * 0.85;
			const double hn = VU::signf(hb) * VU::pow(VU::absf(hb), 1.35) * scale;
			double hl = hn * 0.7 + 14.0;
			if (hl < 3.0) {
				hl = 3.0 + (hl - 3.0) * 0.15;
			}
			hl += 30.0 * VU::smoothstep(0.0, 0.35, f); // land rises from the coast
			const double t = isl.spine_dist(p) / isl.spine_w;
			const double peaks = 0.6 + 0.5 * mask->get_noise_2d(gx * 3.0, gy * 3.0); // passes and peaks along it
			hl += isl.spine_h * peaks * VU::exp(-t * t) * (0.55 + 0.6 * (ridge->get_noise_2d(gx * 0.7, gy * 0.7) * 0.5 + 0.5));
			for (const Hollow &hw : isl.hollows) {
				const double u = p.distance_to(hw.c) / hw.r;
				hl -= hw.depth * VU::maxf(0.0, 1.0 - u * u);
			}
			const double hs = -6.0 - 40.0 * VU::smoothstep(0.0, 0.3, -f);
			double h = VU::lerpf(hs, hl, inland);
			const double rain = isl.rain(p, rain_n, gx, gy);
			double a = 1.0 - VU::smoothstep(desert_core, desert_edge, rain);
			if (climate_override == 1) {
				a = 1.0;
			} else if (climate_override == 2) {
				a = 0.0;
			}
			a *= inland;
			if (a > 0.0) {
				// world_gen._desert_height
				const double d_inland = VU::smoothstep(sea, sea + 15.0, h);
				const double plain = VU::lerpf(h, sea + 10.0 + (h - sea) * 0.2, d_inland);
				const double mt = mesa->get_noise_2d(gx, gy);
				const double lift = mesa_height * (VU::smoothstep(0.05, 0.12, mt) + 0.8 * VU::smoothstep(0.35, 0.42, mt));
				h = VU::lerpf(h, plain + lift * VU::smoothstep(sea + 8.0, sea + 14.0, plain), a);
			}
			out_h[k] = h;
			out_dry[k] = a;
			out_wet[k] = rain;
		}
	});
	Array out;
	out.push_back(heights);
	out.push_back(dry);
	out.push_back(wet);
	return out;
}

// biomes.gd's climate and weights per cell. p_params: px, heights, water_level, sea_level, slope,
// rain, desert, snow_line, beach (noise), beach_low, beach_high, ox, oy, nk, and the blur radii in cells near_px,
// bank_px, sea_near_px, open_sea_px, relief_px. Returns {moisture, weights}: weights holds the
// five biome fields (PINE, WOODLAND, SCRUB, COAST, ALPINE).
Dictionary WorldKernels::biome_fields(const Dictionary &p_params) {
	const int px = p_params["px"];
	const int64_t n = int64_t(px) * px;
	const PackedFloat32Array hp = p_params["heights"];
	const PackedFloat32Array wlp = p_params["water_level"];
	const PackedFloat32Array slope_p = p_params["slope"];
	const PackedFloat32Array rain_p = p_params["rain"];
	const PackedFloat32Array desert_p = p_params["desert"];
	ERR_FAIL_COND_V(px < 1 || hp.size() != n || wlp.size() != n || slope_p.size() != n || rain_p.size() != n || desert_p.size() != n, Dictionary());
	const double sea = p_params["sea_level"];
	const double snow = p_params["snow_line"];
	const double beach_low = p_params["beach_low"];
	const double beach_high = p_params["beach_high"];
	const Ref<FastNoiseLite> beach_r = p_params["beach"];
	ERR_FAIL_COND_V(beach_r.is_null(), Dictionary());
	const FastNoiseLite *beach_n = beach_r.ptr();
	const int ox = p_params["ox"];
	const int oy = p_params["oy"];
	const double nk = p_params["nk"];
	const float *h = hp.ptr();
	const float *wl = wlp.ptr();

	PackedFloat32Array fresh;
	PackedFloat32Array salt;
	fresh.resize_initialized(n);
	salt.resize_initialized(n);
	float *fr = fresh.ptrw();
	float *sa = salt.ptrw();
	for (int64_t i = 0; i < n; i++) {
		if (h[i] < sea) {
			sa[i] = 1.0f;
		} else if (wl[i] > h[i] - 0.3) {
			fr[i] = 1.0f;
		}
	}
	// The five blurs are independent; they run side by side.
	PackedFloat32Array blurred[5];
	const PackedFloat32Array *blur_src[5] = { &fresh, &fresh, &salt, &salt, &hp };
	const int radii[5] = { p_params["near_px"], p_params["bank_px"], p_params["sea_near_px"], p_params["open_sea_px"], p_params["relief_px"] };
	for_each_row(5, [&](int k) {
		blurred[k] = blur_field(*blur_src[k], px, radii[k]);
	});
	const float *near = blurred[0].ptr();
	const float *bank = blurred[1].ptr();
	const float *sea_near = blurred[2].ptr();
	const float *open_sea = blurred[3].ptr();
	const float *mean_h = blurred[4].ptr();
	const float *slope = slope_p.ptr();
	const float *rain_f = rain_p.ptr();
	const float *desert = desert_p.ptr();

	PackedFloat32Array moisture;
	moisture.resize(n);
	float *mo = moisture.ptrw();
	PackedFloat32Array w[5];
	float *wp[5];
	for (int k = 0; k < 5; k++) {
		w[k].resize(n);
		wp[k] = w[k].ptrw();
	}
	for_each_row(px, [&](int y) {
		int64_t i = int64_t(y) * px;
		for (int x = 0; x < px; x++, i++) {
			const double r = double(h[i]) - double(mean_h[i]);
			double m = rain_f[i];
			m += 0.45 * VU::clampf(near[i] * 4.0, 0.0, 1.0); // next to streams and lakes
			m += 0.15 * VU::smoothstep(0.0, 15.0, -r); // valleys collect water
			m -= 0.25 * VU::smoothstep(0.0, 20.0, r); // ridges drain
			m -= 0.20 * VU::smoothstep(0.5, 1.2, slope[i]); // steep ground drains
			m -= 1.2 * desert[i]; // plateaus and desert plains are dry
			m = VU::maxf(m, 0.7 * VU::clampf(bank[i] * 3.0, 0.0, 1.0)); // every bank stays green, even in desert
			mo[i] = VU::clampf(m, 0.0, 1.0);
			double wide = VU::smoothstep(0.3, 0.7, beach_n->get_noise_2d((ox + x) * nk, (oy + y) * nk) * 0.5 + 0.5);
			wide *= VU::smoothstep(0.2, 0.45, open_sea[i]);
			const double top = sea + VU::lerpf(beach_low, beach_high, wide);
			// sea_near is about 0.5 at the waterline and falls to 0 at COAST_REACH_M inland.
			const double reach = VU::lerpf(0.42, 0.06, wide);
			const float coast_f = (1.0 - VU::smoothstep(sea + 0.5, top, h[i])) * VU::smoothstep(reach - 0.04, reach + 0.04, sea_near[i]) * (1.0 - VU::smoothstep(0.35, 0.8, slope[i]));

			// Biomes.weights, from the stored (float) coast and moisture.
			const double coast = coast_f;
			const double alpine = VU::smoothstep(snow - 50.0, snow, h[i]) * (1.0 - coast);
			const double land = 1.0 - coast - alpine;
			const double mm = mo[i];
			const double scrub = 1.0 - VU::smoothstep(0.22, 0.40, mm);
			const double pine = VU::smoothstep(0.46, 0.62, mm);
			const double wood = VU::maxf(0.0, 1.0 - scrub - pine);
			wp[0][i] = pine * land;
			wp[1][i] = wood * land;
			wp[2][i] = scrub * land;
			wp[3][i] = coast;
			wp[4][i] = alpine;
		}
	});
	Array weights;
	weights.set_typed(Variant::PACKED_FLOAT32_ARRAY, StringName(), Variant());
	for (int k = 0; k < 5; k++) {
		weights.push_back(w[k]);
	}
	Dictionary out;
	out["moisture"] = moisture;
	out["weights"] = weights;
	return out;
}

// The hydrology and water map kernels below are world_gen.gd's _hydrology and water_map.gd's build,
// ported line for line in the same order (rivers are traced and carved one after another, and
// GDScript's sort_custom is matched with the same SortArray), so they match the scripts bit for bit.

namespace {

// The ascending order GDScript's Array.sort_custom(func(a, b): return acc[a] > acc[b]) gives.
struct AccGreater {
	const float *acc = nullptr;
	bool operator()(int32_t p_a, int32_t p_b) const { return acc[p_a] > acc[p_b]; }
};

void sort_by_acc(Vector<int32_t> &r_cells, const float *p_acc) {
	r_cells.sort_custom<AccGreater, true>(AccGreater{ p_acc });
}

uint64_t ticks() {
	return OS::get_singleton()->get_ticks_msec();
}

// world_gen.gd's river tracing and carving: its fields and helpers (_cells, _carve_disc, ...).
struct Rivers {
	int px = 0;
	int64_t n = 0;
	double k = 1.0; // world_gen._k()
	double gs = 1.0;
	double ref = 2.0; // REF_SPACING
	Vector2 origin;
	double sea = 0.0;
	double shore = 0.0; // sea_shore_depth
	double catch_area = 0.0;
	double depth_scale = 0.0;
	double width_scale = 0.0;
	float *h = nullptr;
	const float *orig = nullptr;
	const float *filled = nullptr;
	const float *lake = nullptr;
	const float *acc = nullptr;
	const int32_t *recv = nullptr;
	float *wl_out = nullptr; // _water_level
	float *carved = nullptr;
	int64_t river_cells = 0;

	int cells(double p_n) const { return MAX(1, int(Math::round(p_n * k))); }
	double river_depth(double p_a) const { return VU::clampf(depth_scale * Math::sqrt(p_a / catch_area), depth_scale, 3.5); }
	int river_radius(double p_depth) const { return cells(1 + int(p_depth * width_scale)); }

	Vector3 cell_world(int32_t i, double p_y) const {
		return Vector3(double(origin.x) + (i % px) * gs, p_y, double(origin.y) + (i / px) * gs);
	}

	double lake_level_near(int32_t i) const {
		const int x = i % px;
		const int y = i / px;
		double best = -1e9;
		const int reach = cells(1);
		for (int dy = -reach; dy <= reach; dy++) {
			for (int dx = -reach; dx <= reach; dx++) {
				const int xx = x + dx;
				const int yy = y + dy;
				if (xx < 0 || yy < 0 || xx >= px || yy >= px) {
					continue;
				}
				best = VU::maxf(best, lake[yy * px + xx]);
			}
		}
		return best;
	}

	double min_ring_height(int32_t i, int d) const {
		const int cx = i % px;
		const int cy = i / px;
		double lo = Math::INF;
		for (int dy = -d; dy <= d; dy++) {
			const int y = cy + dy;
			if (y < 0 || y >= px) {
				continue;
			}
			const bool on_edge = Math::abs(dy) == d;
			for (int dx = -d; dx <= d; dx++) {
				if (!on_edge && Math::abs(dx) != d) {
					continue;
				}
				const int x = cx + dx;
				if (x < 0 || x >= px) {
					continue;
				}
				const int32_t j = y * px + x;
				const double hh = lake[j] > -1e8 ? lake[j] : h[j];
				if (hh < lo) {
					lo = hh;
				}
			}
		}
		return lo;
	}

	double wet_extent(int cx, int cy, const Vector2 &p_dir, double p_wl) const {
		const int max_cells = cells(10);
		int m = 0;
		for (int s = 1; s <= max_cells; s++) {
			const int x = cx + int(Math::round(double(p_dir.x) * s));
			const int y = cy + int(Math::round(double(p_dir.y) * s));
			if (x < 0 || y < 0 || x >= px || y >= px) {
				break;
			}
			const int32_t c = y * px + x;
			if (lake[c] > -1e8) {
				break;
			}
			const double hh = h[c];
			if (hh < p_wl - 0.03 && (hh > sea || carved[c] < 1e9 || hh <= sea - shore)) {
				m = s;
			} else {
				break;
			}
		}
		return m;
	}

	void carve_disc(int32_t i, double p_wl, double p_depth, bool p_own, double p_notch = 0.0, double p_sill_wl = -1e9, double p_sill_floor = -1e9) {
		const int r = river_radius(p_depth);
		const int x = i % px;
		const int y = i / px;
		// Disc and falloff reach one REF_SPACING cell past r; clipped at the map's edge.
		const double rim = r * r + k * k;
		for (int dy = -r; dy <= r; dy++) {
			if (y + dy < 0 || y + dy >= px) {
				continue;
			}
			for (int dx = -r; dx <= r; dx++) {
				const int dsq = dx * dx + dy * dy;
				if (dsq > rim || x + dx < 0 || x + dx >= px) {
					continue;
				}
				const int32_t j = i + dy * px + dx;
				if (lake[j] > -1e8) {
					continue;
				}
				const double f = 1.0 - Math::sqrt(double(dsq)) / (r + k) * (1.0 + p_notch);
				double bed = p_wl - p_depth * f;
				double level = p_wl;
				if (bed < p_sill_floor && VU::absf(lake_level_near(j) - p_sill_wl) < 0.5) {
					bed = p_sill_floor;
					level = p_sill_wl;
				}
				if (h[j] > bed) {
					h[j] = bed;
				}
				if (p_own && f > 0.02 && bed < carved[j]) {
					carved[j] = bed;
					wl_out[j] = level;
				}
			}
		}
	}

	// _carve_chain: the chain's water levels, carved.
	PackedFloat32Array carve_chain(const PackedInt32Array &p_chain, bool p_taper) {
		const int head_n = cells(10); // HEAD_CELLS: the head over which depth ramps up
		const int cn = p_chain.size();
		const int32_t *chain = p_chain.ptr();
		PackedFloat32Array wl_arr;
		wl_arr.resize(cn);
		float *wl = wl_arr.ptrw();
		for (int q = 0; q < cn; q++) {
			const int32_t c = chain[q];
			const int r = river_radius(river_depth(acc[c]));
			const double rim = min_ring_height(c, r + cells(1));
			wl[q] = VU::minf(VU::minf(filled[c] - 0.05, h[c] - 0.15), rim - 0.15);
		}
		double sill_wl = -1e9;
		double sill_floor = -1e9;
		if (!p_taper) {
			const double lk = lake_level_near(chain[0]);
			if (lk > -1e8) {
				wl[0] = lk - 0.03;
				const double d0 = river_depth(acc[chain[0]]);
				sill_wl = wl[0];
				sill_floor = wl[0] - d0 * (0.2 + 0.8 / (head_n + 1.0));
			}
		}
		for (int q = 1; q < cn; q++) {
			wl[q] = VU::minf(wl[q], wl[q - 1]);
		}
		const int32_t into = recv[chain[cn - 1]];
		if (into >= 0 && wl_out[into] > -1e8) {
			const double target = wl_out[into];
			for (int q = cn - 1; q >= 0; q--) {
				const double lifted = VU::minf(target, orig[chain[q]] - 0.15);
				if (lifted <= wl[q]) {
					break;
				}
				wl[q] = lifted;
			}
		}
		for (int q = 0; q < cn; q++) {
			wl[q] = VU::maxf(wl[q], sea + 0.02);
		}
		for (int q = 0; q < cn; q++) {
			const int32_t i = chain[q];
			double depth = river_depth(acc[i]);
			double notch = 0.0;
			const double head = VU::minf(1.0, (q + 1.0) / (head_n + 1.0));
			if (p_taper) {
				depth *= 0.7 + 0.3 * head;
				notch = 2.0 * (1.0 - head);
			} else {
				depth *= 0.2 + 0.8 * head;
			}
			const bool guard = q <= head_n;
			carve_disc(i, wl[q], depth, true, notch, guard ? sill_wl : -1e9, guard ? sill_floor : -1e9);
			river_cells++;
		}
		const int taper_n = cells(8);
		const int32_t mouth = chain[cn - 1];
		if (into >= 0 && filled[into] <= sea - shore) {
			const int32_t from = chain[MAX(cn - cells(5), 0)];
			const Vector2 dir = Vector2(mouth % px - from % px, mouth / px - from / px).normalized();
			const double depth0 = river_depth(acc[mouth]);
			for (int t = 1; t <= taper_n; t++) {
				const int x = mouth % px + int(Math::round(double(dir.x) * t));
				const int y = mouth / px + int(Math::round(double(dir.y) * t));
				if (x < 0 || y < 0 || x >= px || y >= px || filled[y * px + x] > sea) {
					break;
				}
				carve_disc(y * px + x, wl[cn - 1], depth0 * (1.0 - double(t) / (taper_n + 1)), false);
			}
		}
		return wl_arr;
	}

	// _trace_rivers: every river from its mouth up its biggest tributary, the main rivers first.
	Array trace(const PackedByteArray &p_is_river) {
		const int stride = cells(5);
		const int min_cells = cells(8);
		const uint8_t *is_river = p_is_river.ptr();
		Vector<int32_t> river_list;
		LocalVector<int32_t> best_up;
		best_up.resize(n);
		for (int64_t i = 0; i < n; i++) {
			best_up[i] = -1;
		}
		Vector<int32_t> queue;
		for (int64_t i = 0; i < n; i++) {
			if (!is_river[i]) {
				continue;
			}
			river_list.push_back(i);
			const int32_t r = recv[i];
			if (r >= 0 && is_river[r]) {
				if (best_up[r] < 0 || acc[i] > acc[best_up[r]]) {
					best_up[r] = i;
				}
			} else {
				queue.push_back(i);
			}
		}
		sort_by_acc(queue, acc);

		LocalVector<uint8_t> visited;
		visited.resize_initialized(n);
		Array paths;
		while (!queue.is_empty()) {
			for (const int32_t start : queue) {
				if (visited[start]) {
					continue;
				}
				PackedInt32Array chain_arr;
				int32_t c = start;
				while (c >= 0 && !visited[c]) {
					visited[c] = 1;
					chain_arr.push_back(c);
					c = best_up[c];
				}
				if (chain_arr.size() < min_cells) {
					continue;
				}
				chain_arr.reverse();
				const int32_t *chain = chain_arr.ptr();
				const int cn = chain_arr.size();

				const bool taper = !(lake_level_near(chain[0]) > -1e8);
				const PackedFloat32Array wl_chain_arr = carve_chain(chain_arr, taper);
				const float *wl_chain = wl_chain_arr.ptr();
				PackedVector3Array pts;
				PackedFloat32Array widths;
				PackedFloat32Array accs;
				double last_w = 0.0;

				const int c3i = cells(3);
				if (taper && cn > c3i) {
					const int32_t c0 = chain[0];
					const int32_t c3 = chain[c3i];
					const Vector2 back = Vector2(c0 % px - c3 % px, c0 / px - c3 / px).normalized();
					Vector3 p0 = cell_world(c0, wl_chain[0] - 0.3);
					p0.x = double(p0.x) + double(back.x) * ref;
					p0.z = double(p0.z) + double(back.y) * ref;
					pts.push_back(p0);
					widths.push_back(0.5);
					accs.push_back(acc[c0]);
				}
				for (int q = 0; q < cn; q++) {
					if (q % stride != 0 && q != cn - 1) {
						continue;
					}
					const int32_t i = chain[q];
					const double wl = wl_chain[q];
					const int32_t prev_c = chain[MAX(q - stride, 0)];
					const int32_t next_c = chain[MIN(q + stride, cn - 1)];
					const Vector2 flow(next_c % px - prev_c % px, next_c / px - prev_c / px);
					const Vector2 perp = flow.length_squared() > 0.0f ? Vector2(-flow.y, flow.x).normalized() : Vector2(1, 0);
					const int cx = i % px;
					const int cy = i / px;
					const double left = wet_extent(cx, cy, -perp, wl);
					const double right = wet_extent(cx, cy, perp, wl);
					Vector2 shift = perp * real_t(VU::clampf((right - left) * 0.5, -2.0 * k, 2.0 * k));
					if (!taper && q == 0) {
						shift = Vector2();
					}
					Vector3 p = cell_world(i, wl);
					p.x = double(p.x) + double(shift.x) * gs;
					p.z = double(p.z) + double(shift.y) * gs;
					double w = VU::maxf((left + right) * 0.5 * gs + ref, 2.0);
					if (last_w > 0.0) {
						w = VU::minf(w, last_w * 1.5);
					} else if (!taper) {
						w = VU::minf(w, (k + river_radius(river_depth(acc[i]))) * gs);
					}
					last_w = w;
					pts.push_back(p);
					widths.push_back(w);
					accs.push_back(acc[i]);
				}

				if (!taper) {
					// Start the path inside the lake, at the lake cell the river leaves from.
					const int32_t c0 = chain[0];
					const int32_t cn1 = chain[MIN(stride, cn - 1)];
					const Vector2 flow = Vector2(cn1 % px - c0 % px, cn1 / px - c0 / px).normalized();
					int32_t from = -1;
					double best = -Math::INF;
					const int reach = cells(1);
					for (int dy = -reach; dy <= reach; dy++) {
						for (int dx = -reach; dx <= reach; dx++) {
							const int xx = c0 % px + dx;
							const int yy = c0 / px + dy;
							if (xx < 0 || yy < 0 || xx >= px || yy >= px || (dx == 0 && dy == 0)) {
								continue;
							}
							const int32_t j = yy * px + xx;
							if (lake[j] > -1e8) {
								const double align = -double(Vector2(dx, dy).normalized().dot(flow));
								if (align > best) {
									best = align;
									from = j;
								}
							}
						}
					}
					if (from >= 0 && best >= 0.7 && pts.size() > 0) {
						pts.insert(0, cell_world(from, pts[0].y));
						widths.insert(0, widths[0]);
						accs.insert(0, accs[0]);
					}
				}

				const int32_t mouth = chain[cn - 1];
				const int32_t into = recv[mouth];
				if (into >= 0 && pts.size() >= 2) {
					const Vector3 last = pts[pts.size() - 1];
					Vector3 dir = last - pts[pts.size() - 2];
					dir.y = 0.0;
					if (dir.length_squared() > 0.0f) {
						Vector3 p = last + dir.normalized() * real_t(ref);
						p.y = wl_out[into] > -1e8 ? double(wl_out[into]) - 0.05 : sea + 0.02;
						pts.push_back(p);
						widths.push_back(widths[widths.size() - 1]);
						accs.push_back(acc[mouth]);
					}
				}
				Dictionary path;
				path["points"] = pts;
				path["widths"] = widths;
				path["accs"] = accs;
				path["cells"] = chain_arr;
				path["taper"] = taper;
				paths.push_back(path);
			}
			// Then the tributaries that join what is traced so far.
			queue.clear();
			for (const int32_t i : river_list) {
				if (!visited[i] && recv[i] >= 0 && visited[recv[i]]) {
					queue.push_back(i);
				}
			}
			sort_by_acc(queue, acc);
		}
		return paths;
	}
};

} // namespace

// world_gen.gd's _hydrology after the flood: D8 flow and accumulation, lakes, then rivers traced
// and carved. p_params: px, gs (vertex spacing), ref_spacing, origin, heights, filled, spill (the
// flood's), desert, sea_level, sea_shore_depth, catch, desert_river_catchment, desert_lake_drop,
// desert_lake_cells, desert_lake_depth, lake_min_cells, lake_min_depth, river_depth_scale,
// river_width_scale. Returns {heights, orig (after the lakes, before the rivers), water_level,
// lake_level, carved, acc, river_paths, river_cells, lakes_kept, basins, ms}.
Dictionary WorldKernels::hydrology(const Dictionary &p_params) {
	const int px = p_params["px"];
	const int64_t n = int64_t(px) * px;
	PackedFloat32Array heights = p_params["heights"];
	const PackedFloat32Array filled_p = p_params["filled"];
	const PackedFloat32Array spill_p = p_params["spill"];
	const PackedFloat32Array desert_p = p_params["desert"];
	ERR_FAIL_COND_V(px < 3 || n >= (1 << 24) + 1 || heights.size() != n || filled_p.size() != n || spill_p.size() != n || desert_p.size() != n, Dictionary());
	Rivers rv;
	rv.px = px;
	rv.n = n;
	rv.gs = p_params["gs"];
	rv.ref = p_params["ref_spacing"];
	rv.k = rv.ref / rv.gs;
	rv.origin = p_params["origin"];
	rv.sea = p_params["sea_level"];
	rv.shore = p_params["sea_shore_depth"];
	rv.catch_area = p_params["catch"];
	rv.depth_scale = p_params["river_depth_scale"];
	rv.width_scale = p_params["river_width_scale"];
	const double sea = rv.sea;
	const double desert_river_catchment = p_params["desert_river_catchment"];
	const double desert_lake_drop = p_params["desert_lake_drop"];
	const double desert_lake_cells = p_params["desert_lake_cells"];
	const double desert_lake_depth = p_params["desert_lake_depth"];
	const double lake_min_cells = p_params["lake_min_cells"];
	const double lake_min_depth = p_params["lake_min_depth"];
	const double k = rv.k;
	auto area = [k](double p_n) { return p_n * k * k; }; // world_gen._area
	const float *filled = filled_p.ptr();
	const float *spill = spill_p.ptr();
	const float *desert = desert_p.ptr();
	float *h = heights.ptrw();
	Dictionary ms;
	uint64_t t = ticks();

	// D8 from the highest filled cell down: each cell drains to its steepest lower neighbour.
	PackedInt64Array keys;
	keys.resize(n);
	int64_t *kp = keys.ptrw();
	int64_t kc = 0;
	for (int64_t i = 0; i < n; i++) {
		if (filled[i] > sea - rv.shore) {
			kp[kc++] = int64_t(uint64_t(int64_t(double(filled[i]) * 1000000.0)) << 24) | i;
		}
	}
	keys.resize(kc);
	keys.sort();
	kp = keys.ptrw();
	const int32_t offs[8] = { -px - 1, -px, -px + 1, -1, 1, px - 1, px, px + 1 };
	const double inv_dist[8] = { 0.7071f, 1.0, 0.7071f, 1.0, 1.0, 0.7071f, 1.0, 0.7071f };
	PackedFloat32Array acc_p;
	acc_p.resize(n);
	acc_p.fill(1.0);
	float *acc = acc_p.ptrw();
	PackedInt32Array recv_p;
	recv_p.resize(n);
	recv_p.fill(-1);
	int32_t *recv = recv_p.ptrw();
	for (int64_t q = kc - 1; q >= 0; q--) {
		const int32_t i = int32_t(kp[q] & 0xFFFFFF);
		const int x = i % px;
		const int y = i / px;
		if (x == 0 || y == 0 || x == px - 1 || y == px - 1) {
			continue;
		}
		const double fc = filled[i];
		int32_t r = -1;
		double best = 0.0;
		for (int d = 0; d < 8; d++) {
			const int32_t j = i + offs[d];
			const double drop = (fc - filled[j]) * inv_dist[d];
			if (drop > best) {
				best = drop;
				r = j;
			}
		}
		if (r >= 0) {
			acc[r] = double(acc[r]) + double(acc[i]);
			recv[i] = r;
		}
	}
	PackedFloat32Array catch_min_p;
	catch_min_p.resize(n);
	float *catch_min = catch_min_p.ptrw();
	const double extra = rv.catch_area * (desert_river_catchment - 1.0);
	for (int64_t i = 0; i < n; i++) {
		catch_min[i] = rv.catch_area + extra * desert[i];
	}
	ms["h_d8"] = ticks() - t;
	t = ticks();

	// Lakes: each basin the flood filled, if it is big and deep enough (a river through it keeps
	// a smaller one), stands at its outlet's level over the ground under it, its floor dug a little
	// under it.
	const double lake_floor = 1.5;
	const double lake_above_sea = 0.3; // or the basin is left to the sea
	PackedFloat32Array water_level_p;
	water_level_p.resize(n);
	water_level_p.fill(-1e9);
	float *wlv = water_level_p.ptrw();
	PackedFloat32Array lake_p;
	lake_p.resize(n);
	lake_p.fill(-1e9);
	float *lake = lake_p.ptrw();
	LocalVector<uint8_t> is_filled;
	is_filled.resize_initialized(n);
	for (int64_t i = 0; i < n; i++) {
		// By where the water stands, not the ground: a basin can reach below sea level inland.
		if (filled[i] > sea && filled[i] - double(h[i]) > 0.05) {
			is_filled[i] = 1;
		}
	}
	LocalVector<uint8_t> visited;
	visited.resize_initialized(n);
	LocalVector<int32_t> stack;
	stack.resize(n);
	LocalVector<int32_t> comp;
	const int32_t offs4[4] = { -1, 1, -px, px };
	int basins = 0;
	int kept = 0;
	for (int64_t i = 0; i < n; i++) {
		if (!is_filled[i] || visited[i]) {
			continue;
		}
		comp.clear();
		int64_t sp = 0;
		stack[sp++] = i;
		visited[i] = 1;
		bool has_river = false;
		while (sp > 0) {
			const int32_t c = stack[--sp];
			comp.push_back(c);
			if (acc[c] >= catch_min[c]) {
				has_river = true;
			}
			for (int q = 0; q < 4; q++) {
				const int64_t d = c + offs4[q];
				if (d >= 0 && d < n && is_filled[d] && !visited[d]) {
					visited[d] = 1;
					stack[sp++] = d;
				}
			}
		}
		basins++;
		// One level for the whole basin, its outlet's: the lowest spill in it. Not each cell's own
		// spill: the flood's rise across a big flat (1.7 m on 417) reaches parts of one basin over low
		// bumps, which would split the lake into steps, and lift its water over dry shore.
		double outlet = 1e30;
		double lowest = 1e30;
		for (const int32_t c : comp) {
			outlet = VU::minf(outlet, double(spill[c]));
			lowest = VU::minf(lowest, double(h[c]));
		}
		const double dry = desert[i];
		double drop = 0.0;
		if (has_river) {
			if (comp.size() < area(12)) {
				continue;
			}
		} else {
			drop = desert_lake_drop * dry;
			int64_t wet = 0;
			for (const int32_t c : comp) {
				if (h[c] < outlet - drop) {
					wet++;
				}
			}
			if (wet < area(lake_min_cells) * VU::lerpf(1.0, desert_lake_cells, dry) || outlet - lowest < lake_min_depth * VU::lerpf(1.0, desert_lake_depth, dry)) {
				continue;
			}
		}
		const double level = outlet - drop;
		// A basin whose water would not stand clear of the sea is the sea's.
		if (level < sea + lake_above_sea) {
			continue;
		}
		kept++;
		for (const int32_t c : comp) {
			if (h[c] >= level) {
				continue;
			}
			lake[c] = level;
			wlv[c] = level;
			const double dig = VU::minf(lake_floor, (level - double(h[c])) * 3.0);
			h[c] = VU::minf(h[c], level - dig);
		}
	}
	ms["h_lakes"] = ticks() - t;
	t = ticks();

	// Rivers: cells draining enough of the island that are not lake or open sea.
	const PackedFloat32Array orig = heights;
	PackedFloat32Array carved_p;
	carved_p.resize(n);
	carved_p.fill(1e9);
	h = heights.ptrw(); // orig shares the data until this write
	PackedByteArray is_river;
	is_river.resize(n);
	uint8_t *isr = is_river.ptrw();
	for (int64_t i = 0; i < n; i++) {
		isr[i] = acc[i] >= catch_min[i] && h[i] > sea - rv.shore && lake[i] < -1e8;
	}
	rv.h = h;
	rv.orig = orig.ptr();
	rv.filled = filled;
	rv.lake = lake;
	rv.acc = acc;
	rv.recv = recv;
	rv.wl_out = wlv;
	rv.carved = carved_p.ptrw();
	const Array paths = rv.trace(is_river);
	ms["h_trace_carve"] = ticks() - t;

	Dictionary out;
	out["heights"] = heights;
	out["orig"] = orig;
	out["water_level"] = water_level_p;
	out["lake_level"] = lake_p;
	out["carved"] = carved_p;
	out["acc"] = acc_p;
	out["river_paths"] = paths;
	out["river_cells"] = rv.river_cells;
	out["lakes_kept"] = kept;
	out["basins"] = basins;
	out["ms"] = ms;
	return out;
}

// foliage_placer.gd's ring_distance: for each cell, the number of 8-neighbour rings out to the
// nearest cell with a level (> -1e8), up to p_rings; p_rings + 1 past that.
PackedInt32Array WorldKernels::ring_distance(const PackedFloat32Array &p_levels, int p_size, int p_rings) {
	const int64_t n = int64_t(p_size) * p_size;
	ERR_FAIL_COND_V(p_size < 1 || p_levels.size() != n, PackedInt32Array());
	const float *levels = p_levels.ptr();
	PackedInt32Array dist_p;
	dist_p.resize(n);
	dist_p.fill(p_rings + 1);
	int32_t *dist = dist_p.ptrw();
	LocalVector<int32_t> frontier;
	LocalVector<int32_t> next;
	for (int64_t i = 0; i < n; i++) {
		if (levels[i] > -1e8) {
			dist[i] = 0;
			frontier.push_back(i);
		}
	}
	for (int r = 1; r <= p_rings; r++) {
		next.clear();
		for (const int32_t i : frontier) {
			const int x = i % p_size;
			const int y = i / p_size;
			const int x0 = MAX(x - 1, 0);
			const int x1 = MIN(x + 1, p_size - 1);
			const int y0 = MAX(y - 1, 0);
			const int y1 = MIN(y + 1, p_size - 1);
			for (int yy = y0; yy <= y1; yy++) {
				for (int xx = x0; xx <= x1; xx++) {
					const int32_t j = yy * p_size + xx;
					if (dist[j] > r) {
						dist[j] = r;
						next.push_back(j);
					}
				}
			}
		}
		SWAP(frontier, next);
	}
	return dist_p;
}

// water_map.gd's build (see its header for what the maps hold). p_params: heights, water_level,
// lake_level, sea, px, paths, catch, spacing, origin, and water_map.gd's constants flood_steps,
// flow_max, no_water, depth_max, exposure_cell, exposure_dirs, exposure_reach, river_bank.
// Returns {level, info, depth, exposure, exposure_px, stats}.
Dictionary WorldKernels::water_map(const Dictionary &p_params) {
	const int px = p_params["px"];
	const int64_t n = int64_t(px) * px;
	const PackedFloat32Array heights_p = p_params["heights"];
	const PackedFloat32Array water_level = p_params["water_level"];
	const PackedFloat32Array lake_p = p_params["lake_level"];
	ERR_FAIL_COND_V(px < 3 || heights_p.size() != n || water_level.size() != n || lake_p.size() != n, Dictionary());
	const double sea = p_params["sea"];
	const Array paths = p_params["paths"];
	const double catch_area = p_params["catch"];
	const double spacing = p_params["spacing"];
	const Vector2 origin = p_params["origin"];
	const int flood_steps = p_params["flood_steps"];
	const double flow_max = p_params["flow_max"];
	const double no_water = p_params["no_water"];
	const double depth_max = p_params["depth_max"];
	const int exposure_cell = p_params["exposure_cell"];
	const int exposure_dirs = p_params["exposure_dirs"];
	const double exposure_reach = p_params["exposure_reach"];
	const double river_bank = p_params["river_bank"];
	const float *heights = heights_p.ptr();
	const float *lake = lake_p.ptr();
	Dictionary stats;
	uint64_t t = ticks();

	PackedByteArray info_p;
	info_p.resize(n * 4);
	info_p.fill(0);
	uint8_t *info = info_p.ptrw();
	for (int64_t i = 0; i < n; i++) {
		info[i * 4] = 128;
		info[i * 4 + 1] = 128;
	}
	// _paint_rivers: centre-line level, flow and discharge along every river path, out to
	// river_bank past its banks. The bigger river wins, and within one the nearest centre line.
	LocalVector<float> river_level;
	river_level.resize(n);
	for (int64_t i = 0; i < n; i++) {
		river_level[i] = no_water;
	}
	{
		LocalVector<float> owner_size;
		owner_size.resize_initialized(n);
		LocalVector<float> owner_dist;
		owner_dist.resize_initialized(n);
		for (const Variant &v : paths) {
			const Dictionary rp = v;
			const PackedVector3Array pts = rp["points"];
			const PackedFloat32Array widths = rp["widths"];
			const PackedFloat32Array accs = rp["accs"];
			double size = 0.0;
			for (const float a : accs) {
				size = VU::maxf(size, a);
			}
			for (int s = 0; s < pts.size() - 1; s++) {
				const Vector3 a = pts[s];
				const Vector3 b = pts[s + 1];
				const Vector2 run(double(b.x) - double(a.x), double(b.z) - double(a.z));
				const double len = run.length();
				if (len < 0.01) {
					continue;
				}
				const Vector2 dir = run / real_t(len);
				const double grade = VU::maxf(double(a.y) - double(b.y), 0.0) / len;
				const double speed = VU::clampf(0.5 + 8.0 * grade, 0.5, flow_max);
				const int fx = CLAMP(int(128.0 + dir.x * speed / flow_max * 127.0), 0, 255);
				const int fz = CLAMP(int(128.0 + dir.y * speed / flow_max * 127.0), 0, 255);
				const double r = VU::maxf(widths[s], widths[s + 1]) * 0.5 + river_bank;
				const int x0 = MAX(int(Math::floor((VU::minf(a.x, b.x) - r - origin.x) / spacing)), 0);
				const int x1 = MIN(int(Math::ceil((VU::maxf(a.x, b.x) + r - origin.x) / spacing)), px - 1);
				const int y0 = MAX(int(Math::floor((VU::minf(a.z, b.z) - r - origin.y) / spacing)), 0);
				const int y1 = MIN(int(Math::ceil((VU::maxf(a.z, b.z) + r - origin.y) / spacing)), px - 1);
				const Vector2 a2(a.x, a.z);
				for (int y = y0; y <= y1; y++) {
					for (int x = x0; x <= x1; x++) {
						const int64_t i = int64_t(y) * px + x;
						const Vector2 p(double(origin.x) + x * spacing, double(origin.y) + y * spacing);
						const double f = VU::clampf(double((p - a2).dot(dir)) / len, 0.0, 1.0);
						const double dist = p.distance_to(Vector2(VU::lerpf(a.x, b.x, f), VU::lerpf(a.z, b.z, f)));
						if (dist > r || size < owner_size[i] || (size == owner_size[i] && dist >= owner_dist[i])) {
							continue;
						}
						owner_size[i] = size;
						owner_dist[i] = dist;
						river_level[i] = VU::lerpf(a.y, b.y, f);
						info[i * 4] = fx;
						info[i * 4 + 1] = fz;
						info[i * 4 + 2] = uint8_t(int(VU::clampf(Math::sqrt(VU::lerpf(accs[s], accs[s + 1], f) / catch_area) / 8.0, 0.0, 1.0) * 255.0));
					}
				}
			}
		}
	}
	stats["map_rivers_ms"] = ticks() - t;
	t = ticks();

	// river: the texel's level is its river's centre line, and so is the level it hands on.
	LocalVector<uint8_t> river;
	river.resize_initialized(n);
	PackedFloat32Array level_p = water_level;
	float *level = level_p.ptrw();
	LocalVector<uint8_t> draw;
	draw.resize_initialized(n);
	for (int64_t i = 0; i < n; i++) {
		if (level[i] > -1e8) {
			draw[i] = 1;
			if (river_level[i] > -1e8 && lake[i] < -1e8) {
				level[i] = river_level[i];
				river[i] = 1;
			}
		} else if (heights[i] < sea) {
			level[i] = sea;
			draw[i] = 1;
		} else {
			level[i] = no_water;
		}
	}
	// Edge texels: water with a dry 8-neighbour.
	LocalVector<int32_t> front;
	for (int y = 1; y < px - 1; y++) {
		const int64_t row = int64_t(y) * px;
		for (int x = 1; x < px - 1; x++) {
			const int64_t i = row + x;
			if (draw[i] == 1 && (draw[i - 1] == 0 || draw[i + 1] == 0 || draw[i - px] == 0 || draw[i + px] == 0 || draw[i - px - 1] == 0 || draw[i - px + 1] == 0 || draw[i + px - 1] == 0 || draw[i + px + 1] == 0)) {
				front.push_back(i);
			}
		}
	}
	const int32_t n4[4] = { -1, 1, -px, px };
	const int32_t n8[8] = { -1, 1, -px, px, -px - 1, -px + 1, px - 1, px + 1 };
	// A texel next to a river takes the river's level there; next to a lake or the sea, theirs.
	auto handed = [&](int64_t i, int64_t j) -> double {
		return river[i] == 1 && river_level[j] > -1e8 ? double(river_level[j]) : double(level[i]);
	};
	int64_t flooded = 0;
	LocalVector<int32_t> next;
	for (int step = 0; step < flood_steps; step++) {
		next.clear();
		for (const int32_t i : front) {
			for (int q = 0; q < 4; q++) {
				const int64_t j = int64_t(i) + n4[q];
				if (j < 0 || j >= n || draw[j] != 0) {
					continue;
				}
				const double l = handed(i, j);
				if (heights[j] >= l) {
					continue;
				}
				if (level[j] == no_water) {
					next.push_back(j);
					level[j] = l;
					river[j] = river[i];
				} else if (l < level[j]) {
					level[j] = l;
				}
			}
		}
		for (const int32_t j : next) {
			draw[j] = 1;
		}
		flooded += next.size();
		for (const int32_t j : next) {
			front.push_back(j);
		}
	}
	stats["map_flooded"] = flooded;

	// The ring under the bank (see water_map.gd): dry ground at or above a neighbour's level takes
	// the lowest such level; ground still below one is an open edge.
	LocalVector<int32_t> ring;
	int64_t open = 0;
	double open_worst = 0.0;
	int64_t open_at = -1;
	for (const int32_t i : front) {
		for (int q = 0; q < 8; q++) {
			const int64_t j = int64_t(i) + n8[q];
			if (j < 0 || j >= n || draw[j] == 1) {
				continue;
			}
			const double l = handed(i, j);
			if (heights[j] >= l) {
				if (level[j] == no_water) {
					ring.push_back(j);
					level[j] = l;
				} else {
					level[j] = VU::minf(level[j], l);
				}
			} else if (l - heights[j] > 0.05) {
				open++;
				if (l - heights[j] > open_worst) {
					open_worst = l - heights[j];
					open_at = j;
				}
			}
		}
	}
	for (const int32_t j : ring) {
		draw[j] = 2;
	}
	stats["map_ring"] = ring.size();
	stats["map_open_edges"] = open;
	stats["map_open_worst_m"] = VU::snappedf(open_worst, 0.01);
	if (open_at >= 0) {
		stats["map_open_worst_at"] = vformat("(%.0f, %.0f)", double(origin.x) + (open_at % px) * spacing, double(origin.y) + (open_at / px) * spacing);
	}
	stats["map_shore_ms"] = ticks() - t;
	t = ticks();

	PackedByteArray depth_p;
	depth_p.resize(n);
	depth_p.fill(0);
	uint8_t *depth = depth_p.ptrw();
	for (int64_t i = 0; i < n; i++) {
		if (draw[i] == 1 && level[i] > heights[i]) {
			depth[i] = uint8_t(int(Math::sqrt(VU::minf((double(level[i]) - double(heights[i])) / depth_max, 1.0)) * 255.0));
		}
	}
	// Openness per exposure_cell cell: water cells are those whose centre texel is water; a run
	// stops at the first dry cell; past the world's edge is open sea.
	const int ep = px / exposure_cell;
	LocalVector<uint8_t> wet;
	wet.resize(ep * ep);
	const int half_cell = exposure_cell / 2;
	for (int y = 0; y < ep; y++) {
		for (int x = 0; x < ep; x++) {
			wet[y * ep + x] = draw[int64_t(y * exposure_cell + half_cell) * px + x * exposure_cell + half_cell] == 1 ? 1 : 0;
		}
	}
	const int steps = int(exposure_reach / exposure_cell);
	LocalVector<Vector2> dirs;
	for (int d = 0; d < exposure_dirs; d++) {
		dirs.push_back(Vector2::from_angle(Math::TAU * d / exposure_dirs));
	}
	PackedByteArray exposure_p;
	exposure_p.resize(int64_t(ep) * ep);
	uint8_t *exposure = exposure_p.ptrw();
	for_each_row(ep, [&](int y) {
		for (int x = 0; x < ep; x++) {
			int open_run = 0;
			for (const Vector2 &dir : dirs) {
				int run = steps;
				for (int s = 1; s <= steps; s++) {
					const int cx = int(Math::round(x + double(dir.x) * s));
					const int cy = int(Math::round(y + double(dir.y) * s));
					if (cx < 0 || cy < 0 || cx >= ep || cy >= ep) {
						break;
					}
					if (wet[cy * ep + cx] == 0) {
						run = s - 1;
						break;
					}
				}
				open_run += run;
			}
			exposure[y * ep + x] = uint8_t(int(VU::pow(double(open_run) / (steps * exposure_dirs), 1.5) * 255.0));
		}
	});
	stats["map_waves_ms"] = ticks() - t;
	t = ticks();

	// _fill, pull-push: texels with no water take the average level of the nearest coarser block
	// that has some, so a coarse ring vertex on land sits at the level of the water beside it.
	LocalVector<LocalVector<float>> pyramid;
	LocalVector<int> sizes;
	pyramid.resize(1);
	sizes.push_back(px);
	int size = px;
	while (size > 1) {
		const int half = size / 2;
		pyramid.resize(pyramid.size() + 1);
		LocalVector<float> &dst = pyramid[pyramid.size() - 1];
		dst.resize(int64_t(half) * half);
		const float *src = pyramid.size() == 2 ? level : pyramid[pyramid.size() - 2].ptr();
		for (int y = 0; y < half; y++) {
			const int64_t r0 = int64_t(2 * y) * size;
			const int64_t r1 = r0 + size;
			for (int x = 0; x < half; x++) {
				double sum = 0.0;
				int c = 0;
				const float v4[4] = { src[r0 + 2 * x], src[r0 + 2 * x + 1], src[r1 + 2 * x], src[r1 + 2 * x + 1] };
				for (const float v : v4) {
					if (v != no_water) {
						sum += v;
						c++;
					}
				}
				dst[y * half + x] = c > 0 ? sum / c : no_water;
			}
		}
		sizes.push_back(half);
		size = half;
	}
	for (int lv = int(pyramid.size()) - 2; lv >= 0; lv--) {
		float *fine = lv == 0 ? level : pyramid[lv].ptr();
		const float *coarse = pyramid[lv + 1].ptr();
		const int fs = sizes[lv];
		const int cs = sizes[lv + 1];
		for (int y = 0; y < fs; y++) {
			const int64_t crow = int64_t(MIN(y / 2, cs - 1)) * cs;
			for (int x = 0; x < fs; x++) {
				const int64_t i = int64_t(y) * fs + x;
				if (fine[i] == no_water) {
					fine[i] = coarse[crow + MIN(x / 2, cs - 1)];
				}
			}
		}
	}
	for (int64_t i = 0; i < n; i++) {
		if (draw[i] != 0) {
			info[i * 4 + 3] = 255;
		}
	}
	stats["map_fill_ms"] = ticks() - t;
	Dictionary out;
	out["level"] = level_p;
	out["info"] = info_p;
	out["depth"] = depth_p;
	out["exposure"] = exposure_p;
	out["exposure_px"] = ep;
	out["stats"] = stats;
	return out;
}

// The erosion and control map kernels below are world_gen.gd's _erode_and_smooth, _resample_grid
// and _build_control, ported line for line (droplets one after another, as the script runs them).

namespace {

// _resample_grid: output cell (x, y) reads the input bilinearly at (x * step, y * step).
PackedFloat32Array resample(const PackedFloat32Array &p_src, int p_src_px, int p_dst_px, double p_step) {
	const float *src = p_src.ptr();
	PackedFloat32Array out_p;
	out_p.resize(int64_t(p_dst_px) * p_dst_px);
	float *out = out_p.ptrw();
	const int last = p_src_px - 1;
	for_each_row(p_dst_px, [&](int y) {
		const double fy = VU::minf(y * p_step, last);
		const int iy = MIN(int(fy), last - 1);
		const double ty = fy - iy;
		const int64_t r0 = int64_t(iy) * p_src_px;
		const int64_t r1 = r0 + p_src_px;
		int64_t k = int64_t(y) * p_dst_px;
		for (int x = 0; x < p_dst_px; x++, k++) {
			const double fx = VU::minf(x * p_step, last);
			const int ix = MIN(int(fx), last - 1);
			const double tx = fx - ix;
			const double a = src[r0 + ix] + (double(src[r0 + ix + 1]) - double(src[r0 + ix])) * tx;
			const double b = src[r1 + ix] + (double(src[r1 + ix + 1]) - double(src[r1 + ix])) * tx;
			out[k] = a + (b - a) * ty;
		}
	});
	return out_p;
}

// _erode: droplets, each stepping across and carving the ground the ones before it left.
void erode_grid(float *h, int size, int64_t p_drops, int p_max_steps, int64_t p_seed) {
	Ref<RandomNumberGenerator> rng;
	rng.instantiate();
	rng->set_seed(p_seed);
	const double INERTIA = 0.05;
	const double CAPACITY = 4.0;
	const double MIN_CAPACITY = 0.01;
	const double DEPOSIT = 0.3;
	const double ERODE = 0.3;
	const double GRAVITY = 4.0;
	const double EVAPORATE = 0.01;
	const double lim = double(size - 2);
	for (int64_t d = 0; d < p_drops; d++) {
		double px = rng->randf_range(1.0, lim);
		double py = rng->randf_range(1.0, lim);
		double dx = 0.0;
		double dy = 0.0;
		double speed = 1.0;
		double water = 1.0;
		double sediment = 0.0;
		for (int s = 0; s < p_max_steps; s++) {
			const int xi = int(px);
			const int yi = int(py);
			const double fx = px - xi;
			const double fy = py - yi;
			const int64_t i = int64_t(yi) * size + xi;
			const double h00 = h[i];
			const double h10 = h[i + 1];
			const double h01 = h[i + size];
			const double h11 = h[i + size + 1];
			const double gx = (h10 - h00) * (1.0 - fy) + (h11 - h01) * fy;
			const double gy = (h01 - h00) * (1.0 - fx) + (h11 - h10) * fx;
			const double height = h00 * (1.0 - fx) * (1.0 - fy) + h10 * fx * (1.0 - fy) + h01 * (1.0 - fx) * fy + h11 * fx * fy;

			dx = dx * INERTIA - gx * (1.0 - INERTIA);
			dy = dy * INERTIA - gy * (1.0 - INERTIA);
			const double len = Math::sqrt(dx * dx + dy * dy);
			if (len < 1e-6) {
				const double a = rng->randf() * Math::TAU;
				dx = Math::cos(a);
				dy = Math::sin(a);
			} else {
				dx /= len;
				dy /= len;
			}
			px += dx;
			py += dy;
			if (px < 1.0 || px >= lim || py < 1.0 || py >= lim) {
				break;
			}

			const int nxi = int(px);
			const int nyi = int(py);
			const double nfx = px - nxi;
			const double nfy = py - nyi;
			const int64_t ni = int64_t(nyi) * size + nxi;
			const double new_height = h[ni] * (1.0 - nfx) * (1.0 - nfy) + h[ni + 1] * nfx * (1.0 - nfy) + h[ni + size] * (1.0 - nfx) * nfy + h[ni + size + 1] * nfx * nfy;
			const double dh = new_height - height;

			const double capacity = VU::maxf(-dh, MIN_CAPACITY) * speed * water * CAPACITY;
			const double w00 = (1.0 - fx) * (1.0 - fy);
			const double w10 = fx * (1.0 - fy);
			const double w01 = (1.0 - fx) * fy;
			const double w11 = fx * fy;
			if (sediment > capacity || dh > 0.0) {
				const double amt = dh > 0.0 ? VU::minf(dh, sediment) : (sediment - capacity) * DEPOSIT;
				sediment -= amt;
				h[i] = h[i] + amt * w00;
				h[i + 1] = h[i + 1] + amt * w10;
				h[i + size] = h[i + size] + amt * w01;
				h[i + size + 1] = h[i + size + 1] + amt * w11;
			} else {
				const double amt = VU::minf((capacity - sediment) * ERODE, -dh);
				sediment += amt;
				h[i] = h[i] - amt * w00;
				h[i + 1] = h[i + 1] - amt * w10;
				h[i + size] = h[i + size] - amt * w01;
				h[i + size + 1] = h[i + size + 1] - amt * w11;
			}
			speed = Math::sqrt(VU::maxf(speed * speed - dh * GRAVITY, 0.0));
			water *= 1.0 - EVAPORATE;
		}
	}
}

// _smooth_pits: each cell above the sea floor a little toward its 3 x 3 mean, and no pit lower
// than its lowest neighbour.
PackedFloat32Array smooth_pits(const PackedFloat32Array &p_h, int size, double p_blend, double p_floor) {
	if (p_blend <= 0.0) {
		return p_h;
	}
	PackedFloat32Array out_p = p_h;
	float *out = out_p.ptrw();
	const float *h = p_h.ptr();
	for_each_row(size, [&](int y) {
		if (y < 1 || y >= size - 1) {
			return;
		}
		const int64_t row = int64_t(y) * size;
		for (int x = 1; x < size - 1; x++) {
			const int64_t i = row + x;
			const double c = h[i];
			if (c < p_floor) {
				continue;
			}
			const double n0 = h[i - size - 1];
			const double n1 = h[i - size];
			const double n2 = h[i - size + 1];
			const double n3 = h[i - 1];
			const double n4 = h[i + 1];
			const double n5 = h[i + size - 1];
			const double n6 = h[i + size];
			const double n7 = h[i + size + 1];
			const double nmin = VU::minf(VU::minf(VU::minf(n0, n1), VU::minf(n2, n3)), VU::minf(VU::minf(n4, n5), VU::minf(n6, n7)));
			const double mean = (n0 + n1 + n2 + n3 + n4 + n5 + n6 + n7 + c) / 9.0;
			double o = c + (mean - c) * p_blend;
			if (c < nmin) {
				o = VU::maxf(o, nmin);
			}
			out[i] = o;
		}
	});
	return out_p;
}

} // namespace

PackedFloat32Array WorldKernels::resample_grid(const PackedFloat32Array &p_src, int p_src_px, int p_dst_px, double p_step) {
	ERR_FAIL_COND_V(p_src_px < 2 || p_dst_px < 1 || p_src.size() != int64_t(p_src_px) * p_src_px, PackedFloat32Array());
	return resample(p_src, p_src_px, p_dst_px, p_step);
}

// world_gen.gd's _erode_and_smooth: droplet erosion and pit smoothing on the REF_SPACING grid.
// p_params: heights, px, ref_px (px when the heights are on that grid already), k (ref cells per
// cell, world_gen._k()), drops, max_steps, seed, smoothing, sea_level. Off the reference grid the
// change erosion makes there is resampled back and added. Returns the heights.
PackedFloat32Array WorldKernels::erode(const Dictionary &p_params) {
	PackedFloat32Array h = p_params["heights"];
	const int px = p_params["px"];
	const int rpx = p_params["ref_px"];
	ERR_FAIL_COND_V(px < 3 || rpx < 3 || h.size() != int64_t(px) * px, PackedFloat32Array());
	const int64_t drops = p_params["drops"];
	const int max_steps = p_params["max_steps"];
	const int64_t seed = p_params["seed"];
	const double blend = p_params["smoothing"];
	const double floor_h = double(p_params["sea_level"]) - 5.0;
	auto erode_and_smooth = [&](PackedFloat32Array &r_h, int p_size) {
		if (drops > 0) {
			erode_grid(r_h.ptrw(), p_size, drops, max_steps, seed);
		}
		r_h = smooth_pits(r_h, p_size, blend, floor_h);
	};
	if (rpx == px) {
		erode_and_smooth(h, px);
		return h;
	}
	const double k = p_params["k"];
	const PackedFloat32Array coarse = resample(h, px, rpx, k);
	PackedFloat32Array eroded = coarse;
	erode_and_smooth(eroded, rpx);
	float *e = eroded.ptrw();
	const float *c = coarse.ptr();
	for (int64_t i = 0; i < eroded.size(); i++) {
		e[i] = double(e[i]) - double(c[i]);
	}
	const PackedFloat32Array delta = resample(eroded, rpx, px, 1.0 / k);
	float *hw = h.ptrw();
	const float *dl = delta.ptr();
	for (int64_t i = 0; i < h.size(); i++) {
		hw[i] = double(hw[i]) + double(dl[i]);
	}
	return h;
}

// world_gen.gd's _compute_slope: rise over run by central differences p_o cells either side
// (clamped at the edges), p_inv = 1 / (2 * p_o * spacing).
PackedFloat32Array WorldKernels::slope(const PackedFloat32Array &p_heights, int p_size, int p_o, double p_inv) {
	ERR_FAIL_COND_V(p_size < 1 || p_o < 1 || p_heights.size() != int64_t(p_size) * p_size, PackedFloat32Array());
	const float *h = p_heights.ptr();
	PackedFloat32Array out_p;
	out_p.resize(p_heights.size());
	float *out = out_p.ptrw();
	for_each_row(p_size, [&](int y) {
		const int64_t y0 = int64_t(MAX(y - p_o, 0)) * p_size;
		const int64_t y1 = int64_t(MIN(y + p_o, p_size - 1)) * p_size;
		const int64_t row = int64_t(y) * p_size;
		for (int x = 0; x < p_size; x++) {
			const int x0 = MAX(x - p_o, 0);
			const int x1 = MIN(x + p_o, p_size - 1);
			const double gx = (double(h[row + x1]) - double(h[row + x0])) * p_inv;
			const double gz = (double(h[y1 + x]) - double(h[y0 + x])) * p_inv;
			out[row + x] = Math::sqrt(gx * gx + gz * gz);
		}
	});
	return out_p;
}

// world_gen.gd's _floor_up: fine cell x reads coarse cell floor(x / p_k), the one it lies in.
PackedFloat32Array WorldKernels::floor_up(const PackedFloat32Array &p_src, int p_src_px, int p_dst_px, double p_k) {
	ERR_FAIL_COND_V(p_src_px < 1 || p_dst_px < 1 || p_src.size() != int64_t(p_src_px) * p_src_px, PackedFloat32Array());
	const float *src = p_src.ptr();
	LocalVector<int> idx;
	idx.resize(p_dst_px);
	for (int x = 0; x < p_dst_px; x++) {
		idx[x] = MIN(int(x / p_k), p_src_px - 1);
	}
	PackedFloat32Array out_p;
	out_p.resize(int64_t(p_dst_px) * p_dst_px);
	float *out = out_p.ptrw();
	for_each_row(p_dst_px, [&](int y) {
		const int64_t row = int64_t(idx[y]) * p_src_px;
		float *o = out + int64_t(y) * p_dst_px;
		for (int x = 0; x < p_dst_px; x++) {
			o[x] = src[row + idx[x]];
		}
	});
	return out_p;
}

// world_gen.gd's _build_control: each ground texture's weight per cell from the biome weights,
// blurred, then gravel under water and the road over everything; the two strongest textures
// (the third sometimes, dithered) and their blend packed for Terrain3D. p_params: size, heights,
// slope, moisture, pine, woodland, scrub, coast, alpine (biome weights), water_level, road_mask,
// road_weight, sea_level, snow_line, blur_px, tex (world_gen's Tex enum). Returns {control, biome}.
Dictionary WorldKernels::control_map(const Dictionary &p_params) {
	const int size = p_params["size"];
	const int64_t n = int64_t(size) * size;
	const PackedFloat32Array hp = p_params["heights"];
	const PackedFloat32Array slope_p = p_params["slope"];
	const PackedFloat32Array moist_p = p_params["moisture"];
	const PackedFloat32Array pine_p = p_params["pine"];
	const PackedFloat32Array wood_p = p_params["woodland"];
	const PackedFloat32Array scrub_p = p_params["scrub"];
	const PackedFloat32Array coast_p = p_params["coast"];
	const PackedFloat32Array alpine_p = p_params["alpine"];
	const PackedFloat32Array wl_p = p_params["water_level"];
	const PackedByteArray mask_p = p_params["road_mask"];
	const PackedFloat32Array rw_p = p_params["road_weight"];
	ERR_FAIL_COND_V(size < 1 || hp.size() != n || slope_p.size() != n || moist_p.size() != n || pine_p.size() != n || wood_p.size() != n || scrub_p.size() != n || coast_p.size() != n || alpine_p.size() != n || wl_p.size() != n || mask_p.size() != n || rw_p.size() != n, Dictionary());
	const double sea = p_params["sea_level"];
	const double snow = p_params["snow_line"];
	const int blur_px = p_params["blur_px"];
	const Dictionary tex = p_params["tex"];
	const int ntex = tex.size();
	ERR_FAIL_COND_V(ntex < 1 || ntex > 32, Dictionary());
	const int GRASS = tex["GRASS"];
	const int CLIFF = tex["CLIFF"];
	const int SILT = tex["SILT"];
	const int DIRT = tex["DIRT"];
	const int SNOW = tex["SNOW"];
	const int MEADOW = tex["MEADOW"];
	const int FOREST = tex["FOREST"];
	const int GRAVEL = tex["GRAVEL"];
	const int ROAD = tex["ROAD"];
	const int SEABED = tex["SEABED"];
	const float *h = hp.ptr();
	const PackedFloat32Array slope_b = blur_field(slope_p, size, blur_px);
	const float *slope = slope_b.ptr();
	const float *moist = moist_p.ptr();
	const float *pine = pine_p.ptr();
	const float *wood_w = wood_p.ptr();
	const float *scrub = scrub_p.ptr();
	const float *coast = coast_p.ptr();
	const float *alpine = alpine_p.ptr();

	LocalVector<PackedFloat32Array> fields;
	fields.resize(ntex);
	float *fp[32];
	for (int k = 0; k < ntex; k++) {
		fields[k].resize_initialized(n);
		fp[k] = fields[k].ptrw();
	}
	for_each_row(size, [&](int y) {
		int64_t i = int64_t(y) * size;
		for (int x = 0; x < size; x++, i++) {
			const double wood = wood_w[i];
			const double damp = VU::smoothstep(0.35, 0.55, moist[i]);
			fp[FOREST][i] = pine[i];
			fp[GRASS][i] = wood * damp;
			fp[MEADOW][i] = wood * (1.0 - damp);
			fp[DIRT][i] = scrub[i];
			// The coast is dry sand above the waterline and stony sea floor below it.
			const double under = 1.0 - VU::smoothstep(sea - 0.4, sea + 0.2, h[i]);
			fp[SILT][i] = coast[i] * (1.0 - under);
			fp[SEABED][i] = coast[i] * under;
			fp[SNOW][i] = alpine[i];
			fp[CLIFF][i] = VU::smoothstep(0.7, 1.6, slope[i]) * 1.5;
		}
	});
	for_each_row(ntex, [&](int k) {
		fields[k] = blur_field(fields[k], size, blur_px);
	});
	const float *f[32];
	for (int k = 0; k < ntex; k++) {
		f[k] = fields[k].ptr();
	}
	const float *wl = wl_p.ptr();
	const uint8_t *mask = mask_p.ptr();
	const float *road_w = rw_p.ptr();
	PackedInt32Array control_p;
	control_p.resize(n);
	int32_t *control = control_p.ptrw();
	PackedByteArray biome_p;
	biome_p.resize(n);
	uint8_t *biome = biome_p.ptrw();
	for_each_row(size, [&](int y) {
		float w[32];
		int64_t i = int64_t(y) * size;
		for (int x = 0; x < size; x++, i++) {
			for (int k = 0; k < ntex; k++) {
				w[k] = f[k][i];
			}
			if (wl[i] > -1e8) {
				const double depth = double(wl[i]) - double(h[i]);
				const double w_gravel = 0.6 + 1.4 * VU::smoothstep(0.2, 1.5, depth);
				w[GRAVEL] = w_gravel * (1.0 - VU::smoothstep(snow - 25.0, snow + 25.0, h[i]));
			}
			if (mask[i] == 1 || mask[i] == 2) {
				const double rw = road_w[i];
				for (int k = 0; k < ntex; k++) {
					w[k] = double(w[k]) * (1.0 - rw);
				}
				w[ROAD] = rw;
			}
			int best = 0;
			int second = -1;
			int third = -1;
			for (int k = 1; k < ntex; k++) {
				if (w[k] > w[best]) {
					third = second;
					second = best;
					best = k;
				} else if (second < 0 || w[k] > w[second]) {
					third = second;
					second = k;
				} else if (third < 0 || w[k] > w[third]) {
					third = k;
				}
			}
			if (third >= 0 && w[third] > 0.02) {
				int64_t hh = (int64_t(i % size) * 374761393 + int64_t(i / size) * 668265263) & 0xFFFFFFFF;
				hh = ((hh ^ (hh >> 13)) * 1274126177) & 0xFFFFFFFF;
				const double hsh = double((hh ^ (hh >> 16)) & 0xFFFF) / 65535.0;
				if (hsh < double(w[third]) / (double(w[second]) + double(w[third]))) {
					second = third;
				}
			}
			double blend = 0.0;
			if (second >= 0 && double(w[best]) + double(w[second]) > 0.0) {
				blend = double(w[second]) / (double(w[best]) + double(w[second]));
			} else {
				second = best;
			}
			biome[i] = best;
			control[i] = int32_t((int64_t(best) << 27) | (int64_t(second) << 22) | (int64_t(blend * 255.0) << 14));
		}
	});
	Dictionary out;
	out["control"] = control_p;
	out["biome"] = biome_p;
	return out;
}

// The road and barn kernels below are road_builder.gd's coarse grid, A* and rasterizer and
// barn_site.gd's search, ported line for line (Vector2/Vector3 in floats, the rest in doubles).

// RoadBuilder._build_coarse and _build_shore_distance. p_params: heights, water_level, px, coarse
// (A* cell side in cells), sub (cells per sample), spacing, sea_level. Returns {ch, cwet, csea,
// cslope, cshore}: mean height, any water, below the sea, height range per metre, and for wet or
// sea cells the Chebyshev distance in coarse cells to the nearest dry one.
Dictionary WorldKernels::road_grid(const Dictionary &p_params) {
	const PackedFloat32Array hp = p_params["heights"];
	const PackedFloat32Array wp = p_params["water_level"];
	const int px = p_params["px"];
	const int coarse = p_params["coarse"];
	const int sub = p_params["sub"];
	const double spacing = p_params["spacing"];
	const double sea = p_params["sea_level"];
	ERR_FAIL_COND_V(px < 1 || coarse < 1 || sub < 1 || hp.size() != int64_t(px) * px || wp.size() != hp.size(), Dictionary());
	const float *h = hp.ptr();
	const float *wl = wp.ptr();
	const int cpx = px / coarse;
	const int64_t cn = int64_t(cpx) * cpx;
	// Each coarse cell reads the heightmap every sample (4 x 4 points at any spacing).
	const int taps = int(Math::ceil(double(coarse) / sub));
	const int count = taps * taps;
	PackedFloat32Array ch_p;
	ch_p.resize(cn);
	PackedByteArray wet_p;
	wet_p.resize(cn);
	PackedByteArray sea_p;
	sea_p.resize(cn);
	PackedFloat32Array slope_p;
	slope_p.resize(cn);
	float *ch = ch_p.ptrw();
	uint8_t *cwet = wet_p.ptrw();
	uint8_t *csea = sea_p.ptrw();
	float *cslope = slope_p.ptrw();
	for_each_row(cpx, [&](int cy) {
		for (int cx = 0; cx < cpx; cx++) {
			const int64_t ci = int64_t(cy) * cpx + cx;
			double sum = 0.0;
			double hmin = Math::INF;
			double hmax = -Math::INF;
			uint8_t wet = 0;
			for (int ty = 0; ty < taps; ty++) {
				const int dy = ty * sub;
				for (int tx = 0; tx < taps; tx++) {
					const int dx = tx * sub;
					const int64_t i = int64_t(cy * coarse + dy) * px + cx * coarse + dx;
					const double hh = h[i];
					sum += hh;
					hmin = VU::minf(hmin, hh);
					hmax = VU::maxf(hmax, hh);
					if (wl[i] > hh) {
						wet = 1;
					}
				}
			}
			ch[ci] = sum / count;
			cwet[ci] = wet;
			csea[ci] = ch[ci] < sea ? 1 : 0;
			cslope[ci] = (hmax - hmin) / (coarse * spacing);
		}
	});
	PackedFloat32Array shore_p;
	shore_p.resize(cn);
	shore_p.fill(Math::INF);
	float *shore = shore_p.ptrw();
	LocalVector<int32_t> queue;
	for (int64_t ci = 0; ci < cn; ci++) {
		if (!cwet[ci] && !csea[ci]) {
			shore[ci] = 0.0f;
			queue.push_back(ci);
		}
	}
	for (uint32_t head = 0; head < queue.size(); head++) {
		const int32_t c = queue[head];
		const int cx = c % cpx;
		const int cy = c / cpx;
		for (int dy = -1; dy <= 1; dy++) {
			for (int dx = -1; dx <= 1; dx++) {
				const int nx = cx + dx;
				const int ny = cy + dy;
				if (nx < 0 || ny < 0 || nx >= cpx || ny >= cpx) {
					continue;
				}
				const int32_t nb = ny * cpx + nx;
				if (shore[nb] == Math::INF) {
					shore[nb] = double(shore[c]) + 1.0;
					queue.push_back(nb);
				}
			}
		}
	}
	Dictionary out;
	out["ch"] = ch_p;
	out["cwet"] = wet_p;
	out["csea"] = sea_p;
	out["cslope"] = slope_p;
	out["cshore"] = shore_p;
	return out;
}

// RoadBuilder._astar and _step_cost: the cheapest 8-connected path over the coarse grid from
// start to any cell in goal_set. p_params: grid (road_grid's), cpx, coarse, spacing, max_grade,
// shore_cost, start, goal_set, use_heuristic, goal_c (the goal's coarse cell). Returns the
// path's coarse cells, start first; empty when there is none.
PackedInt32Array WorldKernels::road_astar(const Dictionary &p_params) {
	const Dictionary grid = p_params["grid"];
	const PackedFloat32Array ch_p = grid["ch"];
	const PackedByteArray wet_p = grid["cwet"];
	const PackedByteArray sea_p = grid["csea"];
	const PackedFloat32Array slope_p = grid["cslope"];
	const PackedFloat32Array shore_p = grid["cshore"];
	const PackedByteArray goal_p = p_params["goal_set"];
	const int cpx = p_params["cpx"];
	const int64_t cn = int64_t(cpx) * cpx;
	const int start = p_params["start"];
	ERR_FAIL_COND_V(cpx < 1 || ch_p.size() != cn || wet_p.size() != cn || sea_p.size() != cn || slope_p.size() != cn || shore_p.size() != cn || goal_p.size() != cn || start < 0 || start >= cn, PackedInt32Array());
	const int coarse = p_params["coarse"];
	const double spacing = p_params["spacing"];
	const double max_grade = p_params["max_grade"];
	const double shore_cost = p_params["shore_cost"];
	const bool use_heuristic = p_params["use_heuristic"];
	const Vector2 goal_c = p_params["goal_c"];
	const float *ch = ch_p.ptr();
	const uint8_t *cwet = wet_p.ptr();
	const uint8_t *csea = sea_p.ptr();
	const float *cslope = slope_p.ptr();
	const float *cshore = shore_p.ptr();
	const uint8_t *goal_set = goal_p.ptr();
	auto step_cost = [&](int32_t p_from, int32_t p_to, double p_dist) {
		const double dh = VU::absf(double(ch[p_to]) - double(ch[p_from]));
		const double grade = dh / (p_dist * coarse * spacing);
		double c = p_dist * (1.0 + 12.0 * grade + 6.0 * cslope[p_to]);
		if (grade > max_grade) {
			c += p_dist * 40.0 * (grade / max_grade); // steeper than the road can climb
		}
		if (cslope[p_to] > 1.0) {
			c += p_dist * 30.0; // cliff face
		}
		if (csea[p_to]) {
			c += p_dist * 15.0; // causeway
		} else if (cwet[p_to]) {
			c += p_dist * 10.0; // bridge
		}
		if (cshore[p_to] > 0.0) {
			c += p_dist * shore_cost * cshore[p_to];
		}
		return c;
	};
	LocalVector<float> g;
	g.resize(cn);
	LocalVector<int32_t> came;
	came.resize(cn);
	for (int64_t i = 0; i < cn; i++) {
		g[i] = Math::INF;
		came[i] = -1;
	}
	LocalVector<uint8_t> closed;
	closed.resize_initialized(cn);
	const int ddxs[8] = { -1, 0, 1, -1, 1, -1, 0, 1 };
	const int ddys[8] = { -1, -1, -1, 0, 0, 1, 1, 1 };
	const float diag = 1.41421356;
	const float dists[8] = { diag, 1, diag, 1, 1, diag, 1, diag };
	MinHeap heap;
	g[start] = 0.0f;
	heap.push(0.0, start);
	int32_t found = -1;
	while (heap.n > 0) {
		const int32_t c = heap.pop();
		if (closed[c]) {
			continue;
		}
		closed[c] = 1;
		if (goal_set[c]) {
			found = c;
			break;
		}
		const int cx = c % cpx;
		const int cy = c / cpx;
		for (int d = 0; d < 8; d++) {
			const int nx = cx + ddxs[d];
			const int ny = cy + ddys[d];
			if (nx < 0 || ny < 0 || nx >= cpx || ny >= cpx) {
				continue;
			}
			const int32_t nb = ny * cpx + nx;
			if (closed[nb]) {
				continue;
			}
			const double ng = double(g[c]) + step_cost(c, nb, dists[d]);
			if (ng < g[nb]) {
				g[nb] = ng;
				came[nb] = c;
				double f = ng;
				if (use_heuristic) {
					f += Vector2(nx, ny).distance_to(goal_c);
				}
				heap.push(f, nb);
			}
		}
	}
	PackedInt32Array path;
	for (int32_t cur = found; cur >= 0; cur = came[cur]) {
		path.push_back(cur);
	}
	path.reverse();
	return path;
}

// RoadBuilder._rasterize: flattens the ground under the road's ground samples toward the profile,
// blends the shoulder and fills the road mask (0 none, 1 surface, 2 shoulder, 3 bridge footprint)
// and texture weight. p_params: heights, road_mask and road_weight (written in place: the script's
// arrays, as GDScript's own writes to a packed array it was passed), water_level, px, poly (samples
// in cells), prof (deck heights), bridges (sample ranges), width, spacing, edge_m, shoulder_m,
// sample_px, deck_underside. Returns the surface cells, in index order.
PackedInt32Array WorldKernels::road_rasterize(const Dictionary &p_params) {
	Variant hv = p_params["heights"];
	Variant mv = p_params["road_mask"];
	Variant rwv = p_params["road_weight"];
	ERR_FAIL_COND_V(hv.get_type() != Variant::PACKED_FLOAT32_ARRAY || mv.get_type() != Variant::PACKED_BYTE_ARRAY || rwv.get_type() != Variant::PACKED_FLOAT32_ARRAY, PackedInt32Array());
	PackedFloat32Array *hp = VariantInternal::get_float32_array(&hv);
	PackedByteArray *mp = VariantInternal::get_byte_array(&mv);
	PackedFloat32Array *rwp = VariantInternal::get_float32_array(&rwv);
	const PackedFloat32Array wl_p = p_params["water_level"];
	const int px = p_params["px"];
	const int64_t n = int64_t(px) * px;
	const PackedVector2Array poly = p_params["poly"];
	const PackedFloat32Array prof = p_params["prof"];
	ERR_FAIL_COND_V(px < 1 || hp->size() != n || mp->size() != n || rwp->size() != n || wl_p.size() != n || prof.size() != poly.size(), PackedInt32Array());
	LocalVector<Vector2i> bridges;
	for (const Variant &b : Array(p_params["bridges"])) {
		bridges.push_back(b);
	}
	const double spacing = p_params["spacing"];
	const double half_w = double(p_params["width"]) * 0.5 / spacing; // in cells
	const double edge = double(p_params["edge_m"]) / spacing;
	const double reach = half_w + double(p_params["shoulder_m"]) / spacing;
	const double sample_px = p_params["sample_px"];
	const double deck_underside = p_params["deck_underside"];
	const int r = int(Math::ceil(reach));
	float *h = hp->ptrw();
	uint8_t *mask = mp->ptrw();
	float *road_w = rwp->ptrw();
	const float *wl = wl_p.ptr();
	PackedInt32Array surface;

	// Per-cell scratch over the road's bounding box only; outside it every weight is 0.
	int bx0 = px;
	int by0 = px;
	int bx1 = -1;
	int by1 = -1;
	for (const Vector2 &p : poly) {
		const int cx = int(Math::round(double(p.x)));
		const int cy = int(Math::round(double(p.y)));
		bx0 = MIN(bx0, MAX(cx - r, 0));
		by0 = MIN(by0, MAX(cy - r, 0));
		bx1 = MAX(bx1, MIN(cx + r, px - 1));
		by1 = MAX(by1, MIN(cy + r, px - 1));
	}
	if (bx1 < bx0 || by1 < by0) {
		return surface;
	}
	const int bw = bx1 - bx0 + 1;
	const int64_t bn = int64_t(bw) * (by1 - by0 + 1);
	LocalVector<float> w;
	w.resize_initialized(bn);
	LocalVector<float> target;
	target.resize_initialized(bn);
	// A surface cell is reached by two or three samples; the nearest one sets its height.
	LocalVector<float> near;
	// Under the deck: cells nearer a bridge sample than a ground one may be flattened by an
	// abutment pad, but never above the deck's underside.
	LocalVector<float> deck_near;
	LocalVector<float> deck_h;
	// Within a sample of an abutment the pad is built into the bank: filled to the deck even
	// where wet, and never capped under the deck.
	LocalVector<float> abut_d;
	near.resize(bn);
	deck_near.resize(bn);
	deck_h.resize(bn);
	abut_d.resize(bn);
	for (int64_t i = 0; i < bn; i++) {
		near[i] = deck_near[i] = deck_h[i] = abut_d[i] = Math::INF;
	}

	const float *pr = prof.ptr();
	for (int s = 0; s < poly.size(); s++) {
		bool bridge = false;
		bool abutment = false;
		for (const Vector2i &b : bridges) {
			bridge = bridge || (s > b.x && s < b.y);
			abutment = abutment || s == b.x || s == b.y;
		}
		const Vector2 p = poly[s];
		const int cx = int(Math::round(double(p.x)));
		const int cy = int(Math::round(double(p.y)));
		for (int dy = -r; dy <= r; dy++) {
			const int y = cy + dy;
			if (y < 0 || y >= px) {
				continue;
			}
			for (int dx = -r; dx <= r; dx++) {
				const int x = cx + dx;
				if (x < 0 || x >= px) {
					continue;
				}
				const double d = Vector2(x, y).distance_to(p);
				if (d > reach) {
					continue;
				}
				const int64_t i = int64_t(y) * px + x;
				const int64_t li = int64_t(y - by0) * bw + (x - bx0);
				if (bridge) {
					if (d <= half_w) {
						if (mask[i] == 0) {
							mask[i] = 3;
						}
						if (d < deck_near[li]) {
							deck_near[li] = d;
							deck_h[li] = pr[s];
						}
					}
					continue;
				}
				if (abutment && d < abut_d[li]) {
					abut_d[li] = d;
				}
				double wt = d <= half_w ? 1.0 : 1.0 - (d - half_w) / (reach - half_w);
				wt = wt * wt * (3.0 - 2.0 * wt);
				if (wt > w[li] || (wt >= 0.999 && d < near[li])) {
					w[li] = wt;
					target[li] = pr[s];
					near[li] = d;
				}
			}
		}
	}

	for (int y = by0; y <= by1; y++) {
		for (int x = bx0; x <= bx1; x++) {
			const int64_t li = int64_t(y - by0) * bw + (x - bx0);
			if (w[li] <= 0.0f) {
				continue;
			}
			const int64_t i = int64_t(y) * px + x;
			const bool pad = abut_d[li] <= sample_px;
			if (wl[i] > h[i] && !pad) {
				continue; // never fill a river or lake
			}
			double t = target[li];
			if (deck_near[li] < near[li] && !pad) {
				t = VU::minf(t, double(deck_h[li]) - deck_underside);
			}
			h[i] = VU::lerpf(h[i], t, w[li]);
			road_w[i] = VU::maxf(road_w[i], 1.0 - VU::smoothstep(half_w - edge, half_w + edge, near[li]));
			if (w[li] >= 0.999) {
				mask[i] = 1;
				surface.push_back(i);
			} else if (mask[i] == 0) {
				mask[i] = 2;
			}
		}
	}
	return surface;
}

namespace {

// barn_site.gd's find and _evaluate: the best spot beside a straight, gentle stretch of road.
struct BarnSearch {
	const float *h = nullptr;
	const float *water = nullptr;
	const uint8_t *mask = nullptr;
	int px = 0;
	double spacing = 1.0;
	Vector2 origin;
	double sea = 0.0;
	double foot_w = 0.0;
	double foot_d = 0.0;
	double margin = 0.0;
	double apron = 0.0;
	double blend_m = 0.0;
	double edge_keep_m = 0.0;
	double water_keep_m = 0.0;
	double min_above_sea = 0.0;
	double max_cut = 0.0;
	double shoulder_m = 0.0;

	static double rect_dist(double a, double o, double aw, double o0, double o1) {
		const double da = VU::maxf(VU::absf(a) - aw, 0.0);
		const double dy = VU::maxf(VU::maxf(o0 - o, o - o1), 0.0);
		return Math::sqrt(da * da + dy * dy);
	}

	// _cell_box, as x0, y0, x1, y1.
	void cell_box(const Vector3 &p, const Vector3 &d, const Vector3 &n, double a_reach, double o_lo, double o_hi, int r_box[4]) const {
		Vector2 lo(Math::INF, Math::INF);
		Vector2 hi(-Math::INF, -Math::INF);
		const Vector2 corners[4] = { Vector2(-a_reach, o_lo), Vector2(a_reach, o_lo), Vector2(-a_reach, o_hi), Vector2(a_reach, o_hi) };
		for (const Vector2 &c : corners) {
			const Vector3 w = p + d * c.x + n * c.y;
			lo = lo.min(Vector2(w.x, w.z));
			hi = hi.max(Vector2(w.x, w.z));
		}
		r_box[0] = CLAMP(int(Math::floor((double(lo.x) - double(origin.x)) / spacing)), 1, px - 2);
		r_box[1] = CLAMP(int(Math::floor((double(lo.y) - double(origin.y)) / spacing)), 1, px - 2);
		r_box[2] = CLAMP(int(Math::ceil((double(hi.x) - double(origin.x)) / spacing)), 1, px - 2);
		r_box[3] = CLAMP(int(Math::ceil((double(hi.y) - double(origin.y)) / spacing)), 1, px - 2);
	}

	double evaluate(const Vector3 &p, const Vector3 &d, const Vector3 &n, double half_w, double tgt) const {
		const double aw = foot_w * 0.5 + margin;
		const double o0 = half_w;
		const double o1 = half_w + apron + foot_d + margin;
		const double reach = blend_m + water_keep_m;
		int box[4];
		cell_box(p, d, n, aw + reach, o0 - reach, o1 + reach, box);
		const int edge = int(edge_keep_m / spacing);
		if (box[0] < edge || box[1] < edge || box[2] >= px - edge || box[3] >= px - edge) {
			return Math::INF;
		}
		const double road_band = o0 + shoulder_m + 1.0;
		double sum = 0.0;
		int count = 0;
		double max_pad = 0.0;
		double max_blend = 0.0;
		for (int y = box[1]; y <= box[3]; y++) {
			for (int x = box[0]; x <= box[2]; x++) {
				const int64_t i = int64_t(y) * px + x;
				const Vector3 rel(double(origin.x) + x * spacing - double(p.x), 0.0, double(origin.y) + y * spacing - double(p.z));
				const double a = rel.dot(d);
				const double o = rel.dot(n);
				const double dist = rect_dist(a, o, aw, o0, o1);
				if (dist > reach) {
					continue;
				}
				if (water[i] > -1e8 || h[i] < sea + 0.5) {
					return Math::INF;
				}
				if (dist > blend_m || o < o0) {
					continue;
				}
				const uint8_t m = mask[i];
				if (m == 3 || (m != 0 && o > road_band)) {
					return Math::INF;
				}
				if (m == 1) {
					continue;
				}
				const double dh = VU::absf(double(h[i]) - tgt);
				if (dist == 0.0) {
					if (h[i] < sea + min_above_sea) {
						return Math::INF;
					}
					sum += dh;
					count++;
					max_pad = VU::maxf(max_pad, dh);
				} else {
					max_blend = VU::maxf(max_blend, dh);
				}
			}
		}
		if (count == 0 || max_pad > max_cut) {
			return Math::INF;
		}
		return sum / count + 0.3 * max_pad + 0.15 * max_blend;
	}
};

Vector3 flat(Vector3 v) {
	v.y = 0.0;
	return v.length_squared() > 1e-8 ? v.normalized() : Vector3();
}

} // namespace

// barn_site.gd's find over p_params roads, from the site's own fields and constants (site: a
// barn_site.gd, score: the best so far, shoulder_m and sample_m: RoadBuilder's). Returns {found}
// and, when a spot beats score, score, road (index), s (sample), out_dir, along, half_w, grade.
Dictionary WorldKernels::barn_find(const Dictionary &p_params) {
	Object *site = p_params["site"];
	ERR_FAIL_NULL_V(site, Dictionary());
	const PackedFloat32Array hp = site->get("heights");
	const PackedFloat32Array wp = site->get("water");
	const PackedByteArray mp = site->get("mask");
	BarnSearch bs;
	bs.px = site->get("_px");
	ERR_FAIL_COND_V(bs.px < 3 || hp.size() != int64_t(bs.px) * bs.px || wp.size() != hp.size() || mp.size() != hp.size(), Dictionary());
	bs.h = hp.ptr();
	bs.water = wp.ptr();
	bs.mask = mp.ptr();
	bs.spacing = site->get("_spacing");
	bs.origin = site->get("_origin");
	bs.sea = site->get("_sea");
	bs.foot_w = site->get("FOOT_W");
	bs.foot_d = site->get("FOOT_D");
	bs.margin = site->get("MARGIN");
	bs.apron = site->get("APRON");
	bs.blend_m = site->get("BLEND_M");
	bs.edge_keep_m = site->get("EDGE_KEEP_M");
	bs.water_keep_m = site->get("WATER_KEEP_M");
	bs.min_above_sea = site->get("MIN_ABOVE_SEA");
	bs.max_cut = site->get("MAX_CUT");
	bs.shoulder_m = p_params["shoulder_m"];
	const double step_m = site->get("STEP_M");
	const double max_grade_step = site->get("MAX_GRADE_STEP");
	const double straight_dot = site->get("STRAIGHT_DOT");
	const int bridge_keep = site->get("BRIDGE_KEEP");
	const double sample_m = p_params["sample_m"];
	double best = p_params["score"];

	Dictionary out;
	out["found"] = false;
	const double aw = bs.foot_w * 0.5 + bs.margin;
	// Road points are RoadBuilder.SAMPLE_M apart whatever the heightmap spacing.
	const int reach = int(Math::ceil(aw / sample_m)) + 1;
	const int stride = MAX(1, int(Math::round(step_m / sample_m)));
	const real_t sides[2] = { -1.0, 1.0 };
	const Array roads = p_params["roads"];
	for (int ri = 0; ri < roads.size(); ri++) {
		const Dictionary road = roads[ri];
		const PackedVector3Array pts = road["points"];
		const double half_w = double(road["width"]) * 0.5;
		LocalVector<Vector2i> bridges;
		for (const Variant &b : Array(road["bridges"])) {
			bridges.push_back(b);
		}
		for (int s = reach; s < pts.size() - reach; s += stride) {
			bool near_bridge = false;
			for (const Vector2i &b : bridges) {
				near_bridge = near_bridge || (s >= b.x - bridge_keep && s <= b.y + bridge_keep);
			}
			if (near_bridge) {
				continue;
			}
			const Vector3 d = flat(pts[s + 1] - pts[s - 1]);
			if (d == Vector3() || flat(pts[s] - pts[s - reach]).dot(flat(pts[s + reach] - pts[s])) < straight_dot) {
				continue;
			}
			const double grade = VU::absf(double(pts[s + reach].y) - double(pts[s - reach].y));
			if (grade > max_grade_step) {
				continue;
			}
			for (const real_t side : sides) {
				const Vector3 n = Vector3(-d.z, 0.0, d.x) * side;
				const double sc = bs.evaluate(pts[s], d, n, half_w, pts[s].y) + grade * 3.0;
				if (sc < best) {
					best = sc;
					out["found"] = true;
					out["score"] = sc;
					out["road"] = ri;
					out["s"] = s;
					out["out_dir"] = n;
					out["along"] = d;
					out["half_w"] = half_w;
					out["grade"] = grade;
				}
			}
		}
	}
	return out;
}

// The foliage kernel below is foliage_placer.gd's run, ported line for line: one random sequence
// drawn candidate after candidate and instance after instance, as the script drew it, so every
// plant lands where the script put it. The rules' shares are expressions (forest_rules.gd), read
// by a small evaluator that does GDScript's double math in GDScript's order.

namespace {

// The fields a rule expression may name, one per FoliagePlacer cell variable.
enum CellField {
	CF_H,
	CF_SLOPE,
	CF_ROCKY,
	CF_BIOME,
	CF_WET,
	CF_PINE,
	CF_WOOD,
	CF_SCRUB,
	CF_D,
	CF_SP,
	CF_PATCH,
	CF_WET_WIDE,
	CF_COVER,
	CF_WATER_DIST,
	CF_DEPTH,
	CF_CANOPY,
	CF_EDGE,
	CF_OPEN,
	CF_SEA,
	CF_SNOW,
	CF_COUNT,
};

const char *const CELL_FIELD_NAMES[CF_COUNT] = { "h", "slope", "rocky", "biome", "wet", "pine", "wood", "scrub", "d", "sp",
	"patch", "wet_wide", "cover", "water_dist", "depth", "canopy", "edge", "open", "sea", "snow" };

// One candidate: its fields, the noises filled in when an expression first asks (they draw no
// random numbers, so asking late changes nothing), and the named expressions worked out so far.
struct FoliageCell {
	double f[CF_COUNT] = {};
	double nx = 0.0;
	double nz = 0.0;
	const FastNoiseLite *noise[3] = {}; // d, sp, patch
	uint32_t noise_done = 0;
	LocalVector<double> def_val;
	LocalVector<uint32_t> def_stamp;
	uint32_t stamp = 1;

	double field(int p_f) {
		if (p_f >= CF_D && p_f <= CF_PATCH) {
			const uint32_t bit = 1u << (p_f - CF_D);
			if (!(noise_done & bit)) {
				noise_done |= bit;
				f[p_f] = noise[p_f - CF_D]->get_noise_2d(nx, nz) * 0.5 + 0.5;
			}
		}
		return f[p_f];
	}
};

// Rule expressions: numbers, cell fields, named expressions (defs), + - * /, comparisons, and, or,
// not, `a if cond else b`, and smoothstep, lerpf, clampf, minf, maxf, absf, sqrt, pow, with
// GDScript's precedence and left-to-right order.
struct RuleExpr {
	enum Op : uint8_t {
		CONST,
		FIELD,
		DEF,
		NEG,
		NOT,
		ADD,
		SUB,
		MUL,
		DIV,
		LT,
		LE,
		GT,
		GE,
		EQ,
		NE,
		AND,
		OR,
		SELECT,
		SMOOTHSTEP,
		LERPF,
		CLAMPF,
		MINF,
		MAXF,
		ABSF,
		SQRT,
		POW,
	};
	struct Node {
		Op op = CONST;
		int32_t a = -1;
		int32_t b = -1;
		int32_t c = -1;
		double v = 0.0;
	};
	LocalVector<Node> nodes;
	LocalVector<int32_t> def_root; // -1 until compiled
	HashMap<String, int> def_index;
	LocalVector<Variant> def_src;
	String error;

	// Parser state.
	String src;
	int pos = 0;
	String tok;
	int tok_kind = 0; // 0 end, 1 number, 2 name, 3 symbol

	explicit RuleExpr(const Dictionary &p_defs) {
		for (const KeyValue<Variant, Variant> &kv : p_defs) {
			def_index[kv.key] = def_src.size();
			def_src.push_back(kv.value);
			def_root.push_back(-1);
		}
	}

	int add(Op p_op, int p_a = -1, int p_b = -1, int p_c = -1, double p_v = 0.0) {
		Node n;
		n.op = p_op;
		n.a = p_a;
		n.b = p_b;
		n.c = p_c;
		n.v = p_v;
		nodes.push_back(n);
		return nodes.size() - 1;
	}

	void next() {
		while (pos < src.length() && (src[pos] == ' ' || src[pos] == '\t' || src[pos] == '\n')) {
			pos++;
		}
		tok = "";
		if (pos >= src.length()) {
			tok_kind = 0;
			return;
		}
		const char32_t ch = src[pos];
		const int start = pos;
		if (is_digit(ch) || (ch == '.' && pos + 1 < src.length() && is_digit(src[pos + 1]))) {
			while (pos < src.length() && (is_digit(src[pos]) || src[pos] == '.')) {
				pos++;
			}
			if (pos < src.length() && (src[pos] == 'e' || src[pos] == 'E')) {
				pos++;
				if (pos < src.length() && (src[pos] == '+' || src[pos] == '-')) {
					pos++;
				}
				while (pos < src.length() && is_digit(src[pos])) {
					pos++;
				}
			}
			tok_kind = 1;
		} else if (is_ascii_identifier_char(ch)) {
			while (pos < src.length() && is_ascii_identifier_char(src[pos])) {
				pos++;
			}
			tok_kind = 2;
		} else {
			pos++;
			if (pos < src.length() && src[pos] == '=' && (ch == '<' || ch == '>' || ch == '=' || ch == '!')) {
				pos++;
			}
			tok_kind = 3;
		}
		tok = src.substr(start, pos - start);
	}

	bool fail(const String &p_why) {
		if (error.is_empty()) {
			error = vformat("%s at '%s' in \"%s\"", p_why, tok, src);
		}
		return false;
	}

	bool accept(const char *p_s) {
		if (tok_kind >= 2 && tok == p_s) {
			next();
			return true;
		}
		return false;
	}

	int parse_ternary() {
		const int t = parse_or();
		if (t >= 0 && accept("if")) {
			const int cond = parse_or();
			if (!accept("else")) {
				fail("expected 'else'");
				return -1;
			}
			const int e = parse_ternary();
			return (cond < 0 || e < 0) ? -1 : add(SELECT, cond, t, e);
		}
		return t;
	}

	int parse_or() {
		int l = parse_and();
		while (l >= 0 && accept("or")) {
			const int r = parse_and();
			l = r < 0 ? -1 : add(OR, l, r);
		}
		return l;
	}

	int parse_and() {
		int l = parse_not();
		while (l >= 0 && accept("and")) {
			const int r = parse_not();
			l = r < 0 ? -1 : add(AND, l, r);
		}
		return l;
	}

	int parse_not() {
		if (accept("not")) {
			const int a = parse_not();
			return a < 0 ? -1 : add(NOT, a);
		}
		return parse_cmp();
	}

	int parse_cmp() {
		const int l = parse_add();
		static const char *ops[] = { "<", "<=", ">", ">=", "==", "!=" };
		static const Op codes[] = { LT, LE, GT, GE, EQ, NE };
		for (int k = 0; l >= 0 && k < 6; k++) {
			if (accept(ops[k])) {
				const int r = parse_add();
				return r < 0 ? -1 : add(codes[k], l, r);
			}
		}
		return l;
	}

	int parse_add() {
		int l = parse_mul();
		while (l >= 0) {
			const Op op = tok == "+" ? ADD : (tok == "-" ? SUB : CONST);
			if (tok_kind != 3 || op == CONST) {
				break;
			}
			next();
			const int r = parse_mul();
			l = r < 0 ? -1 : add(op, l, r);
		}
		return l;
	}

	int parse_mul() {
		int l = parse_unary();
		while (l >= 0) {
			const Op op = tok == "*" ? MUL : (tok == "/" ? DIV : CONST);
			if (tok_kind != 3 || op == CONST) {
				break;
			}
			next();
			const int r = parse_unary();
			l = r < 0 ? -1 : add(op, l, r);
		}
		return l;
	}

	int parse_unary() {
		if (tok_kind == 3 && tok == "-") {
			next();
			const int a = parse_unary();
			return a < 0 ? -1 : add(NEG, a);
		}
		return parse_primary();
	}

	int parse_primary() {
		if (tok_kind == 1) {
			const double v = tok.to_float(); // as GDScript's tokenizer reads its literals
			next();
			return add(CONST, -1, -1, -1, v);
		}
		if (accept("(")) {
			const int e = parse_ternary();
			if (!accept(")")) {
				fail("expected ')'");
				return -1;
			}
			return e;
		}
		if (tok_kind != 2) {
			fail("unexpected token");
			return -1;
		}
		const String name = tok;
		next();
		if (accept("(")) {
			struct Fn {
				const char *name;
				Op op;
				int args;
			};
			static const Fn fns[] = { { "smoothstep", SMOOTHSTEP, 3 }, { "lerpf", LERPF, 3 }, { "clampf", CLAMPF, 3 }, { "minf", MINF, 2 },
				{ "maxf", MAXF, 2 }, { "absf", ABSF, 1 }, { "sqrt", SQRT, 1 }, { "pow", POW, 2 } };
			for (const Fn &fn : fns) {
				if (name != fn.name) {
					continue;
				}
				int args[3] = { -1, -1, -1 };
				for (int k = 0; k < fn.args; k++) {
					if (k > 0 && !accept(",")) {
						fail(vformat("%s takes %d arguments", name, fn.args));
						return -1;
					}
					args[k] = parse_ternary();
					if (args[k] < 0) {
						return -1;
					}
				}
				if (!accept(")")) {
					fail("expected ')'");
					return -1;
				}
				return add(fn.op, args[0], args[1], args[2]);
			}
			fail("unknown function " + name);
			return -1;
		}
		for (int k = 0; k < CF_COUNT; k++) {
			if (name == CELL_FIELD_NAMES[k]) {
				return add(FIELD, k);
			}
		}
		const HashMap<String, int>::ConstIterator d = def_index.find(name);
		if (d == def_index.end()) {
			fail("unknown name");
			return -1;
		}
		const Variant &dv = def_src[d->value];
		if (dv.get_type() == Variant::INT || dv.get_type() == Variant::FLOAT) {
			return add(CONST, -1, -1, -1, double(dv));
		}
		if (def_root[d->value] == -2) {
			fail("a def that names itself");
			return -1;
		}
		if (def_root[d->value] == -1) {
			def_root[d->value] = -2;
			const int r = compile(dv);
			if (r < 0) {
				return -1;
			}
			def_root[d->value] = r;
		}
		return add(DEF, d->value, def_root[d->value]);
	}

	// Returns the root node of p_src, or -1 (and error) when it does not parse.
	int compile(const String &p_src) {
		const String saved_src = src;
		const int saved_pos = pos;
		const String saved_tok = tok;
		const int saved_kind = tok_kind;
		src = p_src;
		pos = 0;
		next();
		int r = parse_ternary();
		if (r >= 0 && tok_kind != 0) {
			fail("trailing input");
			r = -1;
		}
		src = saved_src;
		pos = saved_pos;
		tok = saved_tok;
		tok_kind = saved_kind;
		return r;
	}

	double eval(int p_n, FoliageCell &c) const {
		const Node &n = nodes[p_n];
		switch (n.op) {
			case CONST:
				return n.v;
			case FIELD:
				return c.field(n.a);
			case DEF:
				if (c.def_stamp[n.a] != c.stamp) {
					c.def_val[n.a] = eval(n.b, c);
					c.def_stamp[n.a] = c.stamp;
				}
				return c.def_val[n.a];
			case NEG:
				return -eval(n.a, c);
			case NOT:
				return eval(n.a, c) == 0.0 ? 1.0 : 0.0;
			case ADD:
				return eval(n.a, c) + eval(n.b, c);
			case SUB: {
				const double l = eval(n.a, c);
				return l - eval(n.b, c);
			}
			case MUL:
				return eval(n.a, c) * eval(n.b, c);
			case DIV: {
				const double l = eval(n.a, c);
				return l / eval(n.b, c);
			}
			case LT: {
				const double l = eval(n.a, c);
				return l < eval(n.b, c) ? 1.0 : 0.0;
			}
			case LE: {
				const double l = eval(n.a, c);
				return l <= eval(n.b, c) ? 1.0 : 0.0;
			}
			case GT: {
				const double l = eval(n.a, c);
				return l > eval(n.b, c) ? 1.0 : 0.0;
			}
			case GE: {
				const double l = eval(n.a, c);
				return l >= eval(n.b, c) ? 1.0 : 0.0;
			}
			case EQ: {
				const double l = eval(n.a, c);
				return l == eval(n.b, c) ? 1.0 : 0.0;
			}
			case NE: {
				const double l = eval(n.a, c);
				return l != eval(n.b, c) ? 1.0 : 0.0;
			}
			case AND:
				return (eval(n.a, c) != 0.0 && eval(n.b, c) != 0.0) ? 1.0 : 0.0;
			case OR:
				return (eval(n.a, c) != 0.0 || eval(n.b, c) != 0.0) ? 1.0 : 0.0;
			case SELECT:
				return eval(n.a, c) != 0.0 ? eval(n.b, c) : eval(n.c, c);
			case SMOOTHSTEP: {
				const double a = eval(n.a, c);
				const double b = eval(n.b, c);
				return VU::smoothstep(a, b, eval(n.c, c));
			}
			case LERPF: {
				const double a = eval(n.a, c);
				const double b = eval(n.b, c);
				return VU::lerpf(a, b, eval(n.c, c));
			}
			case CLAMPF: {
				const double a = eval(n.a, c);
				const double b = eval(n.b, c);
				return VU::clampf(a, b, eval(n.c, c));
			}
			case MINF: {
				const double a = eval(n.a, c);
				return VU::minf(a, eval(n.b, c));
			}
			case MAXF: {
				const double a = eval(n.a, c);
				return VU::maxf(a, eval(n.b, c));
			}
			case ABSF:
				return VU::absf(eval(n.a, c));
			case SQRT:
				return VU::sqrt(eval(n.a, c));
			case POW: {
				const double a = eval(n.a, c);
				return VU::pow(a, eval(n.b, c));
			}
		}
		return 0.0;
	}
};

struct FoliageRule {
	String name;
	PackedInt32Array ids;
	int share = -1; // RuleExpr roots
	int alt_w = -1;
	double scale_lo = 1.0;
	double scale_hi = 1.0;
	bool tree = false;
	double align = 0.0;
	double sink = 0.0;
	double footprint = 0.0;
	double max_tilt = Math::INF;
	double scale_pow = 1.0;
	int cluster_n = 0;
	double cluster_r = 0.0;
	double cluster_lo = 0.3;
	double cluster_hi = 0.7;
	double cluster_inner = 0.0;
	PackedInt32Array cluster_ids;
	double min_spacing = 0.0;
	PackedInt32Array alt_ids;
	PackedInt32Array alt_cluster_ids;
	int count = -1; // slots in Foliage::counts, given on the first instance
	int kids_count = -1;
	int spaced = -1; // slot in Foliage::spaced
};

struct FoliagePass {
	double step_m = 4.0;
	double slope_max = 0.7;
	double ground_max = Math::INF;
	double h_min = 3.0;
	double h_max = 15.0;
	double water_clear = 1.5;
	bool any_biome = true;
	bool biome_ok[256] = {};
	int density = -1;
	LocalVector<FoliageRule> rules;
};

struct Foliage {
	const float *heights = nullptr;
	const float *slope = nullptr;
	const uint8_t *biome = nullptr;
	const float *moisture = nullptr;
	const float *pine = nullptr;
	const float *wood = nullptr;
	const float *scrub = nullptr;
	const uint8_t *road_mask = nullptr;
	const float *water_level = nullptr;
	const int32_t *water_dist = nullptr;
	int px = 0;
	Vector2 origin;
	double spacing = 1.0;
	double sea_level = 0.0;
	double snow_line = 0.0;
	double cover_bin_m = 8.0;
	double edge_m = 4.0;
	Vector2i region_cell;
	double noise_scale = 1.0;
	PackedColorArray ground_colors;
	Ref<Image> color_map;
	HashMap<int, Vector2> seat_by_id;
	Ref<RandomNumberGenerator> rng;
	RuleExpr *expr = nullptr;
	PackedFloat32Array rocky;
	PackedFloat32Array wet_wide;
	PackedFloat32Array cover;
	int cover_bins = 0;
	double ground_max = Math::INF;

	struct Batch {
		int id = 0;
		LocalVector<Transform3D> xfs;
		LocalVector<Color> cols;
	};
	LocalVector<Batch> by_id; // in the order the ids first appeared
	HashMap<int, int> slot_of_id;
	LocalVector<Transform3D> trees;
	LocalVector<String> count_names;
	LocalVector<int64_t> counts;
	HashMap<String, int> count_slot;
	LocalVector<HashMap<Vector2i, PackedVector2Array>> spaced;
	HashMap<String, int> spaced_slot;

	// _height_at: bilinear between the four heightmap vertices around pos; NaN off the map.
	double height_at(const Vector3 &p_pos) const {
		const double fx = (double(p_pos.x) - double(origin.x)) / spacing;
		const double fz = (double(p_pos.z) - double(origin.y)) / spacing;
		const int64_t x0 = int64_t(Math::floor(fx));
		const int64_t z0 = int64_t(Math::floor(fz));
		const double tx = fx - x0;
		const double tz = fz - z0;
		if (x0 < 0 || z0 < 0 || x0 + 1 >= px || z0 + 1 >= px) {
			return Math::NaN;
		}
		const int64_t i = z0 * px + x0;
		const double h0 = double(heights[i]) + (double(heights[i + 1]) - double(heights[i])) * tx;
		const double h1 = double(heights[i + px]) + (double(heights[i + px + 1]) - double(heights[i + px])) * tx;
		return h0 + (h1 - h0) * tz;
	}

	// _normal_at: forward differences one vertex spacing apart.
	Vector3 normal_at(const Vector3 &p_pos) const {
		const double h = height_at(p_pos);
		const double u = h - height_at(p_pos + Vector3(spacing, 0.0, 0.0));
		const double v = h - height_at(p_pos + Vector3(0.0, 0.0, spacing));
		if (Math::is_nan(u) || Math::is_nan(v)) {
			return Vector3(0, 1, 0);
		}
		return Vector3(u, spacing, v).normalized();
	}

	bool too_steep(const Vector3 &p_pos) const {
		return ground_max < Math::INF && double(normal_at(p_pos).y) < 1.0 / VU::sqrt(1.0 + ground_max * ground_max);
	}

	// _ground_xf: seated, yawed, tilted align of the way to the normal, scaled, sunk.
	Transform3D ground_xf(Vector3 p_pos, double p_yaw, double p_scale, double p_align, double p_sink, double p_footprint, double p_max_tilt) const {
		const Vector3 up(0, 1, 0);
		p_pos.y = height_at(p_pos);
		Basis basis(up, p_yaw);
		const Vector3 n = normal_at(p_pos);
		if (p_align > 0.0 && n.is_finite() && double(n.length_squared()) > 0.5) {
			const double tilt = VU::minf(double(up.angle_to(n)) * p_align, p_max_tilt);
			const Vector3 axis = up.cross(n);
			if (double(axis.length_squared()) > 1e-6 && tilt > 0.001) {
				basis = Basis(axis.normalized(), tilt) * basis;
			}
		}
		basis = basis.scaled(Vector3(1, 1, 1) * real_t(p_scale));
		if (p_footprint > 0.0) {
			double floor_y = p_pos.y;
			for (int k = 0; k < 8; k++) {
				const double a = Math::TAU * k / 8.0;
				const Vector3 rim = basis.xform(Vector3(VU::cos(a) * p_footprint, 0.0, VU::sin(a) * p_footprint));
				const double g = height_at(p_pos + rim);
				if (!Math::is_nan(g)) {
					floor_y = VU::minf(floor_y, g - double(rim.y));
				}
			}
			p_pos.y = floor_y;
		}
		p_pos.y = double(p_pos.y) - p_sink;
		return Transform3D(basis, p_pos);
	}

	void count(int &r_slot, const String &p_name) {
		if (r_slot < 0) {
			const HashMap<String, int>::ConstIterator it = count_slot.find(p_name);
			if (it != count_slot.end()) {
				r_slot = it->value;
			} else {
				r_slot = counts.size();
				count_slot[p_name] = r_slot;
				count_names.push_back(p_name);
				counts.push_back(0);
			}
		}
		counts[r_slot]++;
	}

	// _add: the instance and the ground colour under heightmap cell i.
	void add(int p_id, const Transform3D &p_xf, int64_t p_i) {
		int slot;
		const HashMap<int, int>::ConstIterator it = slot_of_id.find(p_id);
		if (it != slot_of_id.end()) {
			slot = it->value;
		} else {
			slot = by_id.size();
			slot_of_id[p_id] = slot;
			by_id.push_back(Batch());
			by_id[slot].id = p_id;
		}
		Color col(1, 1, 1);
		if (!ground_colors.is_empty()) {
			col = ground_colors[MIN(int(biome[p_i]), ground_colors.size() - 1)];
			if (color_map.is_valid()) {
				const Color m = color_map->get_pixel(p_i % px, p_i / px);
				col = Color(double(col.r) * m.r, double(col.g) * m.g, double(col.b) * m.b);
			}
		}
		by_id[slot].xfs.push_back(p_xf);
		by_id[slot].cols.push_back(col);
	}

	// _claim_spacing: true (and the point kept) if no earlier instance of the rule is within min_spacing.
	bool claim_spacing(FoliageRule &r, const Vector3 &p_pos) {
		if (r.spaced < 0) {
			const HashMap<String, int>::ConstIterator it = spaced_slot.find(r.name);
			if (it != spaced_slot.end()) {
				r.spaced = it->value;
			} else {
				r.spaced = spaced.size();
				spaced_slot[r.name] = r.spaced;
				spaced.push_back(HashMap<Vector2i, PackedVector2Array>());
			}
		}
		HashMap<Vector2i, PackedVector2Array> &bins = spaced[r.spaced];
		const double s = r.min_spacing;
		const Vector2 p(p_pos.x, p_pos.z);
		const Vector2i b(int(Math::floor(double(p.x) / s)), int(Math::floor(double(p.y) / s)));
		const double s2 = s * s;
		for (int dy = -1; dy < 2; dy++) {
			for (int dx = -1; dx < 2; dx++) {
				const HashMap<Vector2i, PackedVector2Array>::ConstIterator q = bins.find(Vector2i(b.x + dx, b.y + dy));
				if (q == bins.end()) {
					continue;
				}
				for (const Vector2 &o : q->value) {
					if (double(p.distance_squared_to(o)) < s2) {
						return false;
					}
				}
			}
		}
		bins[b].push_back(p);
		return true;
	}

	void place_cluster(FoliageRule &r, const Vector3 &p_pos, double p_parent_scale, const PackedInt32Array &p_ids) {
		if (p_ids.is_empty()) {
			return;
		}
		const int n = rng->randi_range((r.cluster_n + 1) / 2, r.cluster_n);
		const double radius = r.cluster_r * p_parent_scale;
		for (int k = 0; k < n; k++) {
			const double a = double(rng->randf()) * Math::TAU;
			const double d = radius * VU::sqrt(VU::lerpf(r.cluster_inner * r.cluster_inner, 1.0, rng->randf()));
			const Vector3 pos(double(p_pos.x) + VU::cos(a) * d, 0.0, double(p_pos.z) + VU::sin(a) * d);
			const int64_t cx = int64_t((double(pos.x) - double(origin.x)) / spacing);
			const int64_t cz = int64_t((double(pos.z) - double(origin.y)) / spacing);
			if (cx < 1 || cz < 1 || cx >= px - 1 || cz >= px - 1) {
				continue;
			}
			const int64_t j = cz * px + cx;
			if (road_mask[j] != 0 || double(water_level[j]) > double(heights[j]) + 0.45) {
				continue;
			}
			const double s = p_parent_scale * double(rng->randf_range(r.cluster_lo, r.cluster_hi));
			const double yaw = double(rng->randf()) * Math::TAU;
			const Transform3D xf = ground_xf(pos, yaw, s, r.align, r.sink, r.footprint, Math::INF);
			const int id = p_ids[int64_t(rng->randi()) % p_ids.size()];
			if (too_steep(pos)) {
				continue;
			}
			add(id, xf, j);
			count(r.kids_count, r.name + "_kids");
		}
	}

	void place(FoliageRule &r, FoliageCell &c, const Vector3 &p_pos, int64_t p_i) {
		if (r.min_spacing > 0.0 && !claim_spacing(r, p_pos)) {
			return;
		}
		const double yaw = double(rng->randf()) * Math::TAU;
		const double scale = r.scale_lo + (r.scale_hi - r.scale_lo) * VU::pow(rng->randf(), r.scale_pow);
		const PackedInt32Array *ids = &r.ids;
		const PackedInt32Array *kids = r.cluster_ids.is_empty() ? &r.ids : &r.cluster_ids;
		if (r.alt_w >= 0 && !r.alt_ids.is_empty()) {
			const double w = expr->eval(r.alt_w, c);
			if (w > 0.0 && double(rng->randf()) < w) {
				ids = &r.alt_ids;
				kids = r.alt_cluster_ids.is_empty() ? &r.alt_ids : &r.alt_cluster_ids;
			}
		}
		const int id = (*ids)[int64_t(rng->randi()) % ids->size()];
		double sink = r.sink;
		double footprint = r.footprint;
		const HashMap<int, Vector2>::ConstIterator seat = seat_by_id.find(id);
		if (seat != seat_by_id.end()) {
			sink = seat->value.x;
			footprint = seat->value.y;
		}
		const Transform3D xf = ground_xf(p_pos, yaw, scale, r.align, sink, footprint, r.max_tilt);
		// Too steep places nothing but draws the same numbers, so the rest of the layout stays put.
		if (too_steep(p_pos)) {
			if (r.cluster_n > 0) {
				place_cluster(r, p_pos, scale, *kids);
			}
			return;
		}
		add(id, xf, p_i);
		count(r.count, r.name);
		if (r.tree) {
			trees.push_back(xf);
		}
		if (r.cluster_n > 0) {
			place_cluster(r, p_pos, scale, *kids);
		}
	}

	void run_pass(FoliagePass &p, FoliageCell &c) {
		const double step = p.step_m;
		ground_max = p.ground_max;
		const int64_t n = int64_t(Math::ceil((px * spacing - 2.0 * edge_m) / step));
		for (int64_t j = 0; j < n; j++) {
			const double gy = edge_m + j * step;
			for (int64_t k = 0; k < n; k++) {
				const double gx = edge_m + k * step;
				// Jitter first and test the cell the instance actually lands in.
				const real_t jx = rng->randf();
				const real_t jy = rng->randf();
				const Vector2 jitter = Vector2(jx, jy) * real_t(step);
				const Vector3 pos(double(origin.x) + gx + double(jitter.x), 0.0, double(origin.y) + gy + double(jitter.y));
				const int64_t cx = CLAMP(int64_t((double(pos.x) - double(origin.x)) / spacing), int64_t(0), int64_t(px - 1));
				const int64_t cz = CLAMP(int64_t((double(pos.z) - double(origin.y)) / spacing), int64_t(0), int64_t(px - 1));
				const int64_t i = cz * px + cx;
				if (road_mask[i] != 0 || (!p.any_biome && !p.biome_ok[biome[i]])) {
					continue;
				}
				const double h = heights[i];
				if (h < sea_level + p.h_min || h > snow_line - p.h_max || double(water_level[i]) > h - p.water_clear) {
					continue;
				}
				if (double(slope[i]) > p.slope_max) {
					continue;
				}
				c.stamp++;
				c.noise_done = 0;
				c.nx = (region_cell.x + cx) * noise_scale;
				c.nz = (region_cell.y + cz) * noise_scale;
				double *f = c.f;
				f[CF_H] = h;
				f[CF_SLOPE] = slope[i];
				f[CF_ROCKY] = rocky[i];
				f[CF_BIOME] = biome[i];
				f[CF_WET] = moisture[i];
				f[CF_WET_WIDE] = wet_wide[i];
				f[CF_PINE] = pine[i];
				f[CF_WOOD] = wood[i];
				f[CF_SCRUB] = scrub[i];
				// Blurred trees per 8 m bin: ~0.6 is a closed stand, 0.15 its fringe.
				const int64_t bx = MIN(int64_t((double(pos.x) - double(origin.x)) / cover_bin_m), int64_t(cover_bins - 1));
				const int64_t bz = MIN(int64_t((double(pos.z) - double(origin.y)) / cover_bin_m), int64_t(cover_bins - 1));
				const double cv = cover[bz * cover_bins + bx];
				f[CF_COVER] = cv;
				f[CF_WATER_DIST] = water_dist[i] * spacing;
				f[CF_DEPTH] = double(water_level[i]) - h;
				f[CF_CANOPY] = VU::smoothstep(0.25, 0.6, cv);
				f[CF_EDGE] = VU::smoothstep(0.06, 0.18, cv) * (1.0 - VU::smoothstep(0.3, 0.5, cv));
				f[CF_OPEN] = 1.0 - VU::smoothstep(0.08, 0.25, cv);
				double r = rng->randf();
				if (p.density >= 0) {
					const double keep = expr->eval(p.density, c);
					if (r >= keep) {
						continue;
					}
					r /= keep;
				}
				for (FoliageRule &rule : p.rules) {
					// An unavailable species cannot win the cell or consume its draw.
					if (rule.ids.is_empty()) {
						continue;
					}
					const double s = expr->eval(rule.share, c);
					if (s <= 0.0) {
						continue;
					}
					if (r < s) {
						place(rule, c, pos, i);
						break;
					}
					r -= s;
				}
			}
		}
	}

	// _rebuild_cover: trees per 8 m bin, blurred over ~50 m.
	void rebuild_cover() {
		const int cb = cover_bins;
		PackedFloat32Array f;
		f.resize(int64_t(cb) * cb);
		f.fill(0.0f);
		float *fw = f.ptrw();
		for (const Transform3D &xf : trees) {
			const int64_t bx = CLAMP(int64_t((double(xf.origin.x) - double(origin.x)) / cover_bin_m), int64_t(0), int64_t(cb - 1));
			const int64_t bz = CLAMP(int64_t((double(xf.origin.z) - double(origin.y)) / cover_bin_m), int64_t(0), int64_t(cb - 1));
			fw[bz * cb + bx] = double(fw[bz * cb + bx]) + double(xf.basis.get_scale().y);
		}
		cover = blur_field(f, cb, 6);
	}
};

} // namespace

// foliage_placer.gd's run and world_gen.gd's banding after it: FoliagePlacer passes (forest_rules.gd)
// over the map, one random sequence from the first candidate to the last instance. p_params:
// heights, slope, biome, moisture, pine, wood, scrub (biome weights), road_mask, water_level,
// water_dist (ring distance, cells), px, origin, spacing, sea_level, snow_line, seed, smooth_px,
// wet_px (moisture blur for wet_wide), noise_scale, region_cell, ground_colors, color_map,
// seat_by_id, passes, defs (named expressions and constants the rules use), cover_bin_m, edge_m,
// band_m. Returns {batches ([mesh id, Array[Transform3D], PackedColorArray], band after band along
// z, ids in first-placed order), trees, counts, pass_ms}.
Dictionary WorldKernels::foliage(const Dictionary &p_params) {
	Dictionary out;
	Foliage fo;
	const PackedFloat32Array heights = p_params["heights"];
	const PackedFloat32Array slope = p_params["slope"];
	const PackedByteArray biome = p_params["biome"];
	const PackedFloat32Array moisture = p_params["moisture"];
	const PackedFloat32Array pine = p_params["pine"];
	const PackedFloat32Array wood = p_params["wood"];
	const PackedFloat32Array scrub = p_params["scrub"];
	const PackedByteArray road_mask = p_params["road_mask"];
	const PackedFloat32Array water_level = p_params["water_level"];
	const PackedInt32Array water_dist = p_params["water_dist"];
	fo.px = p_params["px"];
	const int64_t n = int64_t(fo.px) * fo.px;
	ERR_FAIL_COND_V(fo.px < 2 || heights.size() != n || slope.size() != n || biome.size() != n || moisture.size() != n || pine.size() != n || wood.size() != n || scrub.size() != n || road_mask.size() != n || water_level.size() != n || water_dist.size() != n, out);
	fo.heights = heights.ptr();
	fo.slope = slope.ptr();
	fo.biome = biome.ptr();
	fo.moisture = moisture.ptr();
	fo.pine = pine.ptr();
	fo.wood = wood.ptr();
	fo.scrub = scrub.ptr();
	fo.road_mask = road_mask.ptr();
	fo.water_level = water_level.ptr();
	fo.water_dist = water_dist.ptr();
	fo.origin = p_params["origin"];
	fo.spacing = p_params["spacing"];
	fo.sea_level = p_params["sea_level"];
	fo.snow_line = p_params["snow_line"];
	fo.cover_bin_m = p_params["cover_bin_m"];
	fo.edge_m = p_params["edge_m"];
	fo.region_cell = p_params["region_cell"];
	fo.noise_scale = p_params["noise_scale"];
	fo.ground_colors = p_params["ground_colors"];
	fo.color_map = p_params["color_map"];
	const Dictionary seats = p_params["seat_by_id"];
	for (const KeyValue<Variant, Variant> &kv : seats) {
		fo.seat_by_id[int(kv.key)] = kv.value;
	}
	const int64_t world_seed = p_params["seed"];

	RuleExpr expr(p_params["defs"]);
	fo.expr = &expr;
	LocalVector<FoliagePass> passes;
	for (const Variant &pv : Array(p_params["passes"])) {
		Object *po = pv;
		ERR_CONTINUE(po == nullptr);
		passes.push_back(FoliagePass());
		FoliagePass &p = passes[passes.size() - 1];
		p.step_m = po->get("step_m");
		p.slope_max = po->get("slope_max");
		p.ground_max = po->get("ground_max");
		p.h_min = po->get("h_min");
		p.h_max = po->get("h_max");
		p.water_clear = po->get("water_clear");
		const PackedInt32Array biomes = po->get("biomes");
		p.any_biome = biomes.is_empty();
		for (int b : biomes) {
			if (b >= 0 && b < 256) {
				p.biome_ok[b] = true;
			}
		}
		const String density = po->get("density");
		if (!density.is_empty()) {
			p.density = expr.compile(density);
			ERR_FAIL_COND_V_MSG(p.density < 0, out, "Foliage pass density: " + expr.error);
		}
		for (const Variant &rv : Array(po->get("rules"))) {
			Object *ro = rv;
			ERR_CONTINUE(ro == nullptr);
			p.rules.push_back(FoliageRule());
			FoliageRule &r = p.rules[p.rules.size() - 1];
			r.name = ro->get("name");
			r.ids = PackedInt32Array(ro->get("ids"));
			r.share = expr.compile(ro->get("share"));
			ERR_FAIL_COND_V_MSG(r.share < 0, out, "Foliage rule '" + r.name + "': " + expr.error);
			const String alt_w = ro->get("alt_w");
			if (!alt_w.is_empty()) {
				r.alt_w = expr.compile(alt_w);
				ERR_FAIL_COND_V_MSG(r.alt_w < 0, out, "Foliage rule '" + r.name + "' alt_w: " + expr.error);
			}
			r.scale_lo = ro->get("scale_lo");
			r.scale_hi = ro->get("scale_hi");
			r.tree = ro->get("tree");
			r.align = ro->get("align");
			r.sink = ro->get("sink");
			r.footprint = ro->get("footprint");
			r.max_tilt = ro->get("max_tilt");
			r.scale_pow = ro->get("scale_pow");
			r.cluster_n = ro->get("cluster_n");
			r.cluster_r = ro->get("cluster_r");
			r.cluster_lo = ro->get("cluster_lo");
			r.cluster_hi = ro->get("cluster_hi");
			r.cluster_inner = ro->get("cluster_inner");
			r.cluster_ids = PackedInt32Array(ro->get("cluster_ids"));
			r.min_spacing = ro->get("min_spacing");
			r.alt_ids = PackedInt32Array(ro->get("alt_ids"));
			r.alt_cluster_ids = PackedInt32Array(ro->get("alt_cluster_ids"));
		}
	}

	fo.rng.instantiate();
	// Region (0, 0) keeps the seed it always had.
	fo.rng->set_seed(world_seed + 10 + int64_t(fo.region_cell.x) * 73856093 + int64_t(fo.region_cell.y) * 19349663);
	// Stand density and species noises; moisture comes from Biomes.
	Ref<FastNoiseLite> noise[3];
	const int64_t noise_seed[3] = { 5, 7, 11 };
	const int noise_octaves[3] = { 3, 2, 2 };
	const double noise_freq[3] = { 0.006, 0.0025, 0.08 };
	FoliageCell c;
	for (int k = 0; k < 3; k++) {
		noise[k].instantiate();
		noise[k]->set_seed(world_seed + noise_seed[k]);
		noise[k]->set_fractal_octaves(noise_octaves[k]);
		noise[k]->set_frequency(noise_freq[k]);
		c.noise[k] = noise[k].ptr();
	}
	c.def_val.resize_initialized(expr.def_src.size());
	c.def_stamp.resize_initialized(expr.def_src.size());
	c.f[CF_SEA] = fo.sea_level;
	c.f[CF_SNOW] = fo.snow_line;

	const uint64_t t_prep = ticks();
	fo.cover_bins = int(fo.px * fo.spacing / fo.cover_bin_m);
	fo.cover.resize(int64_t(fo.cover_bins) * fo.cover_bins);
	fo.cover.fill(0.0f);
	// The cliff weight the control map paints with: slope low-passed, rock from ~35 degrees. The
	// two blurs are independent and each runs on one thread, so they run side by side.
	PackedFloat32Array wide;
	const int smooth_px = p_params["smooth_px"];
	const int wet_px = p_params["wet_px"];
	for_each_row(2, [&](int k) {
		if (k == 0) {
			wide = blur_field(slope, fo.px, smooth_px);
		} else {
			fo.wet_wide = blur_field(moisture, fo.px, wet_px);
		}
	});
	fo.rocky.resize(n);
	float *rocky = fo.rocky.ptrw();
	const float *wide_r = wide.ptr();
	for_each_row(fo.px, [&](int y) {
		for (int64_t i = int64_t(y) * fo.px; i < int64_t(y + 1) * fo.px; i++) {
			rocky[i] = VU::smoothstep(0.7, 1.6, wide_r[i]);
		}
	});

	out["prep_ms"] = int(ticks() - t_prep);
	PackedInt32Array pass_ms;
	uint64_t cover_ms = 0;
	for (FoliagePass &p : passes) {
		const uint64_t tp = ticks();
		fo.run_pass(p, c);
		pass_ms.push_back(int(ticks() - tp));
		for (const FoliageRule &r : p.rules) {
			if (r.tree) {
				const uint64_t tc = ticks();
				fo.rebuild_cover();
				cover_ms += ticks() - tc;
				break;
			}
		}
	}

	out["cover_ms"] = int(cover_ms);
	const uint64_t t_band = ticks();
	// world_gen's _band_instances: per band along z, per id, the instances in the order placed.
	const double band_m = p_params["band_m"];
	const int bands = MAX(1, int(Math::ceil(fo.px * fo.spacing / band_m)));
	const int nid = fo.by_id.size();
	LocalVector<LocalVector<uint32_t>> band_of; // per id, the band of each instance
	LocalVector<int> per_cell; // [band * nid + id] count
	per_cell.resize_initialized(int64_t(bands) * nid);
	band_of.resize(nid);
	for (int k = 0; k < nid; k++) {
		const Foliage::Batch &bt = fo.by_id[k];
		band_of[k].resize(bt.xfs.size());
		for (uint32_t i = 0; i < bt.xfs.size(); i++) {
			const int b = CLAMP(int(Math::floor((double(bt.xfs[i].origin.z) - double(fo.origin.y)) / band_m)), 0, bands - 1);
			band_of[k][i] = b;
			per_cell[b * nid + k]++;
		}
	}
	Array batches;
	for (int b = 0; b < bands; b++) {
		for (int k = 0; k < nid; k++) {
			const int cnt = per_cell[b * nid + k];
			if (cnt == 0) {
				continue;
			}
			const Foliage::Batch &bt = fo.by_id[k];
			TypedArray<Transform3D> xfs;
			xfs.resize(cnt);
			PackedColorArray cols;
			cols.resize(cnt);
			Color *cw = cols.ptrw();
			int m = 0;
			for (uint32_t i = 0; i < bt.xfs.size(); i++) {
				if (band_of[k][i] == uint32_t(b)) {
					xfs[m] = bt.xfs[i];
					cw[m] = bt.cols[i];
					m++;
				}
			}
			Array batch;
			batch.push_back(bt.id);
			batch.push_back(xfs);
			batch.push_back(cols);
			batches.push_back(batch);
		}
	}
	TypedArray<Transform3D> trees;
	trees.resize(fo.trees.size());
	for (uint32_t i = 0; i < fo.trees.size(); i++) {
		trees[i] = fo.trees[i];
	}
	Dictionary counts;
	for (uint32_t k = 0; k < fo.counts.size(); k++) {
		counts[fo.count_names[k]] = fo.counts[k];
	}
	out["band_ms"] = int(ticks() - t_band);
	out["batches"] = batches;
	out["trees"] = trees;
	out["counts"] = counts;
	out["pass_ms"] = pass_ms;
	return out;
}

// near_plants.gd's _insert over a whole batch: store[chunk][id] = [Array[Transform3D], PackedColorArray],
// chunk = floor(xz / chunk_m); with near_cell_m > 0 also near_grid[floor(xz / near_cell_m)] gets
// Vector4i(chunk, id, index). Keys are added in the order the script added them.
void WorldKernels::chunk_insert(Dictionary p_store, Dictionary p_near_grid, int p_id, const Array &p_xforms, const PackedColorArray &p_colors, double p_chunk_m, double p_near_cell_m) {
	struct Slot {
		Array entry;
		Array xforms;
		PackedColorArray colors;
	};
	HashMap<Vector2i, Slot> slots;
	HashMap<Vector2i, Array> near_lists;
	const Variant id = p_id;
	for (int i = 0; i < p_xforms.size(); i++) {
		const Transform3D xf = p_xforms[i];
		const Vector2i c(int(Math::floor(double(xf.origin.x) / p_chunk_m)), int(Math::floor(double(xf.origin.z) / p_chunk_m)));
		Slot *slot = slots.getptr(c);
		if (slot == nullptr) {
			Dictionary by_id = p_store.get(c, Dictionary());
			if (!by_id.has(id)) {
				Array xfs;
				xfs.set_typed(Variant::TRANSFORM3D, StringName(), Variant());
				Array entry;
				entry.push_back(xfs);
				entry.push_back(PackedColorArray());
				by_id[id] = entry;
			}
			p_store[c] = by_id;
			Slot fresh;
			fresh.entry = by_id[id];
			fresh.xforms = fresh.entry[0];
			fresh.colors = fresh.entry[1];
			// Ours alone while appending, so append does not copy it every time.
			fresh.entry[1] = PackedColorArray();
			slot = &slots.insert(c, fresh)->value;
		}
		if (p_near_cell_m > 0.0) {
			const Vector2i g(int(Math::floor(double(xf.origin.x) / p_near_cell_m)), int(Math::floor(double(xf.origin.z) / p_near_cell_m)));
			Array *list = near_lists.getptr(g);
			if (list == nullptr) {
				Array found = p_near_grid.get(g, Array());
				p_near_grid[g] = found;
				list = &near_lists.insert(g, found)->value;
			}
			list->push_back(Vector4i(c.x, c.y, p_id, slot->xforms.size()));
		}
		slot->xforms.push_back(xf);
		slot->colors.push_back(i < p_colors.size() ? p_colors[i] : Color(1, 1, 1, 1));
	}
	for (KeyValue<Vector2i, Slot> &kv : slots) {
		kv.value.entry[1] = kv.value.colors;
	}
}

void WorldKernels::_bind_methods() {
	ClassDB::bind_static_method("WorldKernels", D_METHOD("priority_flood", "heights", "px", "sea_floor", "gates", "rise", "eps"), &WorldKernels::priority_flood);
	ClassDB::bind_static_method("WorldKernels", D_METHOD("island_heights", "params"), &WorldKernels::island_heights);
	ClassDB::bind_static_method("WorldKernels", D_METHOD("biome_fields", "params"), &WorldKernels::biome_fields);
	ClassDB::bind_static_method("WorldKernels", D_METHOD("hydrology", "params"), &WorldKernels::hydrology);
	ClassDB::bind_static_method("WorldKernels", D_METHOD("ring_distance", "levels", "size", "rings"), &WorldKernels::ring_distance);
	ClassDB::bind_static_method("WorldKernels", D_METHOD("water_map", "params"), &WorldKernels::water_map);
	ClassDB::bind_static_method("WorldKernels", D_METHOD("resample_grid", "src", "src_px", "dst_px", "step"), &WorldKernels::resample_grid);
	ClassDB::bind_static_method("WorldKernels", D_METHOD("erode", "params"), &WorldKernels::erode);
	ClassDB::bind_static_method("WorldKernels", D_METHOD("control_map", "params"), &WorldKernels::control_map);
	ClassDB::bind_static_method("WorldKernels", D_METHOD("road_grid", "params"), &WorldKernels::road_grid);
	ClassDB::bind_static_method("WorldKernels", D_METHOD("road_astar", "params"), &WorldKernels::road_astar);
	ClassDB::bind_static_method("WorldKernels", D_METHOD("road_rasterize", "params"), &WorldKernels::road_rasterize);
	ClassDB::bind_static_method("WorldKernels", D_METHOD("barn_find", "params"), &WorldKernels::barn_find);
	ClassDB::bind_static_method("WorldKernels", D_METHOD("foliage", "params"), &WorldKernels::foliage);
	ClassDB::bind_static_method("WorldKernels", D_METHOD("chunk_insert", "store", "near_grid", "id", "xforms", "colors", "chunk_m", "near_cell_m"), &WorldKernels::chunk_insert);
	ClassDB::bind_static_method("WorldKernels", D_METHOD("slope", "heights", "size", "o", "inv"), &WorldKernels::slope);
	ClassDB::bind_static_method("WorldKernels", D_METHOD("floor_up", "src", "src_px", "dst_px", "k"), &WorldKernels::floor_up);
}
