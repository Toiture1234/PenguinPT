#pragma once

#define DT_SAMPLES 1024

namespace penguinPT {
	class Volume {
	public:
		nanovdb::FloatGrid* density_grid; // density grid, on device side if cuda else on host
		float sigma_t;
		nanovdb::Vec3f albedo;
		
		// phase function
		float g;

		// transform
		float scale = 1.f;
		nanovdb::Vec3f position = { 0.f, 0.f, 0.f };

		Volume() : density_grid(nullptr), sigma_t(0.f), albedo(0.f), g(0.f) {}

		__hostdev__ bool intersect_volume_replace(nanovdb::math::Ray<float> ray, float& t_out, nanovdb::Vec3f& normal, bool& inside) {
			ray.setEye((ray.eye() - position) / scale);
			if (!ray.clip(density_grid->worldBBox())) return false;
			float t0 = ray.t0();
			float t1 = ray.t1();

			inside = t0 <= 0.01f;
			float t = (inside ? t1 : t0) * scale;
			if (t > t_out) return false;
			t_out = t;
			normal = -ray.dir();
			
			return true;
		}
		__hostdev__ float get_density(nanovdb::Vec3f pos, nanovdb::Vec3f offset) {
			auto acc = density_grid->getAccessor();
			pos -= position;
			pos /= scale;
			nanovdb::Vec3f Ipos = density_grid->worldToIndexF(pos);
			Ipos += offset;
			return fminf(acc.getValue(nanovdb::Coord(Ipos[0], Ipos[1], Ipos[2])), 1.f);
		}

		__device__ bool delta_tracking(nanovdb::math::Ray<float> ray, float& t_out, Rand_state& state);
	};

	__device__ bool Volume::delta_tracking(nanovdb::math::Ray<float> ray, float& t_out, Rand_state& state) {
		float t = 0.f;

		float inv_sigma_T = 1.f / sigma_t;
		for (int i = 0; i < DT_SAMPLES; i++) {
			t -= logf(randC(&state)) * inv_sigma_T;

			if (t > t_out) return false;

			float density = get_density(ray(t), nanovdb::Vec3f(0.f));

			float zeta = randC(&state);
			if ( zeta < density) {
				t_out = t;
				return true;
			}
		}
		return false;
	}
}