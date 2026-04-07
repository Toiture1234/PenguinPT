#pragma once

namespace penguinPT {

	// ratio tracking
	__device__ static float ratio_tracking_spectral_device(renderer_services& rs, nanovdb::math::Ray<float> ray, float t_max, int nb_vol, Volume** volumes, int channel);
	__host__ static float ratio_tracking_spectral_host(renderer_services& rs, nanovdb::math::Ray<float> ray, float t_max, int nb_vol, Volume** volumes, int channel);

	__device__ float direct_visibility_device(renderer_services& rs, nanovdb::math::Ray<float> ray, bool is_inside, nanovdb::Vec3f attenuation, float scattering_sss, int channel) {
		float T = 1.f;

		// loop in scene
		for (int i = 0; i < DIRECT_LIGHT_STEPS; i++) {
			hit_info info;
			info.t = MAX_DISTANCE;

			bool hit = rs.scene.intersectScene_full(ray, info);

			if (!hit) return T;

			// solid hit
			principled_BSDF& surface_bsdf = rs.scene.bsdf_list[info.BSDF_index];
			if (!surface_bsdf.is_through) return 0.f;

			// now the real battle begins
			// account for volumes transmittance
			T *= ratio_tracking_spectral_device(rs, ray, info.t, info.nb_vol, info.all_volumes, channel);

			// account for solids' absorption
			if(is_inside) T *= expf(-info.t * (attenuation[channel] + scattering_sss));
			
			// account for translucent BSDF 
			float eta = is_inside ? surface_bsdf.ior : 1.f / surface_bsdf.ior;
			float bsdf_pdf = INV_4_PI;
			nanovdb::Vec3f bsdf_value = surface_bsdf.eval(-ray.dir(), ray.dir(), info.normal, bsdf_pdf, eta);

			T *= bsdf_value[channel] / INV_4_PI;

			// change inside if BSDF is through
			if (surface_bsdf.is_through && surface_bsdf.change_medium) is_inside = !is_inside, attenuation = surface_bsdf.absorption, scattering_sss = surface_bsdf.scattering;

			ray.setEye(ray(info.t));
		}

		return T;
	}
	__host__ float direct_visibility_host(renderer_services& rs, nanovdb::math::Ray<float> ray, bool is_inside, nanovdb::Vec3f attenuation, float scattering_sss, int channel) {
		float T = 1.f;

		// loop in scene
		for (int i = 0; i < DIRECT_LIGHT_STEPS; i++) {
			hit_info info;
			info.t = MAX_DISTANCE;

			bool hit = rs.scene.intersectScene_full(ray, info);

			if (!hit) return T;

			// solid hit
			principled_BSDF& surface_bsdf = rs.scene.bsdf_list[info.BSDF_index];
			if (!surface_bsdf.is_through) return 0.f;

			// now the real battle begins
			// account for volumes transmittance
			T *= ratio_tracking_spectral_host(rs, ray, info.t, info.nb_vol, info.all_volumes, channel);

			// account for solids' absorption
			if (is_inside) T *= expf(-info.t * (attenuation[channel] + scattering_sss));

			// account for translucent BSDF 
			float eta = is_inside ? surface_bsdf.ior : 1.f / surface_bsdf.ior;
			float bsdf_pdf = INV_4_PI;
			nanovdb::Vec3f bsdf_value = surface_bsdf.eval(-ray.dir(), ray.dir(), info.normal, bsdf_pdf, eta);

			T *= bsdf_value[channel] / INV_4_PI;

			// change inside if BSDF is through
			if (surface_bsdf.is_through && surface_bsdf.change_medium) is_inside = !is_inside, attenuation = surface_bsdf.absorption, scattering_sss = surface_bsdf.scattering;

			ray.setEye(ray(info.t));
		}

		return T;
	}

	__device__ float ratio_tracking_spectral_device(renderer_services& rs, nanovdb::math::Ray<float> ray, float t_max, int nb_vol, Volume** volumes, int channel) {
		if (nb_vol == 0) return 1.f;

		float summed_sigma_t = 0.f;
		for (int i = 0; i < nb_vol; i++) summed_sigma_t += volumes[i]->sigma_t[channel];

		float inv_sigma_t = 1.f / summed_sigma_t;

		float t = 0.f;
		float T = 1.f;

		for (int i = 0; i < DT_SAMPLES; i++) {
			t -= logf(randC(&rs.rng_state)) * inv_sigma_t;

			if (t > t_max) return T;

			nanovdb::Vec3f p = ray(t);
			
			// get density
			float density = 0.f;
			for (int i = 0; i < nb_vol; i++) {
				Volume* current_volume = volumes[i];
				density += current_volume->get_density(p, nanovdb::Vec3f(0.f)) * current_volume->sigma_t[channel];
			}
			T *= 1.f - density * inv_sigma_t;
		}
		return T;
	}
	__host__ float ratio_tracking_spectral_host(renderer_services& rs, nanovdb::math::Ray<float> ray, float t_max, int nb_vol, Volume** volumes, int channel) {
		if (nb_vol == 0) return 1.f;

		float summed_sigma_t = 0.f;
		for (int i = 0; i < nb_vol; i++) summed_sigma_t += volumes[i]->sigma_t[channel];

		float inv_sigma_t = 1.f / summed_sigma_t;

		float t = 0.f;
		float T = 1.f;

		for (int i = 0; i < DT_SAMPLES; i++) {
			t -= logf(rand01) * inv_sigma_t;

			if (t > t_max) return T;

			nanovdb::Vec3f p = ray(t);

			// get density
			float density = 0.f;
			for (int i = 0; i < nb_vol; i++) {
				Volume* current_volume = volumes[i];
				density += current_volume->get_density(p, nanovdb::Vec3f(0.f)) * current_volume->sigma_t[channel];
			}
			T *= 1.f - density * inv_sigma_t;
		}
		return T;
	}
}