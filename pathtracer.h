#pragma once

namespace penguinPT {

	
	
	__device__ nanovdb::Vec3f volume_pathtrace_device_spectral(renderer_services& rs, nanovdb::math::Ray<float> ray) {
		nanovdb::Vec3f L(0.f);
		float throughput(1.f);

		int channel = int(randC(&rs.rng_state) * 3.f);

		float scatterPDF = 1.f; // scatter pdf from last bounce, used for MIS
		bool surface_scatter = false;

		// consider we are outside any volume at the beginning, this doesn't account for heterogeous volumes,
		// only for volumes encapsulated by geometry, no stack of mediums rn
		bool isInside = false;

		nanovdb::Vec3f attenuation = nanovdb::Vec3f(0.f);
		float scattering_sss = 0.f;
		float sss_g = 0.f;

		for (unsigned int i = 0; i < BOUNCES_PT_VOL; i++) {
			hit_info info;
			info.t = MAX_DISTANCE;

			bool hit = rs.scene.intersectScene_full(ray, info);
			bool through = false;

			// fix normal
			info.normal = util::refIfNeg(info.normal, -ray.dir());

			if (!hit) {
#ifdef DIRECT_LIGHTNING
				float envmap_pdf = 1.f;
				nanovdb::Vec3f envmap_col = rs.scene.environnement_map.eval_envmap(ray.dir(), envmap_pdf);

				float misWeight = 1.f;
				if (i > 0)
					misWeight = util::powerHeuristic(scatterPDF, envmap_pdf);

				if (!surface_scatter)
					misWeight = 1.f;

				if (misWeight > 0.f)
					L[channel] += (misWeight * envmap_col)[channel] * throughput;
#else 
				float envmap_pdf;
				L[channel] += (rs.scene.environnement_map.eval_envmap(ray.dir(), envmap_pdf))[channel] * throughput;
#endif 
				break;
			}

			nanovdb::Vec3f bsdf_value;
			float t_max = info.t;

			int action_volume_index;
			bool volume_scatter = integrators::heterogeneous_volumes_integrator::integrate_spectral_device(rs, ray, t_max, info.nb_vol, info.all_volumes, action_volume_index, channel);

			// SSS integration
			bool sss_scatter = false;
			if (isInside) {
				float t_SSS = -logf(randC(&rs.rng_state)) / scattering_sss;
				if (t_SSS < t_max) {
					sss_scatter = true, t_max = t_SSS;

					nanovdb::Vec3f wi;
					bsdf_value = phase_function::Henyey_greenstein::sample(ray.dir(), wi, sss_g, scatterPDF, rs.rng_state);

#ifdef DIRECT_LIGHTNING
					{
						nanovdb::Vec3f envmap_dir;
						float envmap_pdf;

						nanovdb::Vec3f envmap_value = rs.scene.environnement_map.sample_envmap(envmap_dir, envmap_pdf, rs.rng_state);

						float bsdf_pdf;
						float bsdf_value_float = phase_function::Henyey_greenstein::eval(envmap_dir.dot(ray.dir()), sss_g, bsdf_pdf);

						float visibility = direct_visibility_device(rs, nanovdb::math::Ray<float>(ray(t_max), envmap_dir), isInside, attenuation, scattering_sss, channel);

						if (envmap_pdf > 0.f) {
							float misWeight = util::powerHeuristic(envmap_pdf, bsdf_pdf);
							if(misWeight > 0.f)
								L[channel] += (envmap_value)[channel] * bsdf_value_float * visibility * throughput / envmap_pdf * misWeight;
						}
					}
#endif
					
					surface_scatter = true;
					ray.reset(ray(t_max), wi);
				}
				//  absorption
				throughput *= util::exp3f(-t_max * attenuation)[channel];
			}

			if (!sss_scatter && volume_scatter) {
				Volume* action_volume = info.all_volumes[action_volume_index];
				nanovdb::Vec3f wi;
				nanovdb::Vec3f phase_L = phase_function::Henyey_greenstein::sample(ray.dir(), wi, action_volume->g, scatterPDF, rs.rng_state);
				bsdf_value = action_volume->albedo * phase_L;

#ifdef DIRECT_LIGHTNING
				{
					nanovdb::Vec3f envmap_dir;
					float envmap_pdf;

					nanovdb::Vec3f envmap_value = rs.scene.environnement_map.sample_envmap(envmap_dir, envmap_pdf, rs.rng_state);

					float bsdf_pdf;
					float bsdf_value_float = phase_function::Henyey_greenstein::eval(envmap_dir.dot(ray.dir()), action_volume->g, bsdf_pdf);
					nanovdb::Vec3f bsdf_value_envmap = bsdf_value_float * action_volume->albedo;

					float visibility = direct_visibility_device(rs, nanovdb::math::Ray<float>(ray(t_max), envmap_dir), isInside, attenuation, scattering_sss, channel);

					if (envmap_pdf > 0.f) {
						float misWeight = util::powerHeuristic(envmap_pdf, bsdf_pdf);
						if (misWeight > 0.f)
							L[channel] += (envmap_value * bsdf_value_envmap)[channel] * visibility * throughput / envmap_pdf * misWeight;
					}
				}
#endif

				L[channel] += action_volume->emission[channel] * throughput;

				ray.reset(ray(t_max), wi);
				
				surface_scatter = true;
			}
			else if (!sss_scatter && !volume_scatter)
			{
				// account for attenuation
				if (isInside) throughput *= util::exp3f(-info.t * attenuation)[channel];

				principled_BSDF& surface_bsdf = rs.scene.bsdf_list[info.BSDF_index];

				nanovdb::Vec3f L_dir;

				bsdf_value = surface_bsdf.sample_CUDA(-ray.dir(), L_dir, info.normal, scatterPDF, rs.rng_state, through, isInside);

#ifdef DIRECT_LIGHTNING
				if(info.BSDF_index != BSDF_TROUGH_ID) {
					nanovdb::Vec3f envmap_dir;
					float envmap_pdf;

					nanovdb::Vec3f envmap_value = rs.scene.environnement_map.sample_envmap(envmap_dir, envmap_pdf, rs.rng_state);

					float bsdf_pdf = 1.f;
					float eta = isInside ? surface_bsdf.ior : 1.f / surface_bsdf.ior;
					nanovdb::Vec3f bsdf_value_envmap = surface_bsdf.eval(-ray.dir(), envmap_dir, info.normal, bsdf_pdf, eta);

					float visibility = direct_visibility_device(rs, nanovdb::math::Ray<float>(ray(info.t), envmap_dir), isInside, attenuation, scattering_sss, channel);

					if (envmap_pdf > 0.f) {
						float misWeight = util::powerHeuristic(envmap_pdf, bsdf_pdf);
						if (misWeight > 0.f)
							L[channel] += (envmap_value * bsdf_value_envmap)[channel] * visibility * throughput / envmap_pdf * misWeight;
					}
				}
#endif
				
				L[channel] += surface_bsdf.emission[channel] * throughput;

				ray.reset(ray(t_max), L_dir);
				surface_scatter = true;
				
				if (through) attenuation = surface_bsdf.absorption, scattering_sss = surface_bsdf.scattering, sss_g = surface_bsdf.g;
			}

			if (scatterPDF > 0.0001f) throughput *= bsdf_value[channel] / scatterPDF;
			else break;

			// russian roulette
			{
				float p = throughput;
				if (randC(&rs.rng_state) > p) break;
				throughput *= 1.0f / p;
			}
		}
		return L * 3.f;
	}
	__host__ nanovdb::Vec3f volume_pathtrace_host_spectral(renderer_services& rs, nanovdb::math::Ray<float> ray) {
		nanovdb::Vec3f L(0.f);
		float throughput(1.f);

		int channel = int(rand01 * 3.f);

		float scatterPDF = 1.f; // scatter pdf from last bounce, used for MIS

		// consider we are outside any volume at the beginning, this doesn't account for heterogeous volumes,
		// only for volumes encapsulated by geometry, no stack of mediums rn
		bool isInside = false;

		nanovdb::Vec3f attenuation = nanovdb::Vec3f(0.f);
		float scattering_sss = 0.f;
		float sss_g = 0.f;

		bool surface_scatter = false;

		for (unsigned int i = 0; i < BOUNCES_PT_VOL; i++) {
			hit_info info;
			info.t = MAX_DISTANCE;

			bool hit = rs.scene.intersectScene_full(ray, info);
			bool through = false;

			// fix normal
			info.normal = util::refIfNeg(info.normal, -ray.dir());

			if (!hit) {
#ifdef DIRECT_LIGHTNING
				float envmap_pdf = 1.f;
				nanovdb::Vec3f envmap_col = rs.scene.environnement_map.eval_envmap_host(ray.dir(), envmap_pdf);

				float misWeight = 1.f;
				if (i > 0)
					misWeight = util::powerHeuristic(scatterPDF, envmap_pdf);

				if (!surface_scatter)
					misWeight = 1.f;

				if (misWeight > 0.f)
					L[channel] += (misWeight * envmap_col)[channel] * throughput;
#else 
				float envmap_pdf;
				L[channel] += (rs.scene.environnement_map.eval_envmap_host(ray.dir(), envmap_pdf))[channel] * throughput;
#endif 
				break;
			}

			nanovdb::Vec3f bsdf_value;
			float t_max = info.t;

			int action_volume_index;
			bool volume_scatter = integrators::heterogeneous_volumes_integrator::integrate_spectral_host(rs, ray, t_max, info.nb_vol, info.all_volumes, action_volume_index, channel);

			// SSS integration
			bool sss_scatter = false;
			if (isInside) {
				float t_SSS = -logf(rand01) / scattering_sss;
				if (t_SSS < t_max) {
					sss_scatter = true, t_max = t_SSS;

					nanovdb::Vec3f wi;
					bsdf_value = phase_function::Henyey_greenstein::sample_host(ray.dir(), wi, sss_g, scatterPDF);

#ifdef DIRECT_LIGHTNING
					{
						nanovdb::Vec3f envmap_dir;
						float envmap_pdf;

						nanovdb::Vec3f envmap_value = rs.scene.environnement_map.sample_envmap_host(envmap_dir, envmap_pdf, rs.rng_state);

						float bsdf_pdf;
						float bsdf_value_float = phase_function::Henyey_greenstein::eval(envmap_dir.dot(ray.dir()), sss_g, bsdf_pdf);

						float visibility = direct_visibility_host(rs, nanovdb::math::Ray<float>(ray(t_max), envmap_dir), isInside, attenuation, scattering_sss, channel);

						if (envmap_pdf > 0.f) {
							float misWeight = util::powerHeuristic(envmap_pdf, bsdf_pdf);
							if (misWeight > 0.f)
								L[channel] += (envmap_value)[channel] * bsdf_value_float * visibility * throughput / envmap_pdf * misWeight;
						}
					}
#endif
					surface_scatter = true;
					ray.reset(ray(t_max), wi);
				}
				//  absorption
				throughput *= util::exp3f(-t_max * attenuation)[channel];
			}

			if (!sss_scatter && volume_scatter) {
				Volume* action_volume = info.all_volumes[action_volume_index];
				nanovdb::Vec3f wi;
				nanovdb::Vec3f phase_L = phase_function::Henyey_greenstein::sample_host(ray.dir(), wi, action_volume->g, scatterPDF);
				bsdf_value = action_volume->albedo * phase_L;

#ifdef DIRECT_LIGHTNING
				{
					nanovdb::Vec3f envmap_dir;
					float envmap_pdf;

					nanovdb::Vec3f envmap_value = rs.scene.environnement_map.sample_envmap_host(envmap_dir, envmap_pdf, rs.rng_state);

					float bsdf_pdf;
					float bsdf_value_float = phase_function::Henyey_greenstein::eval(envmap_dir.dot(ray.dir()), action_volume->g, bsdf_pdf);
					nanovdb::Vec3f bsdf_value_envmap = bsdf_value_float * action_volume->albedo;

					float visibility = direct_visibility_host(rs, nanovdb::math::Ray<float>(ray(t_max), envmap_dir), isInside, attenuation, scattering_sss, channel);

					if (envmap_pdf > 0.f) {
						float misWeight = util::powerHeuristic(envmap_pdf, bsdf_pdf);
						if (misWeight > 0.f)
							L[channel] += (envmap_value * bsdf_value_envmap)[channel] * visibility * throughput / envmap_pdf * misWeight;
					}
				}
#endif
				surface_scatter = true;
				L[channel] += action_volume->emission[channel] * throughput;

				ray.reset(ray(t_max), wi);
			}
			else if (!sss_scatter && !volume_scatter)
			{
				// account for attenuation
				if (isInside) throughput *= util::exp3f(-info.t * attenuation)[channel];

				principled_BSDF& surface_bsdf = rs.scene.bsdf_list[info.BSDF_index];

				nanovdb::Vec3f L_dir;
				bsdf_value = surface_bsdf.sample_HOST(-ray.dir(), L_dir, info.normal, scatterPDF, rs.rng_state, through, isInside);
				L[channel] += surface_bsdf.emission[channel] * throughput;

#ifdef DIRECT_LIGHTNING
				if (info.BSDF_index != BSDF_TROUGH_ID) {
					nanovdb::Vec3f envmap_dir;
					float envmap_pdf;

					nanovdb::Vec3f envmap_value = rs.scene.environnement_map.sample_envmap_host(envmap_dir, envmap_pdf, rs.rng_state);

					float bsdf_pdf = 1.f;
					float eta = isInside ? surface_bsdf.ior : 1.f / surface_bsdf.ior;
					nanovdb::Vec3f bsdf_value_envmap = surface_bsdf.eval(-ray.dir(), envmap_dir, info.normal, bsdf_pdf, eta);

					float visibility = direct_visibility_host(rs, nanovdb::math::Ray<float>(ray(info.t), envmap_dir), isInside, attenuation, scattering_sss, channel);

					if (envmap_pdf > 0.f) {
						float misWeight = util::powerHeuristic(envmap_pdf, bsdf_pdf);
						if (misWeight > 0.f)
							L[channel] += (envmap_value * bsdf_value_envmap)[channel] * visibility * throughput / envmap_pdf * misWeight;
					}
				}
#endif
				surface_scatter = true;
				ray.reset(ray(info.t), L_dir);

				if (through) attenuation = surface_bsdf.absorption, scattering_sss = surface_bsdf.scattering, sss_g = surface_bsdf.g;
			}

			if (scatterPDF > 0.0001f) throughput *= bsdf_value[channel] / scatterPDF;
			else break;

			// russian roulette
			{
				float p = throughput;
				if (rand01 > p) break;
				throughput *= 1.0f / p;
			}
		}
		return L * 3.f;
	}
}