#pragma once

namespace penguinPT {
	// single color pathtracer, each channel follows the same path
	__device__ nanovdb::Vec3f pathtrace_device(renderer_services& rs, nanovdb::math::Ray<float> ray) {
		nanovdb::Vec3f L(0.f);
		nanovdb::Vec3f throughput(1.f);

		float scatterPDF = 1.f; // scatter pdf from last bounce, used for MIS

		// consider we are outside any volume at the beginning, this doesn't account for heterogeous volumes,
		// only for volumes encapsulated by geometry
		bool isInside = false;
		bool through = false;

		for (unsigned int i = 0; i < BOUNCES_PT; i++) {
			hit_info info;
			info.t = MAX_DISTANCE;

			bool hit = rs.scene.intersectScene_full(ray, info);

			// fix normal
			info.normal = util::refIfNeg(info.normal, -ray.dir());

			if (!hit) {
				L += ray.dir().dot(nanovdb::Vec3f(1.f).normalize()) > 0.8f ? nanovdb::Vec3f(1.f, 0.8f, 0.4f) * throughput * 4.f: nanovdb::Vec3f(0.f);
				break;
			}

			principled_BSDF& surface_bsdf = rs.scene.bsdf_list[info.BSDF_index];
			nanovdb::Vec3f L_dir;
			//nanovdb::Vec3f bsdf_value = surface_bsdf.sample(ray, info.normal, L_dir, rs.rng_state, scatterPDF, isInside, through);
			nanovdb::Vec3f bsdf_value = surface_bsdf.sample_CUDA(-ray.dir(), L_dir, info.normal, scatterPDF, rs.rng_state, through, isInside);
			L += surface_bsdf.emission * throughput;
			
			ray.reset(ray(info.t), L_dir);
			util::safe(info.normal, ray);

			if (scatterPDF > 0.f) throughput = throughput * bsdf_value / scatterPDF;
			else break;
		}
		return L;
	}

	template <typename vec3T> __hostdev__ inline void safe_both(vec3T normal, nanovdb::math::Ray<float>& ray, bool through) {
		float k = through ? -1.f : 1.f;
		ray.setEye(ray.eye() + normal * SAFE_OFFSET * k);
	}
	__device__ nanovdb::Vec3f volume_pathtrace_device(renderer_services& rs, nanovdb::math::Ray<float> ray) {
		nanovdb::Vec3f L(0.f);
		nanovdb::Vec3f throughput(1.f);

		float scatterPDF = 1.f; // scatter pdf from last bounce, used for MIS

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
				nanovdb::Vec3f skyBlue = util::mix(nanovdb::Vec3f(0.4f, 0.7f, 1.f), nanovdb::Vec3f(0.9f, 0.95f, 1.f), expf(-5.f * fabsf(ray.dir()[1]))) * 0.15f;
				nanovdb::Vec3f sun = (ray.dir().dot(nanovdb::Vec3f(1.f, 0.6f, -1.f).normalize()) > 0.9f ? nanovdb::Vec3f(1.f, 0.7f, 1.f) * 2.f : nanovdb::Vec3f(0.f));
				nanovdb::Vec3f sun2 = (ray.dir().dot(nanovdb::Vec3f(1.f).normalize()) > 0.95f ? nanovdb::Vec3f(1.f) * 8.f : nanovdb::Vec3f(0.f));
				L += (sun + sun2 + skyBlue) * throughput;
				//L += nanovdb::Vec3f(ray.dir()[1] * 0.5f + 0.5f) * throughput;
				//L += fmaxf(sinf(ray.dir()[0] * 5.f) * sinf(ray.dir()[2] * 5.f), 0.f) * throughput;
				//L += util::mix(0.5f, 1.f, ray.dir()[1] * 0.5f + 0.5f) * nanovdb::Vec3f(0.8f, 0.9f, 1.f) * throughput * 0.3f;
				float pdf_e;
				//L += rs.scene.environnement_map.eval_envmap(ray.dir(), pdf_e) * throughput;
				//L += ((ray.dir()[1] > 0.90f ? 16.f : 0.f) +) * throughput;
				//L += nanovdb::Vec3f(0.5f) * throughput;
				break;
			}

			nanovdb::Vec3f bsdf_value;
			float t_max = info.t;
			
			// volume integration
			nanovdb::Vec3f volume_w;
			nanovdb::math::Ray<float> volume_ray = ray;
			nanovdb::Vec3f volume_bsdf_value;
			float volume_scatterPDF;
			bool volume_scatter = integrators::heterogeneous_volumes_integrator::integrate_device(rs, volume_ray, t_max, volume_bsdf_value, volume_scatterPDF, info.all_volumes, info.nb_vol, volume_w);

			// SSS integration
			bool sss_scatter = false;
			if (isInside) {
				float t_SSS = -logf(randC(&rs.rng_state)) / scattering_sss;
				if (t_SSS < t_max) {
					sss_scatter = true, t_max = t_SSS;

					nanovdb::Vec3f wi;
					bsdf_value = phase_function::Henyey_greenstein::sample(ray.dir(), wi, sss_g, scatterPDF, rs.rng_state);

					ray.reset(ray(t_max), wi);
				}
				//  absorption
				throughput = throughput * util::exp3f(-t_max * attenuation);
			}

			if (!sss_scatter && volume_scatter) {
				ray = volume_ray;
				throughput = throughput * volume_w;
				bsdf_value = volume_bsdf_value;
				scatterPDF = volume_scatterPDF;
			}
			else if (!sss_scatter && !volume_scatter)
			{
				// account for attenuation
				if (isInside) throughput = throughput * util::exp3f(-info.t * attenuation);

				principled_BSDF& surface_bsdf = rs.scene.bsdf_list[info.BSDF_index];
				
				nanovdb::Vec3f L_dir;
				//bsdf_value = surface_bsdf.sample(ray, info.normal, L_dir, rs.rng_state, scatterPDF, isInside, through);
				bsdf_value = surface_bsdf.sample_CUDA(-ray.dir(), L_dir, info.normal, scatterPDF, rs.rng_state, through, isInside);
				L += surface_bsdf.emission * throughput;

				ray.reset(ray(info.t), L_dir);
				//safe_both(info.normal, ray, through);

				if (through) attenuation = surface_bsdf.absorption, scattering_sss = surface_bsdf.scattering, sss_g = surface_bsdf.g;
			}

			if (scatterPDF > 0.0001f) throughput = throughput * bsdf_value / scatterPDF;
			else break;

			// russian roulette
			{
				float p = throughput.max();
				if (randC(&rs.rng_state) > p) break;
				throughput *= 1.0f / p;
			}
		}
		return L;
	}

	__device__ nanovdb::Vec3f volume_pathtrace_device_spectral(renderer_services& rs, nanovdb::math::Ray<float> ray) {
		nanovdb::Vec3f L(0.f);
		float throughput(1.f);

		int channel = int(randC(&rs.rng_state) * 3.f);

		float scatterPDF = 1.f; // scatter pdf from last bounce, used for MIS

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
				nanovdb::Vec3f skyBlue = util::mix(nanovdb::Vec3f(0.4f, 0.7f, 1.f), nanovdb::Vec3f(0.9f, 0.95f, 1.f), expf(-5.f * fabsf(ray.dir()[1]))) * 0.15f;
				nanovdb::Vec3f sun = (ray.dir().dot(nanovdb::Vec3f(1.f, 0.6f, -1.f).normalize()) > 0.9f ? nanovdb::Vec3f(1.f, 0.7f, 1.f) * 2.f : nanovdb::Vec3f(0.f));
				nanovdb::Vec3f sun2 = (ray.dir().dot(nanovdb::Vec3f(1.f).normalize()) > 0.95f ? nanovdb::Vec3f(1.f) * 8.f : nanovdb::Vec3f(0.f));
				//L[channel] += (sun + sun2 + skyBlue)[channel] * throughput;
				//L += nanovdb::Vec3f(ray.dir()[1] * 0.5f + 0.5f) * throughput;
				//L += fmaxf(sinf(ray.dir()[0] * 5.f) * sinf(ray.dir()[2] * 5.f), 0.f) * throughput;
				//L += util::mix(0.5f, 1.f, ray.dir()[1] * 0.5f + 0.5f) * nanovdb::Vec3f(0.8f, 0.9f, 1.f) * throughput * 0.3f;
				float pdf_e;
				L[channel] += (rs.scene.environnement_map.eval_envmap(ray.dir(), pdf_e))[channel] * throughput;
				//L += ((ray.dir()[1] > 0.90f ? 16.f : 0.f) +) * throughput;
				//L += nanovdb::Vec3f(0.5f) * throughput;
				break;
			}

			nanovdb::Vec3f bsdf_value;
			float t_max = info.t;

			// volume integration
			nanovdb::Vec3f volume_w;
			nanovdb::math::Ray<float> volume_ray = ray;
			nanovdb::Vec3f volume_bsdf_value;
			nanovdb::Vec3f volume_Le;
			float volume_scatterPDF;
			bool volume_scatter = integrators::heterogeneous_volumes_integrator::integrate_spectral_device(rs, volume_ray, t_max, volume_bsdf_value, volume_scatterPDF, info.all_volumes, info.nb_vol, volume_w, channel, volume_Le);

			// SSS integration
			bool sss_scatter = false;
			if (isInside) {
				float t_SSS = -logf(randC(&rs.rng_state)) / scattering_sss;
				if (t_SSS < t_max) {
					sss_scatter = true, t_max = t_SSS;

					nanovdb::Vec3f wi;
					bsdf_value = phase_function::Henyey_greenstein::sample(ray.dir(), wi, sss_g, scatterPDF, rs.rng_state);

					ray.reset(ray(t_max), wi);
				}
				//  absorption
				throughput *= util::exp3f(-t_max * attenuation)[channel];
			}

			if (!sss_scatter && volume_scatter) {
				ray = volume_ray;
				throughput *= volume_w[channel];
				bsdf_value = volume_bsdf_value;
				scatterPDF = volume_scatterPDF;

				L[channel] += volume_Le[channel] * throughput;
			}
			else if (!sss_scatter && !volume_scatter)
			{
				// account for attenuation
				if (isInside) throughput *= util::exp3f(-info.t * attenuation)[channel];

				principled_BSDF& surface_bsdf = rs.scene.bsdf_list[info.BSDF_index];

				nanovdb::Vec3f L_dir;
				bsdf_value = surface_bsdf.sample_CUDA(-ray.dir(), L_dir, info.normal, scatterPDF, rs.rng_state, through, isInside);
				L[channel] += surface_bsdf.emission[channel] * throughput;

				ray.reset(ray(info.t), L_dir);
				
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

		for (unsigned int i = 0; i < BOUNCES_PT_VOL; i++) {
			hit_info info;
			info.t = MAX_DISTANCE;

			bool hit = rs.scene.intersectScene_full(ray, info);
			bool through = false;

			// fix normal
			info.normal = util::refIfNeg(info.normal, -ray.dir());

			if (!hit) {
				nanovdb::Vec3f skyBlue = util::mix(nanovdb::Vec3f(0.4f, 0.7f, 1.f), nanovdb::Vec3f(0.9f, 0.95f, 1.f), expf(-5.f * fabsf(ray.dir()[1]))) * 0.15f;
				nanovdb::Vec3f sun = (ray.dir().dot(nanovdb::Vec3f(1.f, 0.6f, -1.f).normalize()) > 0.9f ? nanovdb::Vec3f(1.f, 0.7f, 1.f) * 2.f : nanovdb::Vec3f(0.f));
				nanovdb::Vec3f sun2 = (ray.dir().dot(nanovdb::Vec3f(1.f).normalize()) > 0.95f ? nanovdb::Vec3f(1.f) * 8.f : nanovdb::Vec3f(0.f));
				L[channel] += (sun + sun2 + skyBlue)[channel] * throughput;
				//L += nanovdb::Vec3f(ray.dir()[1] * 0.5f + 0.5f) * throughput;
				//L += fmaxf(sinf(ray.dir()[0] * 5.f) * sinf(ray.dir()[2] * 5.f), 0.f) * throughput;
				//L += util::mix(0.5f, 1.f, ray.dir()[1] * 0.5f + 0.5f) * nanovdb::Vec3f(0.8f, 0.9f, 1.f) * throughput * 0.3f;
				float pdf_e;
				//L[channel] += (rs.scene.environnement_map.eval_envmap(ray.dir(), pdf_e))[channel] * throughput;
				//L += ((ray.dir()[1] > 0.90f ? 16.f : 0.f) +) * throughput;
				//L += nanovdb::Vec3f(0.5f) * throughput;
				break;
			}

			nanovdb::Vec3f bsdf_value;
			float t_max = info.t;

			// volume integration
			nanovdb::Vec3f volume_w;
			nanovdb::math::Ray<float> volume_ray = ray;
			nanovdb::Vec3f volume_bsdf_value;
			nanovdb::Vec3f volume_Le;
			float volume_scatterPDF;
			bool volume_scatter = integrators::heterogeneous_volumes_integrator::integrate_spectral_host(rs, volume_ray, t_max, volume_bsdf_value, volume_scatterPDF, info.all_volumes, info.nb_vol, volume_w, channel, volume_Le);

			// SSS integration
			bool sss_scatter = false;
			if (isInside) {
				float t_SSS = -logf(rand01) / scattering_sss;
				if (t_SSS < t_max) {
					sss_scatter = true, t_max = t_SSS;

					nanovdb::Vec3f wi;
					bsdf_value = phase_function::Henyey_greenstein::sample_host(ray.dir(), wi, sss_g, scatterPDF);

					ray.reset(ray(t_max), wi);
				}
				//  absorption
				throughput *= util::exp3f(-t_max * attenuation)[channel];
			}

			if (!sss_scatter && volume_scatter) {
				ray = volume_ray;
				throughput *= volume_w[channel];
				bsdf_value = volume_bsdf_value;
				scatterPDF = volume_scatterPDF;

				L[channel] += volume_Le[channel] * throughput;
			}
			else if (!sss_scatter && !volume_scatter)
			{
				// account for attenuation
				if (isInside) throughput *= util::exp3f(-info.t * attenuation)[channel];

				principled_BSDF& surface_bsdf = rs.scene.bsdf_list[info.BSDF_index];

				nanovdb::Vec3f L_dir;
				bsdf_value = surface_bsdf.sample_HOST(-ray.dir(), L_dir, info.normal, scatterPDF, rs.rng_state, through, isInside);
				L[channel] += surface_bsdf.emission[channel] * throughput;

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