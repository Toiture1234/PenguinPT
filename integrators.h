#pragma once

namespace penguinPT::integrators {
	// returns a list of pointers to volumes in memory
	__hostdev__ inline Volume** get_allVolumes(nanovdb::Vec3f position, renderer_services& rs, int& nb_vol);
	class heterogeneous_volumes_integrator {
	public:
		__device__ static bool integrate(renderer_services& rs, nanovdb::Ray<float>& ray, float& t, nanovdb::Vec3f& BSDF_value, float& scatter_pdf, Volume** volumes, int nb_vol, nanovdb::Vec3f& throughput);
	};

	__device__ bool heterogeneous_volumes_integrator::integrate(renderer_services& rs, nanovdb::Ray<float>& ray, float& t_out, nanovdb::Vec3f& BSDF_value, float& scatter_pdf, Volume** volumes, int nb_vol, nanovdb::Vec3f& throughput) {
		// t_out is considered as t_max
		float cdf[VOLUME_STACK_SIZE];
		
		if (nb_vol == 0) return false;
		float summed_sigma_t = 0.f;

		for (int i = 0; i < nb_vol; i++) {
			summed_sigma_t += volumes[i]->sigma_t.max();
			cdf[i] = summed_sigma_t;
		}
		for (int i = 0; i < nb_vol; i++) cdf[i] /= summed_sigma_t;
		
		float inv_sigma_T = 1.f / summed_sigma_t;
		
		float t = 0;
		throughput = nanovdb::Vec3f(1.f);

		for (int i = 0; i < DT_SAMPLES; i++) {
			t -= logf(randC(&rs.rng_state)) * inv_sigma_T;

			if (t > t_out) return false;

			float density = 0.f;
			float zeta = randC(&rs.rng_state) * nb_vol;
			for (int index = 0; index < nb_vol; index++) {
				Volume* current = volumes[index];
				float density_v = current->get_density(ray(t), nanovdb::Vec3f(0.f));
				density += density_v;

				if (zeta < density) {
					nanovdb::Vec3f wi;
					nanovdb::Vec3f phase_L = phase_function::Henyey_greenstein::sample(ray.dir(), wi, current->g, scatter_pdf, rs.rng_state);

					BSDF_value = current->albedo * phase_L;
					ray.reset(ray(t), wi);
					t_out = t;
					return true;
				}
				
			}
		}
		
		return false;
	}
}

__hostdev__ inline penguinPT::Volume** penguinPT::integrators::get_allVolumes(nanovdb::Vec3f position, renderer_services& rs, int& nb_vol) {
	Volume* volume_stack[VOLUME_STACK_SIZE];
	
	nb_vol = 0;
	for (int i = 0; i < rs.scene.num_of_volumes; i++) {
		Volume* current = &rs.scene.volumes[i];
		if (current->is_point_inside(position)) {
			volume_stack[nb_vol] = current;
			nb_vol++;
		}
	}
	return volume_stack;
}