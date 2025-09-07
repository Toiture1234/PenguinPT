#pragma once

namespace penguinPT {
	// single color pathtracer, each channel follows the same path
	__device__ inline nanovdb::Vec3f pathtrace_device(renderer_services& rs, nanovdb::math::Ray<float> ray) {
		nanovdb::Vec3f L(0.f);
		nanovdb::Vec3f throughput(1.f);

		float scatterPDF = 1.f; // scatter pdf from last bounce, used for MIS

		// consider we are outside any volume at the beginning, this doesn't account for heterogeous volumes,
		// only for volumes encapsulated by geometry
		bool isInside = false;

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

			BSDF& surface_bsdf = rs.scene.bsdf_list[info.BSDF_index];
			nanovdb::Vec3f L_dir;
			nanovdb::Vec3f bsdf_value = surface_bsdf.sample(ray, info.normal, L_dir, rs.rng_state, scatterPDF);
			L += surface_bsdf.emission * throughput;
			
			ray.reset(ray(info.t), L_dir);
			util::safe(info.normal, ray);

			if (scatterPDF > 0.f) throughput = throughput * bsdf_value / scatterPDF;
			else break;
		}
		return L;
	}

	template <typename vec3T> __hostdev__ inline void safe_both(vec3T normal, nanovdb::math::Ray<float>& ray, int bsdf_t) {
		float k = (bsdf_t == BSDF_trough) || (bsdf_t == BSDF_glass) ? -1.f : 1.f;
		ray.setEye(ray.eye() + normal * SAFE_OFFSET * k);
	}
	__device__ inline nanovdb::Vec3f volume_pathtrace_device(renderer_services& rs, nanovdb::math::Ray<float> ray) {
		nanovdb::Vec3f L(0.f);
		nanovdb::Vec3f throughput(1.f);

		float scatterPDF = 1.f; // scatter pdf from last bounce, used for MIS

		// consider we are outside any volume at the beginning, this doesn't account for heterogeous volumes,
		// only for volumes encapsulated by geometry
		bool isInside = false;

		for (unsigned int i = 0; i < BOUNCES_PT_VOL; i++) {
			hit_info info;
			info.t = MAX_DISTANCE;

			bool hit = rs.scene.intersectScene_full(ray, info);

			// fix normal
			info.normal = util::refIfNeg(info.normal, -ray.dir());

			if (!hit) {
				L += ray.dir().dot(nanovdb::Vec3f(1.f).normalize()) > 0.8f ? nanovdb::Vec3f(1.f) * throughput * 4.f : nanovdb::Vec3f(0.f);
				//L += nanovdb::Vec3f(ray.dir()[1] * 0.5f + 0.5f) * throughput;
				//L += fmaxf(sinf(ray.dir()[0] * 5.f) * sinf(ray.dir()[2] * 5.f), 0.f) * throughput;
				L += util::mix(0.f, 0.5f, ray.dir()[1] * 0.5f + 0.5f) * nanovdb::Vec3f(0.8f, 0.9f, 1.f) * throughput;
				break;
			}

			// check for volume intersection
			nanovdb::Vec3f bsdf_value;
			bool interact_volume = false;
			Volume& this_volume = rs.scene.volumes[info.volumeIndex];
			if (info.insideVolume) {
				interact_volume = this_volume.delta_tracking(ray, info.t, rs.rng_state);
			}

			if (interact_volume) {
				nanovdb::Vec3f L_dir = util::generateUniformSample(rs.rng_state);
				
				scatterPDF = INV_4_PI;
				bsdf_value = nanovdb::Vec3f(scatterPDF) * this_volume.albedo;

				ray.reset(ray(info.t), L_dir);
			}
			else {
				BSDF& surface_bsdf = rs.scene.bsdf_list[info.BSDF_index];
				nanovdb::Vec3f L_dir;
				bsdf_value = surface_bsdf.sample(ray, info.normal, L_dir, rs.rng_state, scatterPDF);
				L += surface_bsdf.emission * throughput;

				ray.reset(ray(info.t), L_dir);
				safe_both(info.normal, ray, surface_bsdf.bsdf_type);
			}

			if (scatterPDF > 0.f) throughput = throughput * bsdf_value / scatterPDF;
			else break;
		}
		return L;
	}
}