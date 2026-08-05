// Copyright 2026 Toiture1234
// 
// SPDX-License-Identifier : MIT

#pragma once

#include <ppt/util/options.h>
#include <ppt/util/Utility.h>
#include <ppt/core/intersectors.h>
#include <ppt/core/renderer_services.h>

#define RAYMARCH_STEPS 50U
#define PREVIEW_MAX_DEPTH 5U

namespace penguinPT {
	__device__ nanovdb::Vec3f previewRenderCUDA(nanovdb::math::Ray<float> ray, renderer_services& rs);
	__host__ nanovdb::Vec3f previewRenderHOST(nanovdb::math::Ray<float> ray, renderer_services& rs);

	// assume we are already inside the volume
	__hostdev__ nanovdb::Vec3f volumeTransmittance(nanovdb::math::Ray<float> ray, float t_max, int nb_vol, Volume** vol_list);
}

__device__ nanovdb::Vec3f penguinPT::previewRenderCUDA(nanovdb::math::Ray<float> ray, renderer_services& rs) {
	nanovdb::Vec3f Tr = { 1,1,1 };

	for (unsigned int i = 0; i < PREVIEW_MAX_DEPTH; i++) {
		hit_info info;
		info.t = 1e10f;

		if (!rs.scene.intersectScene(ray, info)) break;

		Tr = Tr * volumeTransmittance(ray, info.t, info.nb_vol, info.all_volumes);

		if (info.BSDF_index != BSDF_TROUGH_ID) { // solid : end here (valable for glass too)
			principled_BSDF& surface_bsdf = rs.scene.bsdf_list[info.BSDF_index];

			// normal modification
			nanovdb::Vec3f T, B;
			util::Onb(info.normal, T, B);
			info.normal = util::ToWorld(T, B, info.normal, surface_bsdf.getNormal_textured_CUDA(info.uv) * 2.f - nanovdb::Vec3f(1.f));

			info.normal = util::refIfNeg(info.normal, -ray.dir());

			float temp_pdf = 1.f;
			nanovdb::Vec3f A_term = surface_bsdf.eval_CUDA(-ray.dir(), { 0, 1, 0 }, info.normal, temp_pdf, info.uv, 1.f / surface_bsdf.ior);
			nanovdb::Vec3f C_term = surface_bsdf.eval_CUDA(-ray.dir(), nanovdb::Vec3f(-1, -1, 1).normalize(), info.normal, temp_pdf, info.uv, 1.f / surface_bsdf.ior);
			return (A_term + C_term * 0.5f) * Tr;
		}
		else {
			ray.setEye(ray(info.t));
		}
	}
	float pdf_e;
	return (rs.scene.environnement_map.eval_envmap(ray.dir(), pdf_e)) * Tr;
}
__host__ nanovdb::Vec3f penguinPT::previewRenderHOST(nanovdb::math::Ray<float> ray, renderer_services& rs) {
	nanovdb::Vec3f Tr = { 1,1,1 };

	for (unsigned int i = 0; i < PREVIEW_MAX_DEPTH; i++) {
		hit_info info;
		info.t = 1e10f;

		if (!rs.scene.intersectScene(ray, info)) break;

		Tr = Tr * volumeTransmittance(ray, info.t, info.nb_vol, info.all_volumes);

		if (info.BSDF_index != BSDF_TROUGH_ID) { // solid : end here (valable for glass too)
			principled_BSDF& surface_bsdf = rs.scene.bsdf_list[info.BSDF_index];

			// normal modification
			nanovdb::Vec3f T, B;
			util::Onb(info.normal, T, B);
			info.normal = util::ToWorld(T, B, info.normal, surface_bsdf.getNormal_textured_HOST(info.uv) * 2.f - nanovdb::Vec3f(1.f));

			info.normal = util::refIfNeg(info.normal, -ray.dir());

			float temp_pdf = 1.f;
			nanovdb::Vec3f A_term = surface_bsdf.eval_HOST(-ray.dir(), { 0, 1, 0 }, info.normal, temp_pdf, info.uv, 1.f / surface_bsdf.ior);
			nanovdb::Vec3f C_term = surface_bsdf.eval_HOST(-ray.dir(), nanovdb::Vec3f(-1, -1, 1).normalize(), info.normal, temp_pdf, info.uv, 1.f / surface_bsdf.ior);
			return (A_term + C_term * 0.5f) * Tr;
		}
		else {
			ray.setEye(ray(info.t));
		}
	}
	float pdf_e;
	return (rs.scene.environnement_map.eval_envmap_host(ray.dir(), pdf_e)) * Tr;
}
__hostdev__ nanovdb::Vec3f penguinPT::volumeTransmittance(nanovdb::math::Ray<float> ray, float t_max, int nb_vol, Volume** vol_list) {
	nanovdb::Vec3f T = { 1,1,1 };

	if (nb_vol == 0) return T;

	float step_length = t_max / (float)RAYMARCH_STEPS;
	for (unsigned int i = 0; i < RAYMARCH_STEPS; i++) {

		nanovdb::Vec3f p = ray.eye() + ray.dir() * (float)i * step_length;

		nanovdb::Vec3f density = { 0,0,0 };
		for (int i = 0; i < nb_vol; i++) {
			Volume* current_volume = vol_list[i];
			density += current_volume->get_density(p, nanovdb::Vec3f(0.f)) * current_volume->sigma_t;
		}

		T = T * util::exp3f(-density * step_length);
	}

	return T;
}