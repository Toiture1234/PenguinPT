// Copyright 2026 Toiture1234
// 
// SPDX-License-Identifier : MIT

#pragma once

#include <ppt/util/options.h>
#include <ppt/util/Utility.h>

namespace penguinPT::phase_function {
	class Henyey_greenstein {
	public:
		__hostdev__ static float eval(const float mu, const float& g, float& pdf);

		__device__ static nanovdb::Vec3f sample(const nanovdb::Vec3f& w0, nanovdb::Vec3f& wi, const float& g, float& pdf, Rand_state& rng_state);
		__host__ static nanovdb::Vec3f sample_host(const nanovdb::Vec3f& w0, nanovdb::Vec3f& wi, const float& g, float& pdf);
	};
}

__hostdev__ float penguinPT::phase_function::Henyey_greenstein::eval(const float mu, const float& g, float& pdf) {
	pdf = INV_4_PI * (1. - g * g) / (powf(1. + g * g - 2.0f * g * mu, 1.5f));
	return pdf;
}
__device__ nanovdb::Vec3f penguinPT::phase_function::Henyey_greenstein::sample(const nanovdb::Vec3f& w0, nanovdb::Vec3f& wi, const float& g, float& pdf, Rand_state& rng_state) {
	if (g == 0) return util::generateUniformSample(rng_state);
	
	float xi = randC(&rng_state);
	float t = (1.f - g * g) / (1.f - g + 2.0f * g * xi);
	float mu = (0.5f / g) * ((1.f + g * g) - t * t);

	float phi = TWO_PI * randC(&rng_state);
	float sinTheta = sqrtf(fmaxf(0.f, 1.f - mu * mu));

	nanovdb::Vec3f T, B;
	util::Onb(w0, T, B);
	wi = (sinTheta * cosf(phi) * T + sinTheta * sinf(phi) * B + mu * w0).normalize();

	return nanovdb::Vec3f(eval(mu, g, pdf));
}
__host__ nanovdb::Vec3f penguinPT::phase_function::Henyey_greenstein::sample_host(const nanovdb::Vec3f& w0, nanovdb::Vec3f& wi, const float& g, float& pdf) {
	float xi = rand01;
	float t = (1.f - g * g) / (1.f - g + 2.0f * g * xi);
	float mu = (0.5f / g) * ((1.f + g * g) - t * t);

	float phi = TWO_PI * rand01;
	float sinTheta = sqrtf(fmaxf(0.f, 1.f - mu * mu));

	nanovdb::Vec3f T, B;
	util::Onb(w0, T, B);
	wi = (sinTheta * cosf(phi) * T + sinTheta * sinf(phi) * B + mu * w0).normalize();

	return nanovdb::Vec3f(eval(mu, g, pdf));
}