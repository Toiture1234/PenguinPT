// Copyright 2026 Toiture1234
// 
// SPDX-License-Identifier : MIT
#pragma once

#include <ppt/util/options.h>
#include <ppt/util/Utility.h>

namespace penguinPT::Microfacet {
	class Microfacet_distribubtion {
	public:
		__hostdev__ virtual float D(const nanovdb::Vec3f& wh) const = 0;
		__hostdev__ virtual float lambda(const nanovdb::Vec3f& w) const = 0;

		__hostdev__ float G1(const nanovdb::Vec3f& w) const {
			return 1.f / (1.f + lambda(w));
		}
		__hostdev__ float G(const nanovdb::Vec3f& wo, const nanovdb::Vec3f& wi) const {
			return 1.f / (1.f + lambda(wo) + lambda(wi));
		}

		__hostdev__ nanovdb::Vec3f sample_wh(Rand_state& rngState) const {
			return { 0.f, 0.f, 0.f };
		}
		__hostdev__ nanovdb::Vec3f sample_wh_host() const {
			return { 0.f, 0.f, 0.f };
		}

		__hostdev__ float PDF(const nanovdb::Vec3f& wh, const nanovdb::Vec3f& wo) const {
			return D(wh) * G1(wo) * fabsf(wh.dot(wo)) / fabsf(util::CosTheta(wo));
		}
	};


	class Beckmann_distribution : public Microfacet_distribubtion {
	public:
		__hostdev__ static float roughness_to_alpha(float r) {
			r = fmaxf(r, 1e-3f);
			float x = logf(r);
			return 1.62142f + 0.819955f * x + 0.1734f * x * x + 0.0171201f * x * x * x + 0.000640711f * x * x * x * x;
		}

		__device__ nanovdb::Vec3f sample_wh(Rand_state& rngState) const;
		__host__ nanovdb::Vec3f sample_wh_host() const;

		__hostdev__ float D(const nanovdb::Vec3f& wh) const;

		__hostdev__ Beckmann_distribution(float ax, float ay) : alphax(ax), alphay(ay) {}
		__hostdev__ ~Beckmann_distribution() {}

	private:
		float alphax, alphay;

		__hostdev__ float lambda(const nanovdb::Vec3f& w) const;
	};

	__hostdev__ float Beckmann_distribution::D(const nanovdb::Vec3f& wh) const {
		float tan2Theta = util::Tan2Theta(wh);
		if (isinf(tan2Theta)) return 0.f;
		float cos4Theta = util::Cos2Theta(wh) * util::Cos2Theta(wh);
		return expf(-tan2Theta * (util::Cos2Phi(wh) / (alphax * alphax) +
			util::Sin2Phi(wh) / (alphay * alphay))) / (PI * alphax * alphay * cos4Theta);
	}
	__hostdev__ float Beckmann_distribution::lambda(const nanovdb::Vec3f& w) const {
		float absTanTheta = fabsf(util::TanTheta(w));
		if (isinf(absTanTheta)) return 0.f;

		float alpha = sqrtf(util::Cos2Phi(w) * alphax * alphax + util::Sin2Phi(w) * alphay * alphay);

		float a = 1.f / (alpha * absTanTheta);
		if (a >= 1.6f) return 0.f;
		return (1.f - 1.259f * a + 0.396f * a * a) / (3.535f * a + 2.181f * a * a);
	}

	__device__ nanovdb::Vec3f Beckmann_distribution::sample_wh(Rand_state& rngState) const {
		float tan2Theta, phi;
		float u1 = randC(&rngState);
		float u2 = randC(&rngState);

		if (alphax == alphay) {
			float logSample = logf(1.f - u1);
			if (isinf(logSample)) logSample = 0.f;
			tan2Theta = -alphax * alphax * logSample;
			phi = u2 * TWO_PI;
		}
		else {
			float logSample = logf(u1);
			phi = atanf(alphay / alphax * tanf(TWO_PI * u2 + 0.5f * PI));
			if (u2 > 0.f) phi += PI;

			float sinPhi = sinf(phi), cosPhi = cosf(phi);
			float alphax2 = alphax * alphax, alphay2 = alphay * alphay;
			tan2Theta = -logSample / (cosPhi * cosPhi / alphax2 + sinPhi * sinPhi / alphay2);
		}
		float cosTheta = 1.f / sqrtf(1.f + tan2Theta);
		float sinTheta = sqrtf(fmaxf(1.f - cosTheta * cosTheta, 0.f));

		nanovdb::Vec3f wh = nanovdb::Vec3f(sinTheta * cosf(phi), sinTheta * sinf(phi), cosTheta);
		wh *= SIGN(wh[2]);
		return wh;
	}
	__host__ nanovdb::Vec3f Beckmann_distribution::sample_wh_host() const {
		float tan2Theta, phi;
		float u1 = rand01;
		float u2 = rand01;

		if (alphax == alphay) {
			float logSample = logf(1.f - u1);
			if (isinf(logSample)) logSample = 0.f;
			tan2Theta = -alphax * alphax * logSample;
			phi = u2 * TWO_PI;
		}
		else {
			float logSample = logf(u1);
			phi = atanf(alphay / alphax * tanf(TWO_PI * u2 + 0.5f * PI));
			if (u2 > 0.5f) phi += PI;

			float sinPhi = sinf(phi), cosPhi = cosf(phi);
			float alphax2 = alphax * alphax, alphay2 = alphay * alphay;
			tan2Theta = -logSample / (cosPhi * cosPhi / alphax2 + sinPhi * sinPhi / alphay2);
		}
		float cosTheta = 1.f / sqrtf(1.f + tan2Theta);
		float sinTheta = sqrtf(fmaxf(1.f - cosTheta * cosTheta, 0.f));

		nanovdb::Vec3f wh = nanovdb::Vec3f(sinTheta * cosf(phi), sinTheta * sinf(phi), cosTheta);
		wh *= SIGN(wh[2]);
		return wh;
	}

	// fresnel 
	__hostdev__ float fresnel_dielectric(float cosThetaI, float etaI, float etaT) {
		cosThetaI = CLAMP(cosThetaI, -1.f, 1.f);

		float sinThetaI = sqrtf(fmaxf(1.f - cosThetaI * cosThetaI, 0.f));
		float sinThetaT = etaI / etaT * sinThetaI;
		float cosThetaT = sqrtf(fmaxf(1.f - sinThetaT * sinThetaT, 0.f));

		float Rparl = ((etaT * cosThetaI) - (etaI * cosThetaT)) / ((etaT * cosThetaI) + (etaI * cosThetaT));
		float Rperl = ((etaI * cosThetaI) - (etaT * cosThetaT)) / ((etaI * cosThetaI) + (etaT * cosThetaT));
		return (Rparl * Rparl + Rperl + Rperl) * 0.5f;
	}
	class Phong_distribution {
	public:
		__hostdev__ Phong_distribution() {}
		__hostdev__ Phong_distribution(float roughness) { alpha = fmaxf(roughness * roughness, 0.05f); power = 1.f / (alpha * alpha) + 9.f; }
		__hostdev__ ~Phong_distribution() {}

		__hostdev__ float D(const nanovdb::Vec3f& wh) const;
		__hostdev__ float PDF(const nanovdb::Vec3f& wh, const nanovdb::Vec3f& wo) const;

		__device__ nanovdb::Vec3f sample_wh(Rand_state& state) const;
		__host__ nanovdb::Vec3f sample_wh_host() const;

		float alpha;
		float power;
	};

	__hostdev__ float Phong_distribution::D(const nanovdb::Vec3f& wh) const {
		return (power + 2.f) * INV_TWO_PI * powf(wh[2], power);
	}
	__hostdev__ float Phong_distribution::PDF(const nanovdb::Vec3f& wh, const nanovdb::Vec3f& wo) const {
		return (power + 1.f) * INV_TWO_PI * powf(wh[2], power);
	}
	__device__ nanovdb::Vec3f Phong_distribution::sample_wh(Rand_state& state) const {
		float u1 = randC(&state), u2 = randC(&state);

		float cos_theta = powf(u1, 1.f / (1.f + power));
		float sin_theta = sqrtf(fmaxf(0.f, 1.f - cos_theta * cos_theta));

		float phi = TWO_PI * u2;

		return nanovdb::Vec3f(sin_theta * cosf(phi), sin_theta * sinf(phi), cos_theta);
	}
	__host__ nanovdb::Vec3f Phong_distribution::sample_wh_host() const {
		float u1 = rand01, u2 = rand01;

		float cos_theta = powf(u1, 1.f / (1.f + power));
		float sin_theta = sqrtf(fmaxf(0.f, 1.f - cos_theta * cos_theta));

		float phi = TWO_PI * u2;

		return nanovdb::Vec3f(sin_theta * cosf(phi), sin_theta * sinf(phi), cos_theta);
	}
}