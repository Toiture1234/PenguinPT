#pragma once

#define POW_ROUGHNESS(r) ((r) * (r) * (r) * (r) * (r) * (r))

namespace penguinPT {
	// one sample BSDF type, blend between differents models
	class principled_BSDF {
	public:
		// material data
		float roughness;
		float anisotropy;
		float metalness;
		float ior;
		float transparency; // [0 ; 1] : 1 -> fully transparent, 0 -> fully opaque
		float scattering;

		// for paper-like materials
		float g;
		float opacity;

		nanovdb::Vec3f albedo;
		nanovdb::Vec3f emission;
		nanovdb::Vec3f absorption;

		bool is_through;
		bool change_medium;

		__hostdev__ principled_BSDF() : roughness(0.f), anisotropy(0.), metalness(0.), ior(1.f), g(0.f), opacity(0.f), albedo(0.f), emission(0.f), absorption(0.f), is_through(false), transparency(0.f), scattering(0.f), change_medium(true) {}
		__hostdev__ ~principled_BSDF() {}

		__hostdev__ nanovdb::Vec3f eval(nanovdb::Vec3f wo, nanovdb::Vec3f wi, nanovdb::Vec3f N, float& pdf, float eta) const;

		__device__ nanovdb::Vec3f sample_CUDA(nanovdb::Vec3f wo, nanovdb::Vec3f& wi, nanovdb::Vec3f N, float& pdf, Rand_state& rng_state, bool& through, bool& isInside) const;
		__host__ nanovdb::Vec3f sample_HOST(nanovdb::Vec3f wo, nanovdb::Vec3f& wi, nanovdb::Vec3f N, float& pdf, Rand_state& rng_state, bool& through, bool& isInside) const;
		bool __hostdev__ operator==(const principled_BSDF&);
	};
	bool __hostdev__ principled_BSDF::operator==(const principled_BSDF& other) {
		return other.roughness == this->roughness &&
			other.absorption == this->absorption &&
			other.albedo == this->albedo &&
			other.anisotropy == this->anisotropy &&
			other.emission == this->emission &&
			other.g == this->g &&
			other.ior == this->ior &&
			other.is_through == this->is_through &&
			other.metalness == this->metalness &&
			other.transparency == this->transparency &&
			other.opacity == this->opacity &&
			other.change_medium == this->change_medium;
	}
	
	__device__ inline nanovdb::Vec3f sample_phong_specular(nanovdb::Vec3f wi_perfect, float n, Rand_state& rng_state) {
		float u1 = randC(&rng_state);
		float u2 = randC(&rng_state);

		float phi = TWO_PI * u2;
		float cos_alpha = powf(u1, 1.f / (n + 1.f));
		float sin_alpha = sqrtf(1.f - cos_alpha * cos_alpha);

		nanovdb::Vec3f T, B;
		util::Onb(wi_perfect, T, B);

		return (sin_alpha * cosf(phi) * T + sin_alpha * sinf(phi) * B + cos_alpha * wi_perfect).normalize();
	}
	__host__ inline nanovdb::Vec3f sample_phong_specular_host(nanovdb::Vec3f wi_perfect, float n, Rand_state& rng_state) {
		float u1 = rand01;
		float u2 = rand01;

		float phi = TWO_PI * u2;
		float cos_alpha = powf(u1, 1.f / (n + 1.f));
		float sin_alpha = sqrtf(1.f - cos_alpha * cos_alpha);

		nanovdb::Vec3f T, B;
		util::Onb(wi_perfect, T, B);

		return (sin_alpha * cosf(phi) * T + sin_alpha * sinf(phi) * B + cos_alpha * wi_perfect).normalize();
	}
	__hostdev__ nanovdb::Vec3f principled_BSDF::eval(nanovdb::Vec3f wo, nanovdb::Vec3f wi, nanovdb::Vec3f N, float& pdf, float eta) const {
		if (is_through) {
			float k = wo.dot(wi);
			pdf = k == 1.f ? 1.f : 0.f;
			return nanovdb::Vec3f(pdf);
		}

		auto pow5 = [](float v) {return (v * v) * (v * v) * v; };

		// convert to local space
		nanovdb::Vec3f T, B;
		util::Onb(N, T, B);

		wo = util::ToLocal(T, B, N, wo), wi = util::ToLocal(T, B, N, wi);
		nanovdb::Vec3f wh;
		if(wo[2] > 0.f) wh = (wo + wi).normalize();
		else wh = (wo + wi * eta).normalize();

		if (wh[2] < 0.f) wh *= -1.f;

		float aspect = sqrtf(1.f - 0.9f * anisotropy);
		float ax = Microfacet::Beckmann_distribution::roughness_to_alpha(roughness * roughness * aspect);
		float ay = Microfacet::Beckmann_distribution::roughness_to_alpha(roughness * roughness / aspect);
		Microfacet::Beckmann_distribution distribution(ax, ay);

		// models weight
		float f0 = 0.2f;
		float f90 = 1.f;

		float schlickWt = util::SchlickWeight(wo[2]);

		float dielectric_W = (1.f - metalness) * (1.f - transparency);
		float metal_W = metalness;
		float glass_W = (1.f - metalness) * transparency;
		
		float diffuse_pr = dielectric_W * util::luminance(albedo);
		float specular_pr = dielectric_W * util::mix(f0, f90, schlickWt);
		float metal_pr = metal_W * util::luminance(util::mix(albedo, nanovdb::Vec3f(1.f), schlickWt));
		float glass_pr = glass_W;

		float invTotalWt = 1.f / (diffuse_pr + specular_pr + metal_pr + glass_pr);
		diffuse_pr *= invTotalWt;
		specular_pr *= invTotalWt;
		metal_pr *= invTotalWt;
		glass_pr *= invTotalWt;

		bool reflect = wi[2] * wo[2] > 0;
		nanovdb::Vec3f L(0.f);
		pdf = 0.f;

		if (diffuse_pr > 0.f && reflect) {
			float diff_pdf = INV_PI * fabsf(util::CosTheta(wi));
			nanovdb::Vec3f diffuse = albedo * INV_PI * fabsf(util::CosTheta(wi));

			L += diffuse * dielectric_W;
			pdf += diff_pdf * diffuse_pr;
		}
		if (specular_pr > 0.f && reflect) {
			float fresnel = f0 + pow5(1.f - wi.dot(wh)) * (1.f - f0);
			float cosThetaI = fabsf(util::CosTheta(wi)), cosThetaO = fabsf(util::CosTheta(wo));
			nanovdb::Vec3f specular = distribution.D(wh) * fresnel / (4.f * fabsf(wi.dot(wh)) * fmaxf(cosThetaI, cosThetaO)) * nanovdb::Vec3f(dielectric_W);
			float H_pdf = distribution.PDF(wh, wo) / (4.f * wo.dot(wh));

			L += specular;
			pdf += H_pdf * specular_pr;
		}
		if (metal_pr > 0.f && reflect) {
			float cosThetaI = fabsf(util::CosTheta(wi)), cosThetaO = fabsf(util::CosTheta(wo));
			nanovdb::Vec3f specular = albedo * distribution.D(wh) / (4.f * fabsf(wi.dot(wh)) * fmaxf(cosThetaI, cosThetaO));
			float H_pdf = distribution.PDF(wh, wo) / (4.f * wo.dot(wh));

			L += specular * metal_W;
			pdf += H_pdf * metal_pr;
		}
		if (glass_pr > 0.f) {
			float reflect_prob = util::mix(f0, f90, schlickWt);
			float exponent = 1.f + 10000.f * POW_ROUGHNESS(1.f - roughness);
			
			if (reflect) {
				float cosThetaI = fabsf(util::CosTheta(wi)), cosThetaO = fabsf(util::CosTheta(wo));
				nanovdb::Vec3f specular = nanovdb::Vec3f(1.f) * distribution.D(wh) / (4.f * fabsf(wi.dot(wh)) * fmaxf(cosThetaI, cosThetaO)) * glass_W;
				float H_pdf = distribution.PDF(wh, wo) / (4.f * wo.dot(wh));

				L += specular;
				pdf += H_pdf * glass_pr * reflect_prob;
			}
			else {
				nanovdb::Vec3f perfect_refraction = util::refract(-wo, nanovdb::Vec3f(0.f, 0.f, 1.f), eta);
				float cosAlpha = CLAMP(perfect_refraction.dot(wi), 0.f, 1.f);
				float p_cosAlpha = powf(cosAlpha, exponent) * (exponent + 1.f) * INV_TWO_PI;

				pdf += p_cosAlpha * (1. - reflect_prob) * glass_pr;
				L += nanovdb::Vec3f(p_cosAlpha) * glass_W;
			}
		}

		return L /* fabsf(wi[2])*/;
	}

	__device__ nanovdb::Vec3f principled_BSDF::sample_CUDA(nanovdb::Vec3f wo, nanovdb::Vec3f& wi, nanovdb::Vec3f N, float& pdf, Rand_state& rng_state, bool& through, bool& isInside) const {
		if (is_through) {
			wi = -wo;
			pdf = 1.f;
			through = true;
			return nanovdb::Vec3f(1.f);
		}
		through = false;

		float eta = isInside ? ior : 1.f / ior;

		auto pow5 = [](float v) {return (v * v) * (v * v) * v; };

		// convert to local space
		nanovdb::Vec3f T, B;
		util::Onb(N, T, B);

		wo = util::ToLocal(T, B, N, wo), wi = util::ToLocal(T, B, N, wi);
		nanovdb::Vec3f wh = (wo + wi).normalize();

		float aspect = sqrtf(1.f - 0.9f * anisotropy);
		float ax = Microfacet::Beckmann_distribution::roughness_to_alpha(roughness * roughness * aspect);
		float ay = Microfacet::Beckmann_distribution::roughness_to_alpha(roughness * roughness / aspect);
		Microfacet::Beckmann_distribution distribution(ax, ay);

		// models weight
		float f0 = 0.2f;
		float f90 = 1.f;

		float schlickWt = util::SchlickWeight(wo[2]);

		float dielectric_W = (1.f - metalness) * (1.f - transparency);
		float metal_W = metalness;
		float glass_W = (1.f - metalness) * transparency;

		float diffuse_pr = dielectric_W * util::luminance(albedo);
		float specular_pr = dielectric_W * util::mix(f0, f90, schlickWt);
		float metal_pr = metal_W * util::luminance(util::mix(albedo, nanovdb::Vec3f(1.f), schlickWt));
		float glass_pr = glass_W;

		float invTotalWt = 1.f / (diffuse_pr + specular_pr + metal_pr + glass_pr);
		diffuse_pr *= invTotalWt;
		specular_pr *= invTotalWt;
		metal_pr *= invTotalWt;
		glass_pr *= invTotalWt;

		float zeta = randC(&rng_state);

		float u1 = randC(&rng_state);
		float u2 = randC(&rng_state);

		pdf = 0.f;
		
		// diffuse
		if (zeta < diffuse_pr) {
			wi = util::cosineSampleHemisphere(u1, u2);
		}
		// specular + metallic
		else if(zeta < diffuse_pr + specular_pr + metal_pr) {
			nanovdb::Vec3f wh = distribution.sample_wh(rng_state);
			if (wh[2] < 0.f) wh *= -1.f;

			wi = util::reflect(-wo, wh);
		}
		// specular refraction
		else {
			float reflect_prob = util::mix(f0, f90, schlickWt);
			float exponent = 1.f + 10000.f * POW_ROUGHNESS(1.f - roughness);

			nanovdb::Vec3f refracted = util::refract(-wo, nanovdb::Vec3f(0.f, 0.f, 1.f), eta);

			if (randC(&rng_state) < reflect_prob || refracted.dot(refracted) == 0.f) {
				//wi_perfect = util::reflect(-wo, nanovdb::Vec3f(0.f, 0.f, 1.f));
				through = false;
				nanovdb::Vec3f wh = distribution.sample_wh(rng_state);
				if (wh[2] < 0.f) wh *= -1.f;

				wi = util::reflect(-wo, wh);
			}
			else {
				if(change_medium) isInside = !isInside;
				through = true;
				wi = sample_phong_specular(refracted, exponent, rng_state);
			}
		}

		wo = util::ToWorld(T, B, N, wo);
		wi = util::ToWorld(T, B, N, wi);

		return eval(wo, wi, N, pdf, eta);
	}
	__host__ nanovdb::Vec3f principled_BSDF::sample_HOST(nanovdb::Vec3f wo, nanovdb::Vec3f& wi, nanovdb::Vec3f N, float& pdf, Rand_state& rng_state, bool& through, bool& isInside) const {
		if (is_through) {
			wi = -wo;
			pdf = 1.f;
			through = true;
			return nanovdb::Vec3f(1.f);
		}
		through = false;

		float eta = isInside ? ior : 1.f / ior;

		auto pow5 = [](float v) {return (v * v) * (v * v) * v; };

		// convert to local space
		nanovdb::Vec3f T, B;
		util::Onb(N, T, B);

		wo = util::ToLocal(T, B, N, wo), wi = util::ToLocal(T, B, N, wi);
		nanovdb::Vec3f wh = (wo + wi).normalize();

		float aspect = sqrtf(1.f - 0.9f * anisotropy);
		float ax = Microfacet::Beckmann_distribution::roughness_to_alpha(roughness * roughness * aspect);
		float ay = Microfacet::Beckmann_distribution::roughness_to_alpha(roughness * roughness / aspect);
		Microfacet::Beckmann_distribution distribution(ax, ay);

		// models weight
		float f0 = 0.2f;
		float f90 = 1.f;

		float schlickWt = util::SchlickWeight(wo[2]);

		float dielectric_W = (1.f - metalness) * (1.f - transparency);
		float metal_W = metalness;
		float glass_W = (1.f - metalness) * transparency;

		float diffuse_pr = dielectric_W * util::luminance(albedo);
		float specular_pr = dielectric_W * util::mix(f0, f90, schlickWt);
		float metal_pr = metal_W * util::luminance(util::mix(albedo, nanovdb::Vec3f(1.f), schlickWt));
		float glass_pr = glass_W;

		float invTotalWt = 1.f / (diffuse_pr + specular_pr + metal_pr + glass_pr);
		diffuse_pr *= invTotalWt;
		specular_pr *= invTotalWt;
		metal_pr *= invTotalWt;
		glass_pr *= invTotalWt;

		float zeta = rand01;

		float u1 = rand01;
		float u2 = rand01;

		pdf = 0.f;

		// diffuse
		if (zeta < diffuse_pr) {
			wi = util::cosineSampleHemisphere(u1, u2);
		}
		// specular + metallic
		else if (zeta < diffuse_pr + specular_pr + metal_pr) {
			nanovdb::Vec3f wh = distribution.sample_wh_host();
			if (wh[2] < 0.f) wh *= -1.f;

			wi = util::reflect(-wo, wh);
		}
		// specular refraction
		else {
			float reflect_prob = util::mix(f0, f90, schlickWt);
			float exponent = 1.f + 10000.f * POW_ROUGHNESS(1.f - roughness);

			nanovdb::Vec3f refracted = util::refract(-wo, nanovdb::Vec3f(0.f, 0.f, 1.f), eta);

			if (rand01 < reflect_prob || refracted.dot(refracted) == 0.f) {
				through = false;
				nanovdb::Vec3f wh = distribution.sample_wh_host();
				if (wh[2] < 0.f) wh *= -1.f;

				wi = util::reflect(-wo, wh);
			}
			else {
				if (change_medium) isInside = !isInside;
				through = true;
				wi = sample_phong_specular_host(refracted, exponent, rng_state);
			}
		}

		wo = util::ToWorld(T, B, N, wo);
		wi = util::ToWorld(T, B, N, wi);

		return eval(wo, wi, N, pdf, eta);
	}
}
