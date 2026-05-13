// Copyright 2026 Toiture1234
// 
// SPDX-License-Identifier : MIT

#pragma once

#include <ppt/util/Utility.h>
#include <ppt/cpu/CPU_texture.h>
#include <ppt/core/Microfacet.h>


#define POW_ROUGHNESS(r) ((r) * (r) * (r) * (r) * (r) * (r))
#define GAMMA_CLAMP 0.0001f

namespace penguinPT {
	// one sample BSDF type, blend between differents models
	class principled_BSDF {
	public:
		// material data
		float roughness;
		float anisotropy;
		float metalness;
		float ior;
		float scattering;

		nanovdb::Vec3f albedo;
		nanovdb::Vec3f emission;
		nanovdb::Vec3f absorption;

		// WARNING : it is very important to understand the meaning of these parameters :
		//  - when transparency is used, the surface uses refraction
		//  - when alpha is used, the light ray passes without deformation
		// The total / visual "transparency" can be calculated as follow :
		//               Tv = (1. - alpha) * transparency
		float alpha;
		float transparency; // [0 ; 1] : 1 -> fully transparent, 0 -> fully opaque

		bool change_medium;

		float g;

		bool use_albedo_tex = false;
		bool use_roughness_tex = false;
		bool use_metalness_tex = false;
		bool use_normal_tex = false;

		// CUDA textures
		cudaTextureObject_t albedo_tex_CUDA = 0;
		cudaTextureObject_t roughness_tex_CUDA = 0;
		cudaTextureObject_t normal_tex_CUDA = 0;

		// CPU textures
		CPU_float4_texture albedo_tex_HOST;
		CPU_float_texture roughness_tex_HOST;
		CPU_float4_texture normal_tex_HOST;

		__hostdev__ principled_BSDF() : roughness(0.f), anisotropy(0.), metalness(0.), ior(1.5f), albedo(0.f), emission(0.f), absorption(0.f), alpha(1.f), transparency(0.f), scattering(0.f), change_medium(true) {}
		__hostdev__ ~principled_BSDF() {}

		// Evaluation
		__hostdev__ nanovdb::Vec3f eval_wp(
			nanovdb::Vec3f wo,
			nanovdb::Vec3f wi,
			nanovdb::Vec3f N,
			float& pdf,
			nanovdb::Vec3f albedo_,
			float roughness_,
			float eta) const;

		__device__ nanovdb::Vec3f eval_CUDA(
			nanovdb::Vec3f wo, 
			nanovdb::Vec3f wi, 
			nanovdb::Vec3f N, 
			float& pdf, 
			float2 uv,
			float eta) const;

		__host__ nanovdb::Vec3f eval_HOST(
			nanovdb::Vec3f wo,
			nanovdb::Vec3f wi,
			nanovdb::Vec3f N,
			float& pdf,
			float2 uv,
			float eta) const;

		// Sampling
		__device__ nanovdb::Vec3f sample_CUDA(
			nanovdb::Vec3f wo, 
			nanovdb::Vec3f& wi, 
			nanovdb::Vec3f N, 
			float& pdf, 
			Rand_state& rng_state, 
			bool& through, 
			bool& isInside,
			bool& does_scatter,
			float2 uv) const;

		__host__ nanovdb::Vec3f sample_HOST(
			nanovdb::Vec3f wo, 
			nanovdb::Vec3f& wi, 
			nanovdb::Vec3f N, 
			float& pdf, 
			Rand_state& rng_state, 
			bool& through, 
			bool& isInside, 
			bool& does_scatter,
			float2 uv) const;


		bool __hostdev__ operator==(const principled_BSDF&);

		__device__ nanovdb::Vec4f getAlbedo_textured_CUDA(float2 uv) const;
		__host__ nanovdb::Vec4f getAlbedo_textured_HOST(float2 uv) const;

		__device__ nanovdb::Vec3f getNormal_textured_CUDA(float2 uv) const;
		__host__ nanovdb::Vec3f getNormal_textured_HOST(float2 uv) const;

		__device__ float getRoughness_textured_CUDA(float2 uv) const;
		__host__ float getRoughness_textured_HOST(float2 uv) const;
	};
	bool __hostdev__ principled_BSDF::operator==(const principled_BSDF& other) {
		return other.roughness == this->roughness &&
			other.absorption == this->absorption &&
			other.albedo == this->albedo &&
			other.anisotropy == this->anisotropy &&
			other.emission == this->emission &&
			other.ior == this->ior &&
			//other.is_through == this->is_through &&
			other.metalness == this->metalness &&
			other.transparency == this->transparency &&
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
	
	__hostdev__ nanovdb::Vec3f principled_BSDF::eval_wp(
		nanovdb::Vec3f wo,
		nanovdb::Vec3f wi,
		nanovdb::Vec3f N,
		float& pdf,
		nanovdb::Vec3f albedo_,
		float roughness_,
		float eta) const
	{
		auto pow5 = [](float v) {return (v * v) * (v * v) * v; };

		// convert to local space
		nanovdb::Vec3f T, B;
		util::Onb(N, T, B);

		wo = util::ToLocal(T, B, N, wo), wi = util::ToLocal(T, B, N, wi);
		nanovdb::Vec3f wh;
		if (wo[2] > 0.f) wh = (wo + wi).normalize();
		else wh = (wo + wi * eta).normalize();

		if (wh[2] < 0.f) wh *= -1.f;

		Microfacet::Phong_distribution distribution(roughness_);

		// models weight
		float f0 = (ior - 1.f) * (ior - 1.f) / ((ior + 1.f) * (ior + 1.f));
		float f90 = 1.f;

		float schlickWt = util::SchlickWeight(wo[2]);

		float dielectric_W = (1.f - metalness) * (1.f - transparency);
		float metal_W = metalness;
		float glass_W = (1.f - metalness) * transparency;

		float diffuse_pr = dielectric_W * util::luminance(albedo_);
		float specular_pr = dielectric_W * util::mix(f0, f90, schlickWt);
		float metal_pr = metal_W * util::luminance(util::mix(albedo_, nanovdb::Vec3f(1.f), schlickWt));
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
			nanovdb::Vec3f diffuse = (1.f - f0) * albedo_ / PI;

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
			nanovdb::Vec3f specular = albedo_ * distribution.D(wh) / (4.f * fabsf(wi.dot(wh)) * fmaxf(cosThetaI, cosThetaO));
			float H_pdf = distribution.PDF(wh, wo) / (4.f * wo.dot(wh));

			L += specular * metal_W;
			pdf += H_pdf * metal_pr;
		}
		if (glass_pr > 0.f) {
			float reflect_prob = util::mix(f0, f90, schlickWt);
			//float exponent = 1.f + 10000.f * POW_ROUGHNESS(1.f - roughness);
			float gamma = fmaxf(roughness_ * roughness_, GAMMA_CLAMP);
			float exponent = 1.f + 1.f / gamma;

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
				float p_cosAlpha = powf(cosAlpha, exponent) * INV_TWO_PI;

				L += nanovdb::Vec3f(p_cosAlpha) * glass_W * (exponent + 2.f);
				pdf += p_cosAlpha * (1. - reflect_prob) * glass_pr * (exponent + 1.f);
			}
		}

		return L * fabsf(wi[2]);
	}

	__device__ nanovdb::Vec3f principled_BSDF::eval_CUDA(
		nanovdb::Vec3f wo,
		nanovdb::Vec3f wi,
		nanovdb::Vec3f N,
		float& pdf,
		float2 uv,
		float eta) const 
	{
		nanovdb::Vec4f albedo_ = getAlbedo_textured_CUDA(uv);
		float roughness_ = getRoughness_textured_CUDA(uv);
		return eval_wp(wo, wi, N, pdf, nanovdb::Vec3f(albedo_[0], albedo_[1], albedo_[2]), roughness_, eta);
	}

	__host__ nanovdb::Vec3f principled_BSDF::eval_HOST(
		nanovdb::Vec3f wo,
		nanovdb::Vec3f wi,
		nanovdb::Vec3f N,
		float& pdf,
		float2 uv,
		float eta) const 
	{
		nanovdb::Vec4f albedo_ = getAlbedo_textured_HOST(uv);
		float roughness_ = getRoughness_textured_HOST(uv);
		return eval_wp(wo, wi, N, pdf, nanovdb::Vec3f(albedo_[0], albedo_[1], albedo_[2]), roughness_, eta);
	}

	__device__ nanovdb::Vec3f principled_BSDF::sample_CUDA(nanovdb::Vec3f wo, nanovdb::Vec3f& wi, nanovdb::Vec3f N, float& pdf, Rand_state& rng_state, bool& through, bool& isInside, bool& does_scatter, float2 uv) const {
		through = false;

		float eta = isInside ? ior : 1.f / ior;

		auto pow5 = [](float v) {return (v * v) * (v * v) * v; };

		nanovdb::Vec4f al_data = getAlbedo_textured_CUDA(uv);
		nanovdb::Vec3f albedo_ = nanovdb::Vec3f(al_data[0], al_data[1], al_data[2]);
		float alpha_ = al_data[3];

		float roughness_ = getRoughness_textured_CUDA(uv);
		
		// potentially go through surface
		if (randC(&rng_state) >= alpha_) {
			through = true;
			does_scatter = false;
			return nanovdb::Vec3f(pdf);
		}

		does_scatter = true;

		// convert to local space
		nanovdb::Vec3f T, B;
		util::Onb(N, T, B);

		wo = util::ToLocal(T, B, N, wo), wi = util::ToLocal(T, B, N, wi);
		nanovdb::Vec3f wh = (wo + wi).normalize();

		Microfacet::Phong_distribution distribution(roughness_);

		// models weight
		float f0 = (ior - 1.f) * (ior - 1.f) / ((ior + 1.f) * (ior + 1.f));
		float f90 = 1.f;

		float schlickWt = util::SchlickWeight(wo[2]);

		float dielectric_W = (1.f - metalness) * (1.f - transparency);
		float metal_W = metalness;
		float glass_W = (1.f - metalness) * transparency;

		float diffuse_pr = dielectric_W * util::luminance(albedo_);
		float specular_pr = dielectric_W * util::mix(f0, f90, schlickWt);
		float metal_pr = metal_W * util::luminance(util::mix(albedo_, nanovdb::Vec3f(1.f), schlickWt));
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
		else if (zeta < diffuse_pr + specular_pr + metal_pr) {
			nanovdb::Vec3f wh = distribution.sample_wh(rng_state);
			if (wh[2] < 0.f) wh *= -1.f;

			wi = util::reflect(-wo, wh);
		}
		// specular refraction
		else {
			float reflect_prob = util::mix(f0, f90, schlickWt);
			float gamma = fmaxf(roughness_ * roughness_, GAMMA_CLAMP);
			float exponent = 1.f + 1.f / gamma;

			nanovdb::Vec3f refracted = util::refract(-wo, nanovdb::Vec3f(0.f, 0.f, 1.f), eta);

			if (randC(&rng_state) < reflect_prob || refracted.dot(refracted) == 0.f) {
				//wi_perfect = util::reflect(-wo, nanovdb::Vec3f(0.f, 0.f, 1.f));
				through = false;
				nanovdb::Vec3f wh = distribution.sample_wh(rng_state);
				if (wh[2] < 0.f) wh *= -1.f;

				wi = util::reflect(-wo, wh);
			}
			else {
				if (change_medium) isInside = !isInside;
				through = true;
				wi = sample_phong_specular(refracted, exponent, rng_state);
			}
		}

		wo = util::ToWorld(T, B, N, wo);
		wi = util::ToWorld(T, B, N, wi);

		return eval_wp(wo, wi, N, pdf, albedo_, roughness_, eta);
	}
	__host__ nanovdb::Vec3f principled_BSDF::sample_HOST(nanovdb::Vec3f wo, nanovdb::Vec3f& wi, nanovdb::Vec3f N, float& pdf, Rand_state& rng_state, bool& through, bool& isInside, bool& does_scatter, float2 uv) const {
		through = false;

		float eta = isInside ? ior : 1.f / ior;

		auto pow5 = [](float v) {return (v * v) * (v * v) * v; };

		nanovdb::Vec4f al_data = getAlbedo_textured_HOST(uv);
		nanovdb::Vec3f albedo_ = nanovdb::Vec3f(al_data[0], al_data[1], al_data[2]);
		float alpha_ = al_data[3];

		float roughness_ = getRoughness_textured_HOST(uv);

		// potentially go through surface
		if (rand01 >= alpha_) {
			through = true;
			does_scatter = false;
			return nanovdb::Vec3f(pdf);
		}

		does_scatter = true;

		// convert to local space
		nanovdb::Vec3f T, B;
		util::Onb(N, T, B);

		wo = util::ToLocal(T, B, N, wo), wi = util::ToLocal(T, B, N, wi);
		nanovdb::Vec3f wh = (wo + wi).normalize();

		Microfacet::Phong_distribution distribution(roughness_);

		// models weight
		float f0 = (ior - 1.f) * (ior - 1.f) / ((ior + 1.f) * (ior + 1.f));
		float f90 = 1.f;

		float schlickWt = util::SchlickWeight(wo[2]);

		float dielectric_W = (1.f - metalness) * (1.f - transparency);
		float metal_W = metalness;
		float glass_W = (1.f - metalness) * transparency;

		float diffuse_pr = dielectric_W * util::luminance(albedo_);
		float specular_pr = dielectric_W * util::mix(f0, f90, schlickWt);
		float metal_pr = metal_W * util::luminance(util::mix(albedo_, nanovdb::Vec3f(1.f), schlickWt));
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
			float gamma = fmaxf(roughness_ * roughness_, GAMMA_CLAMP);
			float exponent = 1.f + 1.f / gamma;

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

		return eval_wp(wo, wi, N, pdf, albedo_, roughness_, eta);
	}

	__device__ nanovdb::Vec4f principled_BSDF::getAlbedo_textured_CUDA(float2 uv) const {
		float alpha_ = alpha;
		nanovdb::Vec3f albedo_ = albedo;

		if (use_albedo_tex) {
			float4 albedo_data = tex2D<float4>(albedo_tex_CUDA, uv.x, uv.y);
			albedo_ = { albedo_data.x, albedo_data.y, albedo_data.z };
			alpha_ = albedo_data.w;
		}
		return nanovdb::Vec4f(albedo_[0], albedo_[1], albedo_[2], alpha_);
	}
	__host__ nanovdb::Vec4f principled_BSDF::getAlbedo_textured_HOST(float2 uv) const {
		float alpha_ = alpha;
		nanovdb::Vec3f albedo_ = albedo;

		if (use_albedo_tex) {
			float4 albedo_data = albedo_tex_HOST.sample_float(uv.x, uv.y);
			albedo_ = { albedo_data.x, albedo_data.y, albedo_data.z };
			alpha_ = albedo_data.w;
		}
		return nanovdb::Vec4f(albedo_[0], albedo_[1], albedo_[2], alpha_);
	}
	__device__ float principled_BSDF::getRoughness_textured_CUDA(float2 uv) const {
		float roughness_ = roughness;
		if (use_roughness_tex) {
			roughness_ = tex2D<float>(roughness_tex_CUDA, uv.x, uv.y);
		}
		return roughness_;
	}
	__host__ float principled_BSDF::getRoughness_textured_HOST(float2 uv) const {
		float roughness_ = roughness;
		if (use_roughness_tex) {
			roughness_ = roughness_tex_HOST.sample_float(uv.x, uv.y);
		}
		return roughness_;
	}

	__device__ nanovdb::Vec3f principled_BSDF::getNormal_textured_CUDA(float2 uv) const {
		if (use_normal_tex) {
			float4 data = tex2D<float4>(normal_tex_CUDA, uv.x, uv.y);
			return nanovdb::Vec3f(data.x, data.y, data.z) * 2.f - nanovdb::Vec3f(1.f);
		}
		else {
			return nanovdb::Vec3f(0.0f, 0.0f, 1.f);
		}
	}
	__host__ nanovdb::Vec3f principled_BSDF::getNormal_textured_HOST(float2 uv) const {
		if (use_normal_tex) {
			float4 data = normal_tex_HOST.sample_float(uv.x, uv.y);
			return nanovdb::Vec3f(data.x, data.y, data.z) * 2.f - nanovdb::Vec3f(1.f);
		}
		else {
			return nanovdb::Vec3f(0.0f, 0.0f, 1.f);
		}
	}
}
