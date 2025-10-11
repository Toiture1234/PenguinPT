#pragma once

#define POW_ROUGHNESS(r) ((r) * (r) * (r) * (r) * (r) * (r))

namespace penguinPT {
	enum BSDF_types {
		BSDF_null = 0,
		BSDF_diffuse,
		BSDF_specular,
		BSDF_glass,
		BSDF_blinn_phong,
		BSDF_through
	};

#define BSDF_EVAL_PARAMS nanovdb::math::Ray<float> ray, nanovdb::Vec3f N, nanovdb::Vec3f L_dir, float& pdf, bool inside, bool through
#define BSDF_SAMPLE_PARAMS nanovdb::math::Ray<float> ray, nanovdb::Vec3f N, nanovdb::Vec3f& L_out, Rand_state& rng_state, float& pdf, bool& inside, bool& through
#define BSDF_EVAL_PARAMS_FUNC ray, N, L_dir, pdf, inside, through
#define BSDF_SAMPLE_PARAMS_FUNC ray, N, L_out, rng_state, pdf, inside, through

	// Class for handling different BSDF
	// contains two functions : eval and sample,
	// these functions will depend on which BSDF is used
	class BSDF {
	public:
		nanovdb::Vec3f albedo;
		float roughness;
		float metalness;
		float IOR;
		nanovdb::Vec3f emission;
		nanovdb::Vec3f attenuation;
		float anisotropy;

		unsigned int bsdf_type = BSDF_null;
	public:
		__hostdev__ BSDF() : albedo(0.f), roughness(0.f), metalness(0.f), IOR(0.f), emission(0.f), attenuation(0.f), anisotropy(0.f) {}
		__hostdev__ ~BSDF() {}

		// Eval BSDF response based on incoming and exiting rays along with the normal of the surface,
		// returns the response and the pdf can be obtained in parameters.
		//__device__ nanovdb::Vec3f eval(nanovdb::math::Ray<float> ray, nanovdb::Vec3f N, nanovdb::Vec3f L_dir, float& pdf, Material_params params);
		__device__ inline nanovdb::Vec3f eval(BSDF_EVAL_PARAMS);

		// Sample BSDF based on the normal, incoming ray direction and rng values,
		// returns the BSDF response at sampled direction, the latter is stored in L_out, 
		// pdf is also returned in pdf parameter.
		//__device__ nanovdb::Vec3f sample(nanovdb::math::Ray<float> ray, nanovdb::Vec3f N, nanovdb::Vec3f& L_out, Rand_state& rng_state, float& pdf, Material_params params);
		__device__ inline nanovdb::Vec3f sample(BSDF_SAMPLE_PARAMS);


		// Simple diffuse BSDF, the response is cosine weighted based on the normal of the surface
		__device__ inline nanovdb::Vec3f eval_diffuse(BSDF_EVAL_PARAMS);
		__device__ inline nanovdb::Vec3f sample_diffuse(BSDF_SAMPLE_PARAMS);

		// null BSDF
		__device__ inline nanovdb::Vec3f eval_null(BSDF_EVAL_PARAMS);
		__device__ inline nanovdb::Vec3f sample_null(BSDF_SAMPLE_PARAMS);

		// trough BSDF
		__hostdev__ inline nanovdb::Vec3f eval_through(BSDF_EVAL_PARAMS);
		__hostdev__ inline nanovdb::Vec3f sample_through(BSDF_SAMPLE_PARAMS);

		// specular BSDF
		__device__ inline nanovdb::Vec3f eval_specular(BSDF_EVAL_PARAMS);
		__device__ inline nanovdb::Vec3f sample_specular(BSDF_SAMPLE_PARAMS);

		// glass BSDF
		__device__ inline nanovdb::Vec3f eval_glass(BSDF_EVAL_PARAMS);
		__device__ inline nanovdb::Vec3f sample_glass(BSDF_SAMPLE_PARAMS);

		__device__ inline nanovdb::Vec3f eval_blinn_phong(BSDF_EVAL_PARAMS);
		__device__ inline nanovdb::Vec3f sample_blinn_phong(BSDF_SAMPLE_PARAMS);
		__hostdev__ inline float blinn_phong_pdf(const nanovdb::Vec3f& wo, const nanovdb::Vec3f& wi);
	};


	// diffuse BSDF
	__device__ inline nanovdb::Vec3f BSDF::eval_diffuse(BSDF_EVAL_PARAMS)  {
		float cosine_term = N.dot(L_dir);
		pdf = INV_PI * cosine_term;
		return INV_PI * albedo * cosine_term;
	}

	__device__ inline nanovdb::Vec3f BSDF::sample_diffuse(BSDF_SAMPLE_PARAMS) {
#ifndef QUICK_DIFFUSE
		float u1 = randC(&rng_state);
		float u2 = randC(&rng_state);

		float phi = TWO_PI * u2;
		float cos_theta = sqrtf(u1);
		float sin_theta = sqrtf(1.f - cos_theta * cos_theta);

		nanovdb::Vec3f T, B;
		util::Onb(N, T, B);

		L_out = (sin_theta * cosf(phi) * T + sin_theta * sinf(phi) * B + cos_theta * N).normalize();
		pdf = INV_PI * cos_theta;
		return INV_PI * albedo * cos_theta;
#else
		L_out = (N + util::generateUniformSample(rng_state)).normalize();
		return eval_diffuse(ray, N, L_out, pdf, inside, through);
#endif
	}
	
	// null bsdf
	__device__ inline nanovdb::Vec3f BSDF::eval_null(BSDF_EVAL_PARAMS) {
		pdf = 0.f;
		return { 0.f, 0.f, 0.f };
	}
	__device__ inline nanovdb::Vec3f BSDF::sample_null(BSDF_SAMPLE_PARAMS) {
		pdf = 0.f;
		L_out = nanovdb::Vec3f(0.f);
		return { 0.f, 0.f, 0.f };
	}

	// trough bsdf
	__hostdev__ inline nanovdb::Vec3f BSDF::eval_through(BSDF_EVAL_PARAMS) {
		float k = ray.dir().dot(L_dir);
		pdf = k == 1.f ? 1.f : 0.f;
		return nanovdb::Vec3f(pdf);
	}
	__hostdev__ inline nanovdb::Vec3f BSDF::sample_through(BSDF_SAMPLE_PARAMS) {
		L_out = ray.dir();
		pdf = 1.f;
		through = true;
		return nanovdb::Vec3f(1.f);
	}

	// specular bsdf
	__device__ inline nanovdb::Vec3f BSDF::eval_specular(BSDF_EVAL_PARAMS) {
		/*nanovdb::Vec3f Reflected = util::reflect(ray.dir(), N);
		float n = 1.f + 1000.f * POW_ROUGHNESS(1.f - roughness);

		float cosAlpha = CLAMP(Reflected.dot(L_dir), 0.f, 1.f);
		pdf = powf(cosAlpha, n) * (n + 1.f) * INV_TWO_PI;
		return powf(cosAlpha, n) * util::mix(nanovdb::Vec3f(1.f), albedo, metalness) * (n + 1.f) * INV_TWO_PI;*/

		nanovdb::Vec3f T, B;
		util::Onb(N, T, B);

		nanovdb::Vec3f wo = -ray.dir();
		nanovdb::Vec3f wi = L_dir;

		wo = util::ToLocal(T, B, N, wo);
		wi = util::ToLocal(T, B, N, wi);

		float aspect = sqrtf(1.f - 0.9f * anisotropy);
		float ax = Microfacet::Beckmann_distribution::roughness_to_alpha(roughness * roughness * aspect);
		float ay = Microfacet::Beckmann_distribution::roughness_to_alpha(roughness * roughness / aspect);
		Microfacet::Beckmann_distribution dsrt(ax, ay);

		float cosThetaO = fabsf(util::CosTheta(wo)), cosThetaI = fabsf(util::CosTheta(wi));

		nanovdb::Vec3f H = wo + wi;
		if (cosThetaO == 0.f || cosThetaI == 0.f) return nanovdb::Vec3f(0.f);
		if(H.dot(H) == 0.f) return nanovdb::Vec3f(0.f);

		H = H.normalize();
		float F = Microfacet::fresnel_dielectric(wi.dot(H), 1.f, IOR);

		// pdf
		pdf = dsrt.PDF(H, wo) / (4.f * wo.dot(H));

		return dsrt.D(H) * dsrt.G(wo, wi) / (4.f * cosThetaI * cosThetaO) * util::mix(nanovdb::Vec3f(1.f), albedo, metalness);
	}
	__device__ inline nanovdb::Vec3f BSDF::sample_specular(BSDF_SAMPLE_PARAMS) {
		/*nanovdb::Vec3f Reflected = util::reflect(ray.dir(), N);
		float n = 1.f + 1000.f * POW_ROUGHNESS(1.f - roughness);

		float u1 = randC(&rng_state);
		float u2 = randC(&rng_state);

		float phi = TWO_PI * u2;
		float cos_alpha = powf(u1, 1.f / (n + 1.f));
		float sin_alpha = sqrtf(1.f - cos_alpha * cos_alpha);

		nanovdb::Vec3f T, B;
		util::Onb(Reflected, T, B);

		L_out = (sin_alpha * cosf(phi) * T + sin_alpha * sinf(phi) * B + cos_alpha * Reflected).normalize();
		pdf = powf(cos_alpha, n) * (n + 1.f) * INV_TWO_PI;

		if (L_out.dot(N) < 0.f) pdf = 0.f;
		return pdf * util::mix(nanovdb::Vec3f(1.f), albedo, metalness);*/

		// sample Beckmann
		float aspect = sqrtf(1.f - 0.9f * anisotropy);
		float ax = Microfacet::Beckmann_distribution::roughness_to_alpha(roughness * roughness * aspect);
		float ay = Microfacet::Beckmann_distribution::roughness_to_alpha(roughness * roughness / aspect);
		Microfacet::Beckmann_distribution dsrt(ax, ay);

		nanovdb::Vec3f T, B;
		util::Onb(N, T, B);

		nanovdb::Vec3f wo = util::ToLocal(T, B, N, -ray.dir());
		nanovdb::Vec3f H = dsrt.sample_wh(rng_state);

		L_out = util::reflect(-wo, H);
		
		//pdf = dsrt.PDF(H, wo) / (4.f * wo.dot(H));
		
		// convert back to world
		L_out = util::ToWorld(T, B, N, L_out);

		return eval_specular(ray, N, L_out, pdf, inside, through);
	}

	__device__ inline nanovdb::Vec3f BSDF::eval_glass(BSDF_EVAL_PARAMS) {
		//TODO
		float n1 = !inside ? 1.0f : IOR;
		float n2 = !inside ? IOR : 1.0f;
		float eta = n1 / n2;

		float f_prob = util::fresnelAmount(n1, n2, N, ray.dir(), 0.2f, 1.f);
		float n = 1.f + 10000.f * POW_ROUGHNESS(1.f - roughness);

		nanovdb::Vec3f l = nanovdb::Vec3f(0.f);
		pdf = 0.f;
		if (f_prob > 0.f) { // specular
			nanovdb::Vec3f Reflected = util::reflect(ray.dir(), N);
			float cosAlpha = CLAMP(Reflected.dot(L_dir), 0.f, 1.f);
			float p_cosAlpha = powf(cosAlpha, n) * (n + 1.f) * INV_TWO_PI;

			pdf += p_cosAlpha * f_prob;
			l += nanovdb::Vec3f(p_cosAlpha) * f_prob;
		} 
		if (1.f - f_prob > 0.f) { // refraction
			nanovdb::Vec3f Refracted = util::refract(ray.dir(), N, eta);
			float cosAlpha = CLAMP(Refracted.dot(L_dir), 0.f, 1.f);
			float p_cosAlpha = powf(cosAlpha, n) * (n + 1.f) * INV_TWO_PI;

			pdf += p_cosAlpha * (1.f - f_prob);
			l += nanovdb::Vec3f(p_cosAlpha) * (1.f - f_prob);
		}
		return l;
	}
	__device__ inline nanovdb::Vec3f BSDF::sample_glass(BSDF_SAMPLE_PARAMS) {
		float n1 = !inside ? 1.0f : IOR;
		float n2 = !inside ? IOR : 1.0f;
		float eta = n1 / n2;
		
		float f_prob = util::fresnelAmount(n1, n2, N, ray.dir(), 0.2f, 1.f);
		float n = 1.f + 10000.f * POW_ROUGHNESS(1.f - roughness);

		nanovdb::Vec3f Dir_perfect;
		float w;
		nanovdb::Vec3f refracted = util::refract(ray.dir(), N, eta);
		if (randC(&rng_state) < f_prob || refracted.dot(refracted) == 0.f) {
			Dir_perfect = util::reflect(ray.dir(), N);
			w = f_prob;
			through = false;
		}
		else {
			Dir_perfect = refracted;
			inside = !inside;
			w = 1.f - f_prob;
			through = true;
		}
		
		float u1 = randC(&rng_state);
		float u2 = randC(&rng_state);

		float phi = TWO_PI * u2;
		float cos_alpha = powf(u1, 1.f / (n + 1.f));
		float sin_alpha = sqrtf(1.f - cos_alpha * cos_alpha);

		nanovdb::Vec3f T, B;
		util::Onb(Dir_perfect, T, B);

		L_out = (sin_alpha * cosf(phi) * T + sin_alpha * sinf(phi) * B + cos_alpha * Dir_perfect).normalize();
		float pdf0 = powf(cos_alpha, n) * (n + 1.f) * INV_TWO_PI;
		pdf = pdf0 * w;
		
		if (L_out.dot(N) * (through ? -1.f : 1.f) <= 0.f) pdf = 0.f;
		
		return nanovdb::Vec3f(pdf0);
	}

	
	// actually not but who cares
	__hostdev__ inline float BSDF::blinn_phong_pdf(const nanovdb::Vec3f& wo, const nanovdb::Vec3f& wi) {
		float aspect = sqrtf(1.f - 0.9f * anisotropy);
		float ax = Microfacet::Beckmann_distribution::roughness_to_alpha(roughness * roughness * aspect);
		float ay = Microfacet::Beckmann_distribution::roughness_to_alpha(roughness * roughness / aspect);
		Microfacet::Beckmann_distribution dsrt(ax, ay);

		nanovdb::Vec3f H = (wo + wi).normalize();
		float H_pdf = dsrt.PDF(H, wo);
		return 0.5f * (fabsf(util::CosTheta(wi)) * INV_PI + H_pdf / (4.f * wo.dot(H)));
	}
	__device__ inline nanovdb::Vec3f BSDF::eval_blinn_phong(BSDF_EVAL_PARAMS) {
		auto pow5 = [](float v) {return (v * v) * (v * v) * v; };

		float rS = 0.3f;

		nanovdb::Vec3f T, B;
		util::Onb(N, T, B);
		nanovdb::Vec3f wo = util::ToLocal(T, B, N, -ray.dir());
		nanovdb::Vec3f wi = util::ToLocal(T, B, N, L_dir);

		float cosThetaI = fabsf(util::CosTheta(wi)), cosThetaO = fabsf(util::CosTheta(wo));
		nanovdb::Vec3f diffuse = (28.f / (23.f * PI)) * albedo * (1.f - rS) * (1.f - pow5(1.f - 0.5f * cosThetaI)) * (1.f - pow5(1.f - 0.5f * cosThetaO));
		nanovdb::Vec3f H = wi + wo;
		if (H.dot(H) == 0.f) return nanovdb::Vec3f(0.f);
		H = H.normalize();

		float aspect = sqrtf(1.f - 0.9f * anisotropy);
		float ax = Microfacet::Beckmann_distribution::roughness_to_alpha(roughness * roughness * aspect);
		float ay = Microfacet::Beckmann_distribution::roughness_to_alpha(roughness * roughness / aspect);
		Microfacet::Beckmann_distribution dsrt(ax, ay);

		float fresnel = rS + pow5(1.f - wi.dot(H)) * (1.f - rS);
		nanovdb::Vec3f specular = nanovdb::Vec3f(1.f) * dsrt.D(H) / (4.f * fabsf(wi.dot(H)) * fmaxf(cosThetaI, cosThetaO)) * fresnel;
		return diffuse + specular;
	}
	__device__ inline nanovdb::Vec3f BSDF::sample_blinn_phong(BSDF_SAMPLE_PARAMS) {
		float u1 = randC(&rng_state);
		float u2 = randC(&rng_state);

		nanovdb::Vec3f T, B;
		util::Onb(N, T, B);
		nanovdb::Vec3f wo = util::ToLocal(T, B, N, -ray.dir());

		if (u1 < 0.5f) {
			u1 *= 2.f;
			L_out = util::cosineSampleHemisphere(u1, u2);
		}
		else {
			u1 = 2.f * (u1 - 0.5f);
			// sample Beckmann
			float aspect = sqrtf(1.f - 0.9f * anisotropy);
			float ax = Microfacet::Beckmann_distribution::roughness_to_alpha(roughness * roughness * aspect);
			float ay = Microfacet::Beckmann_distribution::roughness_to_alpha(roughness * roughness / aspect);
			Microfacet::Beckmann_distribution dsrt(ax, ay);

			nanovdb::Vec3f H = dsrt.sample_wh(rng_state);

			L_out = util::reflect(-wo, H);
		}

		pdf = blinn_phong_pdf(wo, L_out);
		
		L_out = util::ToWorld(T, B, N, L_out);

		float pdf0 = 0.f;
		return eval_blinn_phong(ray, N, L_out, pdf0, false, false);
	}

	__device__ inline nanovdb::Vec3f BSDF::eval(BSDF_EVAL_PARAMS) {

		switch (bsdf_type)
		{
		case BSDF_null :
			return eval_null(BSDF_EVAL_PARAMS_FUNC);
		case BSDF_diffuse :
			return eval_diffuse(BSDF_EVAL_PARAMS_FUNC);
		case BSDF_through:
			return eval_through(BSDF_EVAL_PARAMS_FUNC);
		case BSDF_specular:
			return eval_specular(BSDF_EVAL_PARAMS_FUNC);
		case BSDF_glass:
			return eval_glass(BSDF_EVAL_PARAMS_FUNC);
		case BSDF_blinn_phong:
			return eval_blinn_phong(BSDF_EVAL_PARAMS_FUNC);
		}
	}

	__device__ inline nanovdb::Vec3f BSDF::sample(BSDF_SAMPLE_PARAMS) {

		switch (bsdf_type)
		{
		case BSDF_null:
			return sample_null(BSDF_SAMPLE_PARAMS_FUNC);
		case BSDF_diffuse:
			return sample_diffuse(BSDF_SAMPLE_PARAMS_FUNC);
		case BSDF_through:
			return sample_through(BSDF_SAMPLE_PARAMS_FUNC);
		case BSDF_specular:
			return sample_specular(BSDF_SAMPLE_PARAMS_FUNC);
		case BSDF_glass:
			return sample_glass(BSDF_SAMPLE_PARAMS_FUNC);
		case BSDF_blinn_phong:
			return sample_blinn_phong(BSDF_SAMPLE_PARAMS_FUNC);
		}
	}

	// one sample BSDF type, blend between differents models
	class principled_BSDF {
	public:
		// material data
		float roughness;
		float anisotropy;
		float metalness;
		float ior;

		float g;
		float opacity;

		nanovdb::Vec3f albedo;
		nanovdb::Vec3f emission;
		nanovdb::Vec3f absorption;

		__hostdev__ principled_BSDF() : roughness(0.f), anisotropy(0.), metalness(0.), ior(0.f), g(0.f), opacity(0.f), albedo(0.f), emission(0.f), absorption(0.f) {}
		__hostdev__ ~principled_BSDF() {}

		__hostdev__ nanovdb::Vec3f eval() const;

		__device__ nanovdb::Vec3f sample_CUDA() const;
		__host__ nanovdb::Vec3f sample_HOST() const;
	};
}