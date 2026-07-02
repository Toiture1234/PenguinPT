// Copyright 2026 Toiture1234
// 
// SPDX-License-Identifier : MIT

#pragma once

#include <ppt/util/options.h>
#include <ppt/util/Utility.h>
#include <ppt/core/renderer_services.h>

namespace penguinPT {
	__hostdev__ nanovdb::Vec3f previewRender(nanovdb::math::Ray<float> ray, renderer_services& rs);
	__hostdev__ float volumeTransmittance(nanovdb::math::Ray<float> ray, renderer_services& rs);
}