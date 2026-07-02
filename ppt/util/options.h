// Copyright 2026 Toiture1234
// 
// SPDX-License-Identifier : MIT

#pragma once

#define WINDOWS_VERSION 

// options
#define QUICK_DIFFUSE
#define MAX_DISTANCE 1.0e30f
#define BOUNCES_PT 10
#define BOUNCES_PT_VOL 1024
#define SAFE_OFFSET 0.001f
#define VOLUME_STACK_SIZE 16
#define DIRECT_LIGHT_STEPS 16

#define TLAS_INTERSECTOR_STACK_SIZE 64
#define BVH_INTERSECTOR_STACK_SIZE 32

#define BASE_BSDF 1 // pure black BSDF
#define BSDF_TROUGH_ID 0 // for volumes

#define MODE_TRIANGLE 1

#define ANTIALIAS_SIZE 0.0005f

// debug things
//#define WHITE_FURNACE
//#define SEE_NORMALS

// Warning : for some reasons the direct lightning is not working on CPU,
// this is caused by unworking envmap sampling I don't know why
#define DIRECT_LIGHTNING
//#define EXPERIMENTAL_CAUSTICS

//#define NO_SKY_NO_BOUNCE
