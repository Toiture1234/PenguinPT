#pragma once

// options
#define QUICK_DIFFUSE
#define MAX_DISTANCE 1.0e30f
#define BOUNCES_PT 10
#define BOUNCES_PT_VOL 32
#define SAFE_OFFSET 0.005f

#include "cuda_runtime.h"
#include "device_launch_parameters.h"
#include <curand_kernel.h>

#include <nanovdb/io/IO.h>
#include <nanovdb/cuda/DeviceBuffer.h>
#include <nanovdb/tools/GridBuilder.h>
#include <nanovdb/math/Ray.h>

#include <SFML/Graphics.hpp>

#include <stdio.h>
#include <iostream>
#include <vector>
#include <string>
#include <random>
#include <fstream>
#include <filesystem>

typedef curandStatePhilox4_32_10_t Rand_state;
#define randC(state) curand_uniform(state)
#define rand_both(state, roC) roC ? curand_uniform(state) : rand()

#define CUDA_CHECK(expr) if(expr != CUDA_SUCCESS) { printf("CUDA ERROR AT LINE %i IN %s : %s \n", __LINE__, __FILE__, cudaGetErrorString(expr)); exit(99); }

#define __hostdev__ __device__ __host__

#define PI 3.1415926535897932384626433832795f
#define TWO_PI 6.283185307179586476925286766559f
#define INV_TWO_PI 0.15915494309189533576888376337251f
#define INV_PI 0.31830988618379067153776752674503f
#define INV_4_PI 0.07957747154594766788444188168626f

#include "Utility.h"
#include "Material.h"
#include "volumes.h"
#include "intersectors.h"
#include "camera.h"
#include "renderer_services.h"
#include "pathtracer.h"
#include "obj_loader.h"
#include "nanovdb_loader.h"