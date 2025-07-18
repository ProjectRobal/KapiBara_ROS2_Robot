#pragma once

#include <cstdint>
#include <experimental/simd>

// specific number types used by neurons
typedef float number;

#define MAX_SIMD_VECTOR_SIZE std::experimental::simd_abi::max_fixed_size<number>

typedef std::experimental::fixed_size_simd<number , MAX_SIMD_VECTOR_SIZE> SIMD;

typedef std::experimental::fixed_size_simd<number , MAX_SIMD_VECTOR_SIZE>::reference SIMD_ref;

typedef std::experimental::fixed_size_simd_mask<number , MAX_SIMD_VECTOR_SIZE> SIMD_MASK;

#define ERROR_THRESHOLD_KAN (0.01f)

#define MAITING_THRESHOLD 0.5f

#define AMOUNT_THAT_PASS 0.3f

#define USESES_TO_MAITING 4

#define MAX_THREAD_POOL 8

#define SWARMING_SPEED_DEFAULT 10.f

#define INITIAL_STD 0.1f

#define MIN_STD 0.00001f

#define MAX_STD 0.5f

#define MUTATION_PROBABILITY 0.25f

#define ERROR_THRESHOLD_FOR_FIT (0.0001f)

#define ERROR_THRESHOLD_FOR_INSERTION (0.00000001f)

#define ERROR_THRESHOLD_FOR_POINT_REMOVAL (0.00000001f)

// KAN configs:

// we use uniform distribution for x values
#define DEF_X_LEFT ((number)-10.f)

#define DEF_X_RIGHT ((number)10.f)

// we use gaussian distribution for initial values
#define DEF_Y_INIT snn::GaussInit<(number)0.f,(number)0.001f>


#define THREAD_COUNT (4)


// A maximum weight switch probablity
#define MAX_SWITCH_PROBABILITY 0.5f

// A probability at which weights are switched
#define REWARD_TO_SWITCH_PROBABILITY -0.05f

// A rate at weights move to positive weight
#define POSITIVE_P 0.1f


#define USED_THREADS 4