#pragma once

#include <cstdint>
#include <experimental/simd>

// specific number types used by neurons
typedef float number;

#define MAX_SIMD_VECTOR_SIZE std::experimental::simd_abi::max_fixed_size<number>

typedef std::experimental::fixed_size_simd<number , MAX_SIMD_VECTOR_SIZE> SIMD;

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


// A maximum weight switch probablity
#define MAX_SWITCH_PROBABILITY 0.5f

// A probability at which weights are switched
#define REWARD_TO_SWITCH_PROBABILITY -0.05f

// A rate at weights move to positive weight
#define POSITIVE_P 0.1f


#define USED_THREADS 4