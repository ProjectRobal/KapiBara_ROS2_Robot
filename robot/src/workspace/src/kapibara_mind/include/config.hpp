#pragma once

#include <cstdint>
#include <experimental/simd>

// specific number types used by neurons
typedef long double number;

#define MAX_SIMD_VECTOR_SIZE std::experimental::simd_abi::max_fixed_size<number>

typedef std::experimental::fixed_size_simd<number , MAX_SIMD_VECTOR_SIZE> SIMD;

typedef std::experimental::fixed_size_simd_mask<number , MAX_SIMD_VECTOR_SIZE> SIMD_MASK;

#define MAITING_THRESHOLD 0.5f

#define AMOUNT_THAT_PASS 0.3f

#define USESES_TO_MAITING 4

#define MAX_THREAD_POOL 8

#define SWARMING_SPEED_DEFAULT 10.f

#define MAX_PAST_NEURONS 6

#define DUMPING_FACTOR 0.8f