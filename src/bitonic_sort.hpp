#pragma once

#include <intrin.h>

__m512i bitonic_sort(__m512i indices, __m512 distances) {
	
	std::array<float, 16> ary_dist;
	std::array<uint32_t, 16> ary_indx;

#define shuffle_once(i0, i1, i2, i3, i4, i5, i6, i7, i8, i9, i10, i11, i12, i13, i14, i15) \
	{\
		__m512i pattern = _mm512_setr_epi32(i0, i1, i2, i3, i4, i5, i6, i7, i8, i9, i10, i11, i12, i13, i14, i15); \
		__m512i shuffled_indices = _mm512_permutexvar_epi32(pattern, indices); \
		__m512 shuffled_distances = _mm512_permutexvar_ps(pattern, distances); \
		__mmask16 cmp = _mm512_cmple_ps_mask(shuffled_distances, distances); \
		distances = _mm512_mask_blend_ps(cmp, distances, shuffled_distances); \
		indices = _mm512_mask_blend_epi32(cmp, indices, shuffled_indices); \
		_mm512_storeu_epi32(ary_indx.data(), indices); \
		_mm512_storeu_ps(ary_dist.data(), distances); \
	}

	shuffle_once(1, 0, 3, 2, 5, 4, 7, 6, 9, 8, 11, 10, 13, 12, 15, 14);
	
	shuffle_once(3, 2, 1, 0, 7, 6, 5, 4, 11, 10, 9, 8, 15, 14, 13, 12);
	shuffle_once(1, 0, 3, 2, 5, 4, 7, 6, 9, 8, 11, 10, 13, 12, 15, 14);

	shuffle_once(7, 6, 5, 4, 3, 2, 1, 0, 15, 14, 13, 12, 11, 10, 9, 8);
	shuffle_once(2, 3, 0, 1, 6, 7, 4, 5, 10, 11, 8, 9, 14, 15, 12, 13);
	shuffle_once(1, 0, 3, 2, 5, 4, 7, 6, 9, 8, 11, 10, 13, 12, 15, 14);

	shuffle_once(15, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0);
	shuffle_once(4, 5, 6, 7, 0, 1, 2, 3, 12, 13, 14, 15, 8, 9, 10, 11);
	shuffle_once(2, 3, 0, 1, 6, 7, 4, 5, 10, 11, 8, 9, 14, 15, 12, 13);
	shuffle_once(1, 0, 3, 2, 5, 4, 7, 6, 9, 8, 11, 10, 13, 12, 15, 14);
	
	return indices;

#undef shuffle_once

}
