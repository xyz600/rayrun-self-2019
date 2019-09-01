#pragma once

#include <intrin.h>

__m512i bitonic_sort(__m512i indices, __m512 distances) {
	
#define shuffle_once(i0, i1, i2, i3, i4, i5, i6, i7, i8, i9, i10, i11, i12, i13, i14, i15, p0, p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11, p12, p13, p14, p15) \
	{\
		__m512i pattern = _mm512_setr_epi32(i0, i1, i2, i3, i4, i5, i6, i7, i8, i9, i10, i11, i12, i13, i14, i15); \
		__m512i shuffled_indices = _mm512_permutexvar_epi32(pattern, indices); \
		__m512 shuffled_distances = _mm512_permutexvar_ps(pattern, distances); \
		__mmask16 cmp = _mm512_cmple_ps_mask(distances, shuffled_distances); \
		__m512i enlarged_cmp = _mm512_movm_epi32(cmp); \
		__m512i selected_position = _mm512_setr_epi32(p0, p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11, p12, p13, p14, p15); \
		__mmask16 mask = _mm512_movepi32_mask(_mm512_permutexvar_epi32(selected_position, enlarged_cmp)); \
		distances = _mm512_mask_blend_ps(mask, shuffled_distances, distances); \
		indices = _mm512_mask_blend_epi32(mask, shuffled_indices, indices); \
	}

	shuffle_once(1, 0, 3, 2, 5, 4, 7, 6, 9, 8, 11, 10, 13, 12, 15, 14, 0, 0, 2, 2, 4, 4, 6, 6, 8, 8, 10, 10, 12, 12, 14, 14);
	
	shuffle_once(3, 2, 1, 0, 7, 6, 5, 4, 11, 10, 9, 8, 15, 14, 13, 12, 0, 1, 1, 0, 4, 5, 5, 4, 8, 9, 9, 8, 12, 13, 13, 12);
	shuffle_once(1, 0, 3, 2, 5, 4, 7, 6, 9, 8, 11, 10, 13, 12, 15, 14, 0, 0, 2, 2, 4, 4, 6, 6, 8, 8, 10, 10, 12, 12, 14, 14);

	shuffle_once(7, 6, 5, 4, 3, 2, 1, 0, 15, 14, 13, 12, 11, 10, 9, 8, 0, 1, 2, 3, 3, 2, 1, 0, 8, 9, 10, 11, 11, 10, 9, 8);
	shuffle_once(2, 3, 0, 1, 6, 7, 4, 5, 10, 11, 8, 9, 14, 15, 12, 13, 0, 1, 0, 1, 4, 5, 4, 5, 8, 9, 8, 9, 12, 13, 12, 13);
	shuffle_once(1, 0, 3, 2, 5, 4, 7, 6, 9, 8, 11, 10, 13, 12, 15, 14, 0, 0, 2, 2, 4, 4, 6, 6, 8, 8, 10, 10, 12, 12, 14, 14);

	shuffle_once(15, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0, 0, 1, 2, 3, 4, 5, 6, 7, 7, 6, 5, 4, 3, 2, 1, 0);
	shuffle_once(4, 5, 6, 7, 0, 1, 2, 3, 12, 13, 14, 15, 8, 9, 10, 11, 0, 1, 2, 3, 0, 1, 2, 3, 8, 9, 10, 11, 8, 9, 10, 11);
	shuffle_once(2, 3, 0, 1, 6, 7, 4, 5, 10, 11, 8, 9, 14, 15, 12, 13, 0, 1, 0, 1, 4, 5, 4, 5, 8, 9, 8, 9, 12, 13, 12, 13);
	shuffle_once(1, 0, 3, 2, 5, 4, 7, 6, 9, 8, 11, 10, 13, 12, 15, 14, 0, 0, 2, 2, 4, 4, 6, 6, 8, 8, 10, 10, 12, 12, 14, 14);
	
	return indices;

#undef shuffle_once

}
