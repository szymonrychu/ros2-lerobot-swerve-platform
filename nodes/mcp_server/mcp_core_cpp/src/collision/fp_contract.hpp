// Included by every .cpp of the collision family that does arithmetic: no floating-point contraction (fused
// multiply-add) in this translation unit, so a sum of products rounds after every operation as numpy does and a value
// gets the same bits whichever kernel (pruned or full, C++ or the numpy reference) computes it.
#pragma once

#if defined(__clang__)
#pragma STDC FP_CONTRACT OFF
#elif defined(__GNUC__)
#pragma GCC optimize("fp-contract=off")
#endif
