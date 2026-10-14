// CUDA kernels behind PostProcessor and TemporalFilter (postprocess_kernels.cu). Raw device pointers only, so the
// .cu file needs no OpenCV and compiles under nvcc 10.2 (C++14). Disparity is CV_16SC1 = disparity * 16, invalid <= 0.
// Every launch is asynchronous on the given stream except label_components, which reads one flag back per iteration.
#ifndef DEPTHD_POSTPROCESS_KERNELS_HPP_
#define DEPTHD_POSTPROCESS_KERNELS_HPP_

#include <cuda_runtime.h>

#include <cstddef>

namespace depthd
{
namespace kernels
{

// Pitched 2D plane: base pointer and row pitch in bytes.
struct Disp
{
  short * ptr;
  size_t step;
};
struct CDisp
{
  const short * ptr;
  size_t step;
};
struct Grey
{
  const unsigned char * ptr;
  size_t step;
};

// What label_components groups: valid pixels whose neighbours differ by at most tol16, or invalid pixels (holes).
enum class Grouping
{
  kSimilarValid,
  kInvalid
};

// Left-right consistency: the pixel becomes 0 when its partner x - round(d/16) is outside the right image, the right
// disparity there is invalid, or |dL - dR| > tol16. Invalid pixels become 0.
void lr_check(Disp disp, CDisp right, int w, int h, int tol16, cudaStream_t stream);

// Confidence: invalid unless 16 <= d < (num_disparities - 1) * 16 (below 1 px or saturated at the search range) and the
// partner pixel x - round(d/16) lies inside the right image. Invalid pixels become 0.
void confidence(Disp disp, int w, int h, int num_disparities, cudaStream_t stream);

// 4-connected component labelling by label equivalence. labels and sizes are contiguous w*h ints. After the call
// sizes[labels[i]] is the component size of member pixel i; non-members keep label -1. flag_dev is one device int and
// flag_host one pinned host int. Throws std::runtime_error on a CUDA error.
void label_components(CDisp disp, Grouping grouping, int tol16, int w, int h, int * labels, int * sizes, int * flag_dev,
                      int * flag_host, cudaStream_t stream);

// Valid pixels whose component (from label_components with kSimilarValid) is smaller than max_size become 0.
void speckle_apply(Disp disp, const int * labels, const int * sizes, int w, int h, int max_size, cudaStream_t stream);

// Writes out = disp, with every invalid component (from label_components with kInvalid) of at most max_hole_px pixels
// filled from the first valid pixel found in each of the 4 directions whose left-image grey is closest to the pixel
// (ties: the smaller disparity). Other invalid pixels are written as 0. in and out must not alias.
void hole_fill(CDisp in, Disp out, Grey left, const int * labels, const int * sizes, int w, int h, int max_hole_px,
               cudaStream_t stream);

// Motion-gated EMA. state (float, contiguous w*h) holds the filtered disparity (<= 0: no history) and prev_grey
// (contiguous w*h) the previous grey frame; both are updated. first ignores both (no usable history).
void temporal(Disp disp, Grey grey, float * state, unsigned char * prev_grey, int w, int h, float alpha,
              int motion_threshold, bool first, cudaStream_t stream);

}  // namespace kernels
}  // namespace depthd

#endif  // DEPTHD_POSTPROCESS_KERNELS_HPP_
