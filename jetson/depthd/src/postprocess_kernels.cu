// Kernels for the GPU post-processing stack and the temporal filter. See postprocess_kernels.hpp.
#include <cuda_runtime.h>

#include <stdexcept>
#include <string>

#include "depthd/postprocess_kernels.hpp"

namespace depthd
{
namespace kernels
{
namespace
{

constexpr int kBlockX = 32;
constexpr int kBlockY = 8;
constexpr int kMaxLabelIterations = 4096;

inline dim3 grid_for(int w, int h) { return dim3((w + kBlockX - 1) / kBlockX, (h + kBlockY - 1) / kBlockY); }
inline dim3 block() { return dim3(kBlockX, kBlockY); }

void check(cudaError_t e, const char * what)
{
  if (e != cudaSuccess) throw std::runtime_error(std::string(what) + ": " + cudaGetErrorString(e));
}

__device__ inline short * row(Disp d, int y) { return reinterpret_cast<short *>(reinterpret_cast<char *>(d.ptr) + y * d.step); }
__device__ inline const short * row(CDisp d, int y)
{
  return reinterpret_cast<const short *>(reinterpret_cast<const char *>(d.ptr) + y * d.step);
}
__device__ inline const unsigned char * row(Grey g, int y) { return g.ptr + y * g.step; }

// x - round(d / 16)
__device__ inline int partner_x(int x, int d16) { return x - ((d16 + 8) >> 4); }

__global__ void lr_check_kernel(Disp disp, CDisp right, int w, int h, int tol16)
{
  const int x = blockIdx.x * blockDim.x + threadIdx.x, y = blockIdx.y * blockDim.y + threadIdx.y;
  if (x >= w || y >= h) return;
  short * p = row(disp, y) + x;
  const int d = *p;
  if (d <= 0) {
    *p = 0;
    return;
  }
  const int xr = partner_x(x, d);
  bool ok = false;
  if (xr >= 0) {
    const int r = row(right, y)[xr];
    ok = r > 0 && abs(d - r) <= tol16;
  }
  if (!ok) *p = 0;
}

__global__ void confidence_kernel(Disp disp, int w, int h, int max16)
{
  const int x = blockIdx.x * blockDim.x + threadIdx.x, y = blockIdx.y * blockDim.y + threadIdx.y;
  if (x >= w || y >= h) return;
  short * p = row(disp, y) + x;
  const int d = *p;
  if (d < 16 || d >= max16 || partner_x(x, d) < 0) *p = 0;
}

__device__ inline bool in_group(Grouping g, int d) { return g == Grouping::kInvalid ? d <= 0 : d > 0; }
__device__ inline bool linked(Grouping g, int a, int b, int tol16)
{
  return g == Grouping::kInvalid ? (a <= 0 && b <= 0) : (a > 0 && b > 0 && abs(a - b) <= tol16);
}

__global__ void label_init_kernel(CDisp disp, Grouping g, int w, int h, int * labels)
{
  const int x = blockIdx.x * blockDim.x + threadIdx.x, y = blockIdx.y * blockDim.y + threadIdx.y;
  if (x >= w || y >= h) return;
  labels[y * w + x] = in_group(g, row(disp, y)[x]) ? y * w + x : -1;
}

// Hooking: point the root of the pixel's tree at the smallest root among its linked neighbours.
__global__ void label_scan_kernel(CDisp disp, Grouping g, int tol16, int w, int h, int * labels, int * changed)
{
  const int x = blockIdx.x * blockDim.x + threadIdx.x, y = blockIdx.y * blockDim.y + threadIdx.y;
  if (x >= w || y >= h) return;
  const int i = y * w + x;
  const int own = labels[i];
  if (own < 0) return;
  const int d = row(disp, y)[x];
  int best = own;
  if (x > 0 && linked(g, d, row(disp, y)[x - 1], tol16)) best = min(best, labels[i - 1]);
  if (x + 1 < w && linked(g, d, row(disp, y)[x + 1], tol16)) best = min(best, labels[i + 1]);
  if (y > 0 && linked(g, d, row(disp, y - 1)[x], tol16)) best = min(best, labels[i - w]);
  if (y + 1 < h && linked(g, d, row(disp, y + 1)[x], tol16)) best = min(best, labels[i + w]);
  if (best < own) {
    atomicMin(&labels[own], best);
    *changed = 1;
  }
}

// Pointer jumping: every member points straight at its root.
__global__ void label_analyze_kernel(int n, int * labels)
{
  const int i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i >= n || labels[i] < 0) return;
  int l = labels[i];
  while (l != labels[l]) l = labels[l];
  labels[i] = l;
}

__global__ void label_count_kernel(int n, const int * labels, int * sizes)
{
  const int i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i < n && labels[i] >= 0) atomicAdd(&sizes[labels[i]], 1);
}

__global__ void disparity_to_depth_kernel(CDisp disp, unsigned short * out, size_t out_step, int w, int h,
                                          float numerator, int min_mm, int max_mm)
{
  const int x = blockIdx.x * blockDim.x + threadIdx.x, y = blockIdx.y * blockDim.y + threadIdx.y;
  if (x >= w || y >= h) return;
  const int d = row(disp, y)[x];
  int z = 0;
  if (d > 0) {
    z = min(max(__float2int_rn(__fdiv_rn(numerator, static_cast<float>(d))), 0), 65535);
    if (z < min_mm || z > max_mm) z = 0;
  }
  reinterpret_cast<unsigned short *>(reinterpret_cast<char *>(out) + y * out_step)[x] = static_cast<unsigned short>(z);
}

__global__ void speckle_apply_kernel(Disp disp, const int * labels, const int * sizes, int w, int h, int max_size)
{
  const int x = blockIdx.x * blockDim.x + threadIdx.x, y = blockIdx.y * blockDim.y + threadIdx.y;
  if (x >= w || y >= h) return;
  short * p = row(disp, y) + x;
  if (*p > 0 && sizes[labels[y * w + x]] < max_size) *p = 0;
}

__global__ void hole_fill_kernel(CDisp in, Disp out, Grey left, const int * labels, const int * sizes, int w, int h,
                                 int max_hole_px)
{
  const int x = blockIdx.x * blockDim.x + threadIdx.x, y = blockIdx.y * blockDim.y + threadIdx.y;
  if (x >= w || y >= h) return;
  const short d = row(in, y)[x];
  short result = d;
  if (d <= 0) {
    result = 0;
    if (sizes[labels[y * w + x]] <= max_hole_px) {
      const int grey = row(left, y)[x];
      int best_cost = 1 << 30, best_d = 0;
      for (int dir = 0; dir < 4; ++dir) {
        const int dx = dir == 0 ? -1 : dir == 1 ? 1 : 0, dy = dir == 2 ? -1 : dir == 3 ? 1 : 0;
        int cx = x + dx, cy = y + dy;
        for (int step = 0; step < max_hole_px + 1 && cx >= 0 && cx < w && cy >= 0 && cy < h; ++step) {
          const int c = row(in, cy)[cx];
          if (c > 0) {
            const int cost = abs(grey - static_cast<int>(row(left, cy)[cx]));
            if (cost < best_cost || (cost == best_cost && c < best_d)) {
              best_cost = cost;
              best_d = c;
            }
            break;
          }
          cx += dx;
          cy += dy;
        }
      }
      result = static_cast<short>(best_d);
    }
  }
  row(out, y)[x] = result;
}

__global__ void temporal_kernel(Disp disp, Grey grey, float * state, unsigned char * prev_grey, int w, int h,
                                float alpha, int motion_threshold, bool first)
{
  const int x = blockIdx.x * blockDim.x + threadIdx.x, y = blockIdx.y * blockDim.y + threadIdx.y;
  if (x >= w || y >= h) return;
  const int i = y * w + x;
  short * p = row(disp, y) + x;
  const int g = row(grey, y)[x];
  const int pg = prev_grey[i];
  prev_grey[i] = static_cast<unsigned char>(g);
  const int d = *p;
  if (d <= 0) {  // an invalid pixel stays invalid and loses its history
    state[i] = 0.f;
    return;
  }
  const float s = state[i];
  float f = static_cast<float>(d);
  if (!first && s > 0.f && abs(g - pg) <= motion_threshold) f = alpha * f + (1.f - alpha) * s;
  state[i] = f;
  *p = static_cast<short>(max(1, __float2int_rn(f)));
}

}  // namespace

void lr_check(Disp disp, CDisp right, int w, int h, int tol16, cudaStream_t stream)
{
  lr_check_kernel<<<grid_for(w, h), block(), 0, stream>>>(disp, right, w, h, tol16);
  check(cudaGetLastError(), "lr_check launch");
}

void confidence(Disp disp, int w, int h, int num_disparities, cudaStream_t stream)
{
  confidence_kernel<<<grid_for(w, h), block(), 0, stream>>>(disp, w, h, (num_disparities - 1) * 16);
  check(cudaGetLastError(), "confidence launch");
}

void label_components(CDisp disp, Grouping grouping, int tol16, int w, int h, int * labels, int * sizes, int * flag_dev,
                      int * flag_host, cudaStream_t stream)
{
  const int n = w * h;
  const int threads = 256, blocks = (n + threads - 1) / threads;
  label_init_kernel<<<grid_for(w, h), block(), 0, stream>>>(disp, grouping, w, h, labels);
  for (int it = 0; it < kMaxLabelIterations; ++it) {
    check(cudaMemsetAsync(flag_dev, 0, sizeof(int), stream), "label flag clear");
    label_scan_kernel<<<grid_for(w, h), block(), 0, stream>>>(disp, grouping, tol16, w, h, labels, flag_dev);
    label_analyze_kernel<<<blocks, threads, 0, stream>>>(n, labels);
    check(cudaMemcpyAsync(flag_host, flag_dev, sizeof(int), cudaMemcpyDeviceToHost, stream), "label flag read");
    check(cudaStreamSynchronize(stream), "label sync");
    if (*flag_host == 0) break;
  }
  check(cudaMemsetAsync(sizes, 0, sizeof(int) * n, stream), "label sizes clear");
  label_count_kernel<<<blocks, threads, 0, stream>>>(n, labels, sizes);
  check(cudaGetLastError(), "label launch");
}

void speckle_apply(Disp disp, const int * labels, const int * sizes, int w, int h, int max_size, cudaStream_t stream)
{
  speckle_apply_kernel<<<grid_for(w, h), block(), 0, stream>>>(disp, labels, sizes, w, h, max_size);
  check(cudaGetLastError(), "speckle launch");
}

void hole_fill(CDisp in, Disp out, Grey left, const int * labels, const int * sizes, int w, int h, int max_hole_px,
               cudaStream_t stream)
{
  hole_fill_kernel<<<grid_for(w, h), block(), 0, stream>>>(in, out, left, labels, sizes, w, h, max_hole_px);
  check(cudaGetLastError(), "hole_fill launch");
}

void disparity_to_depth(CDisp disp, unsigned short * out, size_t out_step, int w, int h, float numerator, int min_mm,
                        int max_mm, cudaStream_t stream)
{
  disparity_to_depth_kernel<<<grid_for(w, h), block(), 0, stream>>>(disp, out, out_step, w, h, numerator, min_mm,
                                                                     max_mm);
  check(cudaGetLastError(), "disparity_to_depth launch");
}

void temporal(Disp disp, Grey grey, float * state, unsigned char * prev_grey, int w, int h, float alpha,
              int motion_threshold, bool first, cudaStream_t stream)
{
  temporal_kernel<<<grid_for(w, h), block(), 0, stream>>>(disp, grey, state, prev_grey, w, h, alpha, motion_threshold,
                                                          first);
  check(cudaGetLastError(), "temporal launch");
}

}  // namespace kernels
}  // namespace depthd
