// Test helper: counts cv::cuda::GpuMat allocations made while it is in scope (it wraps the default allocator).
// A per-frame stage that allocates pays a cudaMalloc and a device-synchronising cudaFree on every frame.
#ifndef DEPTHD_TEST_GPU_ALLOC_COUNTER_HPP_
#define DEPTHD_TEST_GPU_ALLOC_COUNTER_HPP_

#include <opencv2/core/cuda.hpp>

class GpuAllocCounter : public cv::cuda::GpuMat::Allocator
{
public:
  GpuAllocCounter() : inner_(cv::cuda::GpuMat::defaultAllocator()) { cv::cuda::GpuMat::setDefaultAllocator(this); }
  ~GpuAllocCounter() override { cv::cuda::GpuMat::setDefaultAllocator(inner_); }
  GpuAllocCounter(const GpuAllocCounter &) = delete;
  GpuAllocCounter & operator=(const GpuAllocCounter &) = delete;

  bool allocate(cv::cuda::GpuMat * mat, int rows, int cols, size_t elem_size) override
  {
    ++count;
    mat->allocator = inner_;  // freed by the allocator that made it, even after this counter is gone
    return inner_->allocate(mat, rows, cols, elem_size);
  }
  void free(cv::cuda::GpuMat * mat) override { inner_->free(mat); }

  int count = 0;

private:
  cv::cuda::GpuMat::Allocator * inner_;
};

#endif  // DEPTHD_TEST_GPU_ALLOC_COUNTER_HPP_
