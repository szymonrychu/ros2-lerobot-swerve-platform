# Stereo depth on Jetson Nano 4GB B01 (JetPack 4.6 / L4T R32.6.1) - research brief

Date: 2026-10-14. Target platform: Jetson Nano 4GB B01, Maxwell GM20B 128 cores (sm_53),
4 GB shared LPDDR4 (25.6 GB/s), L4T R32.6.1, CUDA 10.2, cuDNN 8.2.1, TensorRT 8.0.1,
VPI 1.1, Python 3.6, two IMX219 CSI sensors.

Legend: [V] verified from a primary source in this session; [S] secondary/community source;
[E] my estimate (derived, not measured - must be benchmarked on the board).

## TL;DR

- Realistic target: 640x480 (or 640x360), 64-128 disparities, SGM on GPU, about 10-15 fps
  end to end is plausible [E]; 320x240 at 25-30 fps is easy [E]. No published Nano
  number exists for libSGM, cv::cuda::StereoSGM or VPI 1.1 at 640x480 - benchmark first.
- Best classic engine: census/SGM on CUDA. libSGM (or its OpenCV port cv::cuda::StereoSGM)
  gives left-right check, uniqueness and subpixel; VPI 1.1 stereo exposes only windowSize and
  maxDisparity and an undocumented confidence map.
- Learned stereo is not a primary path on Nano: the only measured Nano stereo net found
  (SwitchStereo) does 7 fps at 320x640 and needs TensorRT 8.2.1 (JetPack 4.6.1). Most modern
  nets (CREStereo, RAFT-Stereo, IGEV, Fast-ACV) need GridSample, which TensorRT only parses
  from 8.5 - a version that never ships for Nano.
- Monocular depth (Depth Anything v2 small etc.) is useful only as a slow (1-5 Hz [E]),
  scale-aligned hole filler for textureless regions, not as the primary depth source.
- Consider moving to JetPack 4.6.1+ (4.6.6 is the last for Nano): TensorRT 8.2.1, VPI 1.2
  with production Python bindings, same CUDA 10.2.

## 1. Capture

### Two sensors with nvarguscamerasrc
- Open each sensor as its own pipeline: `nvarguscamerasrc sensor-id=0` and `sensor-id=1`
  [S: NVIDIA forum 156807]. JetPack 4.6 added run-time IMX219/IMX477 selection via jetson-io
  on Nano B01 [V: JetPack 4.6 release notes].
- IMX219 sensor modes on Jetson (from the stock driver; verify with `nvarguscamerasrc` log
  output): 3264x2464@21, 3264x1848@28, 1920x1080@30, 1640x1232@30 (2x2 binned, full FOV),
  1280x720@60 (cropped) [E from driver tables, verify]. For stereo prefer the full-FOV binned
  1640x1232 mode, downscaled to 640x480 by nvvidconv (VIC) inside NVMM: binning lowers noise
  and keeps FOV.
- Lock the two ISPs to identical exposure, gain and white balance (`aelock`, `awblock`,
  `exposuretimerange`, `gainrange`, `ispdigitalgainrange`) so census costs are not biased by
  per-camera auto-exposure. Census is robust to gain offsets, but not to different motion blur.
  Keep `ee-mode` (edge enhancement) off or low and `tnr-mode` modest: sharpening halos and
  temporal denoise smear create false matches [E, standard practice].

### Synchronisation limits
- IMX219 has no hardware sync / trigger pin [S: NVIDIA forum 256831]. Two independently
  free-running sensors drift; software-only sync is approximate, and IMX219 is rolling
  shutter, so there is no guarantee of exact exposure alignment [S: Arducam blog].
- Best software option: the Argus `syncSensor` sample (Jetson Multimedia API) issues one
  capture request to both sensors from one session - "close enough", not exact [S: Arducam
  blog, NVIDIA forum 256831]. nvarguscamerasrc with two pipelines does not do this.
- Timestamp pairing: nvarguscamerasrc attaches a small struct (frame number + timestamp, ns,
  CLOCK_MONOTONIC based, start-of-frame in kernel) as GstBuffer qdata with quark
  `GstBufferMetaData`, readable from a pad probe on the src pad; it is lost after some
  elements (e.g. videorate) [S: NVIDIA forums 182510, 158969, 199563]. Pair left/right by
  nearest timestamp and drop pairs with delta > ~1/4 frame period (about 8 ms at 30 fps) [E].
- Hardware alternative: Arducam stereo HAT / Waveshare-style boards that share the clock and
  broadcast I2C - vendor claim, not verified [S: Arducam blog]. For a moving robot, residual
  skew of a few ms at 30 fps is a real error source at speed; mitigate with short exposure
  (bright scenes / outdoors) and by slowing down near grasp targets.

### Zero copy into CUDA / VPI
- On R32 the NVMM path is nvbuf_utils (not NvBufSurface, which is JetPack 5): appsink buffer
  -> `ExtractFdFromNvBuffer` -> `NvEGLImageFromFd` -> `cuGraphicsEGLRegisterImage` -> CUDA
  device pointer. dusty-nv jetson-utils (`videoSource("csi://0")`, C++ and Python) already
  implements this and supports JetPack 4.6 - reuse it rather than re-writing it [S].
- Wrap the device pointer as a VPI image (`vpiImageCreateCUDAMemWrapper`, VPI 1.1 C API) or
  `cv::cuda::GpuMat(rows, cols, CV_8UC1, ptr, pitch)`, or pass it straight to libSGM (it
  accepts device input/output). Python VPI in 1.1 is a "developer preview" [V: JetPack 4.6
  release notes]; production Python bindings arrived with VPI 1.2 / JetPack 4.6.1 [S:
  JetPack 4.6.1 notes]. Recommend C++ for the hot loop, exposing results to Python/ROS.

### Rectification
- Calibrate once (OpenCV `stereoCalibrate` + `stereoRectify` + `initUndistortRectifyMap`,
  stock JetPack OpenCV 4.1.1 CPU is enough). Rectification quality dominates result quality:
  a Nano user got "almost pure noise" from VPI stereo on a Waveshare IMX219 pair until
  rectifying; NVIDIA's answer: the algorithm expects rectified pairs, and rectifying "greatly
  improves" results [V: NVIDIA forum 202240].
- Per-frame remap options on Nano:
  - VPI 1.1 Remap: CUDA backend on Nano; VIC backend for Remap is Xavier-only; build a dense
    `vpi.WarpMap` / `VPIWarpMap` and copy the OpenCV map coordinates into its keypoints [V:
    VPI 1.1 Remap docs]. Formats U8/Y8/NV12_ER etc.
  - NPP `nppiRemap_8u_C1R` (ships with CUDA 10.2, no OpenCV build needed) [E: standard NPP].
  - `cv::cuda::remap` (needs OpenCV built with CUDA).
  - Remap at 640x480 U8 is a few ms on GPU [E]; do it on the Y plane only (grayscale).

## 2. Classic stereo options

### VPI 1.1 stereo disparity [V: VPI 1.1 and 1.2 docs]
- Backends: CUDA and CPU on Nano. PVA is "only available on Jetson Xavier devices"; VIC not
  implemented; PVA+NVENC+VIC (1920x1080 only) needs PVA, so also Xavier-only.
- CUDA: maxDisparity <= 256, downscale factor 1 (output = input size), no fixed resolution.
  CPU: maxDisparity <= 64, no confidence map.
- Inputs U8, U16, Y8, NV12, NV24 (+_ER variants). Outputs: disparity and confidence both U16;
  disparity is Q-format with 5 fractional bits (display scale 1/(32*maxDisparity)*255).
- Parameters exposed in 1.1/1.2: only `windowSize` (census window) and `maxDisparity`. No
  P1/P2, uniqueness, LR check or confidence threshold. Confidence-map value semantics are not
  documented; a Nano user had to hand-tune a threshold (327670) because the Python API did not
  expose the quality parameters [V: forum 202240]. A forum thread reports a zero disparity
  image when the confidence flag is passed (forum 195010) [S].
- Nano timing: the only published Nano number is VPI 0.2 at 480x270: CUDA 54.7 ms (64
  disparities), 51.3 ms (32) [V: VPI 0.2 docs]. VPI 1.1 release notes claim about 2.2x faster
  CUDA stereo, measured on AGX Xavier [S, page fetch blocked]. Extrapolated VPI 1.1 at
  640x480x64: roughly 50-70 ms (14-20 fps for disparity alone) [E]. The VPI 1.1 performance
  table for Nano exists on the docs page but did not render - read it on the board or with a
  JS browser before trusting [unverified].

### libSGM (fixstars) [V: repo README]
- CUDA SGM, Apache 2.0. Releases 3.0.0 (2022-12) and 3.1.0 (2023-08). Requires CMake >= 3.18
  (Ubuntu 18.04 ships 3.10 -> `pip3 install cmake` or a Kitware build), compute capability
  >= 3.5 (sm_53 fine), OpenCV >= 3.4.8 only for samples / the OpenCV wrapper.
- Features: 4 or 8 paths, subpixel, uniqueness, LR consistency check, min disparity,
  disparity size 64/128/256, device-pointer I/O (zero copy).
- Published timing (1024x440, 128 disp, 4 paths, subpixel): Tegra X2 28.5 ms, Xavier 15W
  17.3 ms. No Nano figure published anywhere I found.
- Nano estimate: SGM is memory-bandwidth bound; TX2 has 59.7 GB/s and 256 Pascal cores vs
  Nano 25.6 GB/s and 128 Maxwell cores -> about 2-2.5x slower than TX2. 1024x440x128 ~ 60-70
  ms; 640x480x128 ~ 40-50 ms; 640x480x64 ~ 20-30 ms; 320x240x64 ~ 6-8 ms [E].

### OpenCV CUDA: cv::cuda::StereoSGM / StereoBM [V: opencv_contrib source]
- `cv::cuda::StereoSGM` was added to opencv_contrib cudastereo by commit 7883ec8 (Dec 2018)
  and first ships in release 4.5.1 (absent at tags 4.4.0 and 4.5.0, present at 4.5.1/4.5.4/
  4.5.5). It is a port of the libSGM approach: input CV_8UC1/CV_16UC1, numDisparities must be
  64, 128 or 256, MODE_HH4 (4 paths, default) or MODE_HH (8 paths), P1/P2, uniquenessRatio,
  output 16-bit fixed point with 4 fractional bits. Python-wrapped (`CV_EXPORTS_W`).
- `cv::cuda::StereoBM` (block matching) is faster but much noisier on low texture; use only for
  a coarse obstacle fallback. Also in cudastereo: `DisparityBilateralFilter` (GPU
  edge-preserving disparity refinement), StereoBeliefPropagation, StereoConstantSpaceBP
  (slow on Nano [E]).
- JetPack 4.6 ships OpenCV 4.1.1 without CUDA [S, widely reported; not in the JP 4.6 release
  notes page] - so neither cudastereo nor contrib (ximgproc WLS) is available without a build.

### Post-filtering (ordered by value per ms on Nano)
1. Left-right consistency + uniqueness ratio inside SGM (libSGM / cv::cuda::StereoSGM) - the
   single largest noise reduction; VPI 1.1 lacks it (only the opaque confidence map).
2. Speckle filter: `cv::filterSpeckles` (CPU, core OpenCV 4.1.1) on 640x480 is a few ms
   [E]; tune maxSpeckleSize ~ 100-200 px, maxDiff ~ 1 px * 16.
3. Edge-aware hole fill / smoothing guided by the left image:
   - `cv::cuda::DisparityBilateralFilter` (needs OpenCV CUDA build).
   - `cv::ximgproc::DisparityWLSFilter` / `fastGlobalSmootherFilter` (contrib, CPU only):
     WLS at 640x480 on Cortex-A57 is likely 50-150 ms [E]; run at 320x240 or skip.
   - A custom CUDA guided/joint-bilateral filter (~1-3 ms [E]) is the cheapest good option.
4. Temporal filtering: confidence-weighted EMA per pixel, reset where |d_t - d_{t-1}| is large
   (motion). Only helpful when the robot is slow or still (grasping); with ego-motion it needs
   warping by odometry, which is not worth it on Nano [E].
5. For collision avoidance prefer invalid over wrong: keep holes, and treat "unknown" cells
   conservatively in the costmap rather than hallucinating fill.

### Geometry sanity (pick max disparity from it) [E]
- f_px = (W/2)/tan(HFOV/2). For 640 px wide and ~77 deg HFOV (typical IMX219 M12 lens), f ~ 400
  px. With baseline B = 60 mm (common on IMX219-83 boards - verify yours), Z_min = f*B/d_max:
  64 disp -> 0.37 m, 128 -> 0.19 m. Grasp interpretation at 0.2-0.4 m needs 128 disparities
  (or a minDisparity offset, which loses far range). Depth error dZ = Z^2/(f*B)*dd: with
  0.25 px subpixel, 1 m -> ~1 cm, 3 m -> ~9 cm.

## 3. Building OpenCV with CUDA on JetPack 4.6

- Scripts: Qengineering `Install-OpenCV-Jetson-Nano` (scripts for 4.5.0, 4.5.5, 4.6.0 ...
  4.14.0; auto-detects Nano and sets `CUDA_ARCH_BIN=5.3`, `CUDA_ARCH_PTX=sm_53`, WITH_CUDA,
  WITH_CUDNN, WITH_CUBLAS, CUDA_FAST_MATH) [V: repo]. Memory: needs >= 6.5 GB RAM+swap for
  4.5.x and >= 8.5 GB for 4.6.0+; with less, the script falls back to `make -j1` [V: README].
  Also JetsonHacks `buildOpenCV` and mdegans `nano_build_opencv` (older) [S].
- Version for JetPack 4.6 / CUDA 10.2: 4.5.5 or 4.6.0 with contrib is the sweet spot [E];
  4.5.1 is the minimum for cuda::StereoSGM. Newer 4.x releases still build against CUDA 10.2
  per Qengineering, but Python 3.6 + numpy constraints make 4.5.5-4.6.0 the safe pick [E].
- Build time: about 2-3 h with 4 jobs and enough swap, much longer at -j1 [S, commonly
  reported; not stated in the README]. Add a swapfile on SSD/USB, not SD, if possible.
- Does it beat VPI here? For stereo yes, on features: cuda::StereoSGM has P1/P2, uniqueness,
  8-path mode, plus DisparityBilateralFilter and contrib WLS. But libSGM gives the same SGM
  core without a multi-hour OpenCV build, and NPP covers remap. OpenCV CUDA is worth building
  only if you also want cudastereo filters, ximgproc and cv2.cuda from Python.

## 4. Learned options on Nano (TensorRT 8.0)

### Hard platform limits [V: onnx-tensorrt operator tables]
- TensorRT 8.0 parses ONNX up to opset 13. Not supported in 8.0: Einsum, GatherND, ScatterND,
  GridSample. 8.2 adds Einsum and ScatterND (still opset 13). GridSample arrives only in the 8.5
  ONNX parser. JetPack 4.x tops out at TensorRT 8.2.1 [V: JetPack 4.6.1 notes], so GridSample
  never parses natively on Nano.
- Consequence: iterative/warping nets (RAFT-Stereo, CREStereo, IGEV-Stereo, Fast-ACVNet,
  Metric3D's refinement) need a GridSample plugin or rewriting the warp as gather-based bilinear
  sampling - and even then are far too heavy for 128 Maxwell cores [E: well under 1 fps at
  useful resolution]. LayerNormalization is not a native ONNX op for 8.x either; exporters
  decompose it (fine).
- Memory: 4 GB shared with CPU, camera buffers and ROS; TensorRT engine build on Nano itself
  is required (engines are not portable across TRT versions/GPUs).

### Stereo networks - measured / estimated on Nano
- SwitchStereo (2025, resource-aware): Jetson Nano 4GB, TensorRT 8.2.1 FP16, 320x640, MAXN:
  fast engine 143.5 ms (6.97 fps, D1-all 4.25% KITTI full), balanced 410.8 ms (2.43 fps),
  155-203 MiB engine memory [V: repo README]. Requires JetPack 4.6.1 (TRT 8.2.1). Best measured
  learned stereo on Nano found; KITTI-trained (outdoor driving) - indoor domain gap.
- HITNet: ONNX exports exist (ibaiGorordo; 103 ms on a GTX 1660 Super) [S]; OmniNxt reports
  160 ms -> 50 ms with FP16 tensor cores on Orin NX class [S]. Nano (no tensor cores, ~1/10
  the compute) estimated 0.5-2 s per frame at 640x480 [E]. TF-exported graphs often contain
  GatherND -> fails on TRT 8.0 [E: check with `polygraphy inspect`].
- AnyNet / StereoNet / MobileStereoNet / StereoVAE / ESMStereo / HAGVNet: published numbers
  are TX2, AGX or Orin [S]; no Nano measurement found. AnyNet stage-1 / StereoNet-8x at
  320x240 are the only plausible >5 fps candidates [E], and quality at that operating point
  is roughly SGM-level.
- NVIDIA ESS / Isaac ROS DNN stereo: JetPack 5+/Orin only - not an option [S].
- Verdict: no learned stereo net beats GPU SGM + good post-filtering on Nano at 10-15 fps.

### Monocular depth (refinement / hole filling)
- Measured on Nano [S, papers]: RT-MonoDepth 18.4 fps and RT-MonoDepth-S 30.5 fps at 640x192
  (KITTI) [V: arXiv 2308.10569 abstract]; FastDepth 59-63 fps at 224x224 (NYU indoor, TRT)
  [S]; guided-decoding net up to 35 fps (NYU) [S]. These are domain-trained, metric-ish
  only in-domain.
- MiDaS v2.1 small (EfficientNet-Lite3, 256x256): 53.9 fps on AGX Orin [S]; no Nano number.
  Estimate 8-15 fps FP16 at 256x256 on Nano [E]. CNN, exports cleanly to opset 11-13.
- Depth Anything v2 small (ViT-S + DPT, 24.8M params): Orin NX ~25 ms/image at 480x300 BF16
  (batch 4 in 98.6 ms) [S: arXiv 2603.01999]. No Nano number found. Nano has no tensor cores
  and ~472 GFLOPS FP16: estimate 3-6 fps at 252x252, under 2 fps at 518x518 [E]. Export at a
  fixed input size so the DINOv2 bicubic position-embedding resize is constant-folded (TRT 8.x
  Resize supports only nearest/linear) [V: operator table; E: folding approach].
- Metric3D v2 small: uses iterative refinement with grid sampling -> TRT 8.0 incompatible
  without plugins, and too heavy [E]. Not recommended.
- Scale alignment: relative (affine-invariant inverse depth) output can be aligned per frame by
  a robust least-squares fit of scale s and shift t in disparity space against confident SGM
  pixels (MiDaS-style scale-and-shift alignment; VI-Depth extends it with a dense learned
  scale map, arXiv 2303.12134) [S]. Use RANSAC/Huber, fit on LR-checked pixels only, then fill
  only invalid SGM pixels, and mark filled pixels as low-confidence. Works well indoors for
  planar textureless surfaces (walls, tables); unreliable at depth discontinuities and for thin
  obstacles.
- Learned SGM refinement (e.g. small residual CNN taking disparity + left image, StereoNet
  refinement block, DeepPruner-style refinement) is plausible at 320x240 but nothing ready-made
  targets Nano [E]; a classic GPU guided filter gets most of the benefit for ~1-3 ms.

## 5. Recommended pipeline

### Primary (best quality at a realistic rate) [E for all timings]
1. Capture: 2x IMX219 in 1640x1232@30 binned mode, locked AE/AWB/gain, edge enhancement off;
   jetson-utils (NVMM -> CUDA zero copy) or Argus syncSensor in C++; pair by
   `GstBufferMetaData` timestamp, drop pairs with skew > ~8 ms.
2. VIC (nvvidconv) downscale to 640x480 GRAY8/NV12 Y-plane in NVMM.
3. Rectify on GPU: NPP `nppiRemap` or VPI CUDA Remap with a precomputed map (~2-4 ms).
4. libSGM: 640x480, 128 disparities, 4 paths, subpixel, uniqueness ~0.95, LR check on
   (~40-50 ms). Drop to 64 disparities + 0.37 m near limit for 20+ fps when grasp range is not
   needed; or run 128 only on a cropped ROI near the gripper.
5. Post: speckle filter (CPU, few ms) + GPU guided/joint-bilateral hole fill limited to small
   holes (~2-3 ms); confidence-weighted temporal EMA only when the base is stationary.
6. Outputs: disparity -> depth (f*B/d) -> point cloud / costmap; keep an invalid mask.
   Expected: 640x480x128 about 12-15 fps; 640x480x64 about 18-25 fps; 320x240x64 30 fps.
7. Optional, async at 1-3 Hz: MiDaS-small (256x256) or Depth Anything v2 small (252x252)
   in TensorRT FP16, scale/shift-aligned to SGM, used only to fill large textureless holes
   for grasp-scene interpretation, never for collision safety.

### Fallback
- VPI 1.1 CUDA stereo (`windowSize=5`, `maxDisparity=64`, 640x480 or 480x270) + VPI remap,
  Python-callable, zero extra builds. Expect 10-20 fps at 640x480x64 [E], noisier, no LR check;
  threshold the confidence map empirically. Lowest-effort way to get a first depth map.
- Second fallback for raw speed: 320x240, 64 disparities, 4 paths - collision-avoidance
  quality only.

### Trade-offs
- Resolution vs disparity range vs fps: SGM cost scales ~W*H*D; 128 disparities double cost but
  halve the minimum range (needed for grasping).
- libSGM vs OpenCV CUDA: same core; libSGM avoids a 2-3 h OpenCV build, OpenCV adds filters and
  Python cv2.cuda.
- Upgrading to JetPack 4.6.1+/4.6.6: gains TRT 8.2.1 (needed for SwitchStereo; Einsum/ScatterND)
  and VPI 1.2 production Python; costs a reflash and revalidating the camera stack.
- GPU sharing: any DNN running concurrently steals cycles from SGM on the same 128 cores; run
  mono depth at low rate or on demand.
- Outdoors: sunlight helps texture and exposure (less motion blur); IMX219 rolling shutter and
  unsynced sensors hurt at speed; repeated textures (fences, grass) need the uniqueness check.

### Outdated / unverifiable flags
- No published Nano measurement exists for libSGM, cv::cuda::StereoSGM, VPI 1.1 stereo at
  640x480, HITNet, MiDaS-small or Depth Anything v2 - all such numbers above are [E].
- VPI 1.1 Nano performance table did not render; VPI 1.1 release notes (2.2x claim) were 403.
- IMX219 sensor-mode list and the 60 mm baseline are typical values; confirm on your hardware.
- JetPack 4.x is end-of-life (Nano not supported by JetPack 5/6); new model ecosystems
  (TRT >= 8.5 ops, PyTorch >= 1.11 exporters) increasingly assume newer stacks. Export ONNX on
  a desktop with opset 13 and build engines on the Nano.
- Python 3.6 limits pip wheels (numpy <= 1.19, onnx older releases); keep Python as a thin
  layer.

## Sources

- VPI 1.1 Stereo Disparity: https://archive.docs.nvidia.com/vpi/1.1/algo_stereo_disparity.html
- VPI 1.2 Stereo Disparity: https://archive.docs.nvidia.com/vpi/1.2/algo_stereo_disparity.html
- VPI 0.2 Stereo Disparity (Nano timings): https://docs.nvidia.com/vpi/0.2.0/algo_stereo_disparity.html
- VPI 1.1 Remap: https://archive.docs.nvidia.com/vpi/1.1/algo_remap.html
- VPI 1.1 release notes (fetch 403): https://docs.nvidia.com/vpi/1.1/release_notes.html
- JetPack 4.6 release notes: https://docs.nvidia.com/jetson/archives/jetpack-archived/jetpack-46/release-notes/index.html
- JetPack 4.6.1 release notes: https://docs.nvidia.com/jetson/archives/jetpack-archived/jetpack-461/release-notes/index.html
- JetPack 4.6.6: https://developer.nvidia.com/jetpack-sdk-466
- NVIDIA forum, VPI stereo real-world results on Nano: https://forums.developer.nvidia.com/t/vpi-stereo-disparity-real-world-results/202240
- NVIDIA forum, VPI confidence map zero output: https://forums.developer.nvidia.com/t/vpisubmitstereodisparityestimator-giving-zero-disparity-image-when-confidence-map-flag-is-passed/195010
- NVIDIA forum, Nano camera sync: https://forums.developer.nvidia.com/t/jetson-nano-camera-sync/256831
- NVIDIA forum, stereo IMX219 on Nano: https://forums.developer.nvidia.com/t/stero-vision-with-imx219-isp-camera-by-jetson-nano/156807
- NVIDIA forums, nvarguscamerasrc timestamp metadata: https://forums.developer.nvidia.com/t/what-time-point-does-nvarguscamerasrc-timestamp-metadata-represent/182510 , https://forums.developer.nvidia.com/t/nvarguscamerasrc-timestamp-from-buffer-metadata-jp-4-4-3-cameras-sinchronized/158969/4 , https://forums.developer.nvidia.com/t/nvarguscamerasrc-timestamping/199563
- Arducam, Jetson Nano stereo sync: https://www.arducam.com/blog/jetson-nano-stereo-camera-sync-issue-arducam/
- libSGM: https://github.com/fixstars/libSGM
- opencv_contrib cudastereo (StereoSGM header, commit 7883ec8, tags 4.5.0 vs 4.5.1): https://github.com/opencv/opencv_contrib/tree/4.x/modules/cudastereo
- Qengineering OpenCV for Nano: https://github.com/Qengineering/Install-OpenCV-Jetson-Nano
- onnx-tensorrt operator tables (branches 8.0-GA, 8.2-GA, 8.5-GA): https://github.com/onnx/onnx-tensorrt/blob/8.0-GA/docs/operators.md
- SwitchStereo (Nano measurements): https://github.com/konwnohing/SwitchStereo
- ONNX HITNet: https://github.com/ibaiGorordo/ONNX-Split-HITNET-Stereo-Depth-Estimation
- OmniNxt (HITNet FP16 timing): https://arxiv.org/abs/2403.20085
- SteROI-D (HITNet on Orin Nano): https://arxiv.org/abs/2502.09528
- StereoVAE: https://arxiv.org/abs/2305.11566 ; ESMStereo: https://arxiv.org/abs/2506.21091
- RT-MonoDepth (Nano fps): https://arxiv.org/abs/2308.10569
- FastDepth: https://arxiv.org/abs/1903.03273 ; guided decoding: https://arxiv.org/abs/2203.04206
- Depth Anything v2 on Orin NX (teacher-student nav): https://arxiv.org/abs/2603.01999
- VI-Depth (scale alignment of mono depth): https://arxiv.org/abs/2303.12134
- Matchstic depthmapper (Nano + IMX219-83, BM/SGBM/AANet, no fps): https://github.com/Matchstic/depthmapper
- ReS2tAC (SGM on TX1/TX2/Xavier): https://arxiv.org/abs/2106.07927

## Plan inputs

- reuse: dusty-nv jetson-utils `videoSource` (csi://) for NVMM -> CUDA zero-copy capture on R32.
- reuse: libSGM 3.1.0 (device-pointer API, LR check, subpixel, uniqueness) as the disparity core.
- reuse: NPP `nppiRemap` (CUDA 10.2) or VPI 1.1 CUDA Remap for rectification; OpenCV 4.1.1 CPU
  (stock) for calibration and `filterSpeckles`.
- reuse: VPI 1.1 CUDA stereo as the zero-build fallback / first baseline.
- reuse: Argus `syncSensor` sample pattern if timestamp pairing skew proves too large.
- do-not-build: a learned stereo network as the primary depth source (none hits 10 fps on Nano;
  most need GridSample, unsupported before TRT 8.5).
- do-not-build: VIC/PVA paths (Remap VIC and stereo PVA are Xavier-only).
- do-not-build: a full OpenCV CUDA build unless cudastereo filters / ximgproc are actually needed.
- do-not-build: Metric3D / RAFT-Stereo / CREStereo / IGEV ports to TRT 8.0.
- constraint: TRT 8.0 = ONNX opset <= 13, no Einsum/GatherND/ScatterND/GridSample; engines built on
  the Nano itself.
- constraint: VPI 1.1 stereo exposes only windowSize/maxDisparity, U16 output with 5 fractional
  bits; CUDA maxDisparity <= 256; Python bindings are developer preview.
- constraint: IMX219 has no hardware sync; rolling shutter; pair by nvarguscamerasrc
  `GstBufferMetaData` timestamps (lost after videorate).
- constraint: 4 GB shared RAM; SGM cost memory ~ W*H*D*paths bytes; DNN and SGM share 128 cores.
- constraint: 128 disparities needed for ~0.2 m near range at 640 px with a ~60 mm baseline.
- risk: all Nano fps numbers for SGM/VPI at 640x480 and for MiDaS/Depth Anything are estimates;
  benchmark (`nvpmodel -m 0`, `jetson_clocks`, `trtexec --fp16`) before committing.
- risk: unsynced sensors produce depth errors on fast motion / rotation.
- risk: TRT 8.2.1-only models (SwitchStereo) require upgrading to JetPack 4.6.1+.
- risk: CMake >= 3.18 needed for libSGM on Ubuntu 18.04 (install via pip).
- risk: OpenCV CUDA build needs >= 6.5-8.5 GB RAM+swap and hours; SD-card swap is slow and wears.
- risk: monocular fill is unreliable at edges and for thin obstacles - never feed it to collision
  avoidance.
