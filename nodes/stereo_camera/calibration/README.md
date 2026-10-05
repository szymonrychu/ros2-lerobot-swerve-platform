# Stereo calibration files

`left.yaml` and `right.yaml` (camera_info YAML as written by `camera_calibration`, 320x240, plumb_bob) belong in this
directory after the calibration described in `../README.md`. camera_ros loads them through
`camera_info_url: file://<deployed repo>/nodes/stereo_camera/calibration/{left,right}.yaml`.

No calibration is committed on purpose: an invented or default calibration would feed wrong rectification and depth
into the stack. While either file is missing, or its `projection_matrix` is all zero, the launch publishes only the raw
images (`/stereo/{left,right}/image_raw`) and logs a warning.

How to get the files: record a checkerboard bag, run `cameracalibrator` (target RMS below 0.3 px), take `left.yaml` and
`right.yaml` from `/tmp/calibrationdata.tar.gz`, and commit them here. The stereo calibration fills the rectification
matrix `R` and the projection matrices `P` (the right one carries `-fx * baseline` in `P[3]`).
