# jetson/models - ONNX export of the depthd monocular models

Mac-side Python tool, not a ROS node. It exports the monocular depth models used by `depthd` (design D5) to ONNX
that TensorRT 8.0.1 on the Jetson Nano can parse, and checks them before they reach the board. It runs on the Mac
(Apple Silicon, CPU); the TensorRT FP16 engines are built on the Jetson itself by `trtexec` (T18/T19), because
engines are tied to the GPU and TensorRT version they were built with.

| Model | Role | Source | Input size | Output file |
|---|---|---|---|---|
| Depth Anything V2 Small | primary | Hugging Face `depth-anything/Depth-Anything-V2-Small-hf` (transformers) | 266x266 (19 x 14, the ViT patch) | `build/dav2_small_266.onnx` |
| MiDaS v2.1 small | fallback | torch hub `intel-isl/MiDaS:v2_1`, `MiDaS_small` + `model-small-70d6b9c8.pt` | 256x256 | `build/midas_small_256.onnx` |

266x266 exports cleanly; no other size had to be measured.

## Usage

```bash
cd jetson/models
uv sync
uv run poe export        # python export_onnx.py [export]: download weights, export, simplify, check
uv run poe check         # python export_onnx.py check: re-check build/*.onnx (or pass paths)
uv run pytest -q         # checker unit tests + both real exports (downloads weights, always runs)
uv run poe lint
```

Everything is driven by `config.yaml`: the opset (13), the output directory (`build/`, relative to the config,
gitignored) and per model its id, source, input height and width, ViT patch size and the normalisation mean/std.
Weights are cached by Hugging Face (`~/.cache/huggingface`) and torch hub (`~/.cache/torch/hub`). `MiDaS_small`
builds its EfficientNet-Lite3 backbone with a nested `torch.hub.load` of `rwightman/gen-efficientnet-pytorch`, so
that repo is listed in `trusted_hub_repos` and trusted before loading (otherwise torch hub prompts on stdin).

## Graph contract (consumed by T18)

- Input `image`: `1x3xHxW` float32, RGB scaled to [0, 1] then normalised `(x - mean) / std` with the ImageNet
  mean `0.485,0.456,0.406` and std `0.229,0.224,0.225` (both models). The values are also stored in the ONNX
  `metadata_props` (`mean`, `std`, `model_id`).
- Output `inverse_depth`: `1x1xHxW` float32, relative inverse depth (larger is nearer, arbitrary scale and shift;
  depthd aligns it to stereo).
- All dimensions static, batch 1.

## TensorRT 8.0 ops constraint

TensorRT 8.0.1 parses ONNX up to opset 13 and does not support Einsum, GatherND, ScatterND or GridSample; bicubic
Resize must not survive into the graph. The export therefore:

- uses the TorchScript exporter (`dynamo=False`) at opset 13 (the dynamo exporter starts at opset 18),
- loads Depth Anything with `attn_implementation="eager"` (no opset-13 symbolic for SDPA),
- runs onnxsim, which constant-folds the bicubic interpolation of the DINOv2 position embeddings to the fixed size.

`export_onnx.find_violations` / `check_onnx` enforce it: they fail on a default-domain opset above 13, any Einsum,
GatherND, ScatterND or GridSample node (subgraphs included), a Resize with `mode=cubic`, or a graph input with a
dynamic or unknown dimension. Every export is checked; `export_onnx.py check` re-checks any file.

The real-export test also runs each ONNX in onnxruntime and requires it to match the torch model to 1e-3 of the
output range (measured: about 2e-6 for Depth Anything, 1e-6 for MiDaS).

## Tests

`tests/test_export.py`:

- checker accepts a tiny model exported with `torch.onnx.export` at opset 13 and a linear Resize;
- checker rejects synthetic models with each of Einsum, GatherND, ScatterND, GridSample, cubic Resize, an Einsum
  inside an If subgraph, opset 14, a dynamic batch axis from export and an unknown input dimension;
- config loads both models at opset 13 and rejects opset 14;
- both real models are exported into `build/`, pass the checker, have the `image`/`inverse_depth` contract and
  shapes, carry the normalisation metadata, and match torch in onnxruntime.
