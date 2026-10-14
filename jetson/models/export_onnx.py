"""Export the depthd monocular depth models to ONNX that TensorRT 8.0 on the Jetson Nano can build.

Runs on the Mac. Reads config.yaml (model ids, input sizes, opset), downloads the weights, exports each model
with a fixed input shape at opset <= 13, simplifies it with onnxsim (which also constant-folds the bicubic
position-embedding resize of the ViT) and checks the result against the TensorRT 8.0 ONNX parser limits.

Every exported graph has one input ``image`` (1x3xHxW float32, RGB in [0, 1] normalised with the ``mean``/``std``
stored in the model metadata) and one output ``inverse_depth`` (1x1xHxW float32, relative inverse depth).

Usage:
    python export_onnx.py [export] [--config config.yaml]   (export is the default)
    python export_onnx.py check [--config config.yaml] [model.onnx ...]
"""

import argparse
import logging
import sys
from collections.abc import Iterator
from pathlib import Path
from typing import Literal

import onnx
import torch
import yaml
from onnxsim import simplify
from pydantic import BaseModel, Field, model_validator
from transformers import AutoModelForDepthEstimation

MAX_OPSET = 13
FORBIDDEN_OPS = frozenset({"Einsum", "GatherND", "ScatterND", "GridSample"})
DEFAULT_DOMAINS = frozenset({"", "ai.onnx"})
INPUT_NAME = "image"
OUTPUT_NAME = "inverse_depth"
DEFAULT_CONFIG = Path(__file__).resolve().parent / "config.yaml"

LOG = logging.getLogger("export_onnx")


class OnnxCompatError(ValueError):
    """An ONNX model TensorRT 8.0 cannot build. ``violations`` lists every reason."""

    def __init__(self, path: Path, violations: list[str]) -> None:
        self.violations = violations
        super().__init__(f"{path}: not TensorRT 8.0 compatible: " + "; ".join(violations))


class ModelSpec(BaseModel):
    """One model to export.

    ``source`` is ``transformers`` (``model_id`` is a Hugging Face repo) or ``torch_hub`` (``model_id`` is a
    ``owner/repo:ref`` hub repo, ``entrypoint`` its hub function, ``weights_url`` the checkpoint).
    """

    name: str
    source: Literal["transformers", "torch_hub"]
    model_id: str
    height: int = Field(gt=0)
    width: int = Field(gt=0)
    patch_size: int = Field(default=1, gt=0)
    mean: tuple[float, float, float]
    std: tuple[float, float, float]
    entrypoint: str | None = None
    weights_url: str | None = None
    trusted_hub_repos: list[str] = Field(default_factory=list)

    @model_validator(mode="after")
    def check_spec(self) -> "ModelSpec":
        if self.height % self.patch_size or self.width % self.patch_size:
            raise ValueError(f"{self.name}: {self.height}x{self.width} is not a multiple of patch {self.patch_size}")
        if self.source == "torch_hub" and not (self.entrypoint and self.weights_url):
            raise ValueError(f"{self.name}: torch_hub models need entrypoint and weights_url")
        return self


class ExportConfig(BaseModel):
    """The whole config.yaml. ``output_dir`` is absolute once loaded."""

    opset: int = Field(le=MAX_OPSET, gt=0)
    output_dir: Path
    models: list[ModelSpec]


class InverseDepthNet(torch.nn.Module):
    """Adapts a depth model to the export contract: ``image`` 1x3xHxW -> ``inverse_depth`` 1x1xHxW."""

    def __init__(self, net: torch.nn.Module, source: str) -> None:
        super().__init__()
        self.net = net
        self.source = source

    def forward(self, image: torch.Tensor) -> torch.Tensor:
        if self.source == "transformers":
            depth = self.net(pixel_values=image).predicted_depth
        else:
            depth = self.net(image)
        return depth.unsqueeze(1)


def load_config(path: Path) -> ExportConfig:
    """Load and validate config.yaml.

    Args:
        path (Path): the config file.
    Returns:
        ExportConfig: the config, with ``output_dir`` resolved against the config file's directory.
    Raises:
        ValueError: on an invalid config (for example opset above 13).
    """
    path = Path(path)
    config = ExportConfig.model_validate(yaml.safe_load(path.read_text()))
    if not config.output_dir.is_absolute():
        config.output_dir = (path.resolve().parent / config.output_dir).resolve()
    return config


def iter_nodes(graph: onnx.GraphProto) -> Iterator[onnx.NodeProto]:
    """Yield every node of ``graph``, including the nodes of nested subgraphs (If, Loop, Scan bodies).

    Args:
        graph (onnx.GraphProto): the graph to walk.
    Returns:
        Iterator[onnx.NodeProto]: all nodes, depth first.
    """
    for node in graph.node:
        yield node
        for attr in node.attribute:
            if attr.type == onnx.AttributeProto.GRAPH:
                yield from iter_nodes(attr.g)
            elif attr.type == onnx.AttributeProto.GRAPHS:
                for sub in attr.graphs:
                    yield from iter_nodes(sub)


def find_violations(model: onnx.ModelProto, max_opset: int = MAX_OPSET) -> list[str]:
    """List every reason TensorRT 8.0 cannot parse ``model``.

    Rejects an ai.onnx opset above ``max_opset``, any Einsum, GatherND, ScatterND or GridSample node, a Resize in
    cubic mode, and any graph input dimension that is not a fixed positive size.

    Args:
        model (onnx.ModelProto): the model to inspect.
        max_opset (int): the highest default-domain opset allowed.
    Returns:
        list[str]: human-readable violations; empty when the model is compatible.
    """
    violations = []
    for opset in model.opset_import:
        if opset.domain in DEFAULT_DOMAINS and opset.version > max_opset:
            violations.append(f"opset {opset.version} is above {max_opset}")
    for node in iter_nodes(model.graph):
        if node.op_type in FORBIDDEN_OPS:
            violations.append(f"forbidden op {node.op_type} (node {node.name or '<unnamed>'})")
        elif node.op_type == "Resize":
            mode = next((onnx.helper.get_attribute_value(a) for a in node.attribute if a.name == "mode"), b"nearest")
            if mode == b"cubic":
                violations.append(f"Resize in cubic mode (node {node.name or '<unnamed>'})")
    initializers = {init.name for init in model.graph.initializer}
    for graph_input in model.graph.input:
        if graph_input.name in initializers:
            continue
        dims = graph_input.type.tensor_type.shape.dim
        shape = [d.dim_param or (d.dim_value if d.HasField("dim_value") else "?") for d in dims]
        if not dims or any(not d.HasField("dim_value") or d.dim_value <= 0 for d in dims):
            violations.append(f"input {graph_input.name} has a dynamic dimension {shape}")
    return violations


def check_onnx(path: Path, max_opset: int = MAX_OPSET) -> None:
    """Load an ONNX file and fail if TensorRT 8.0 cannot build it.

    Args:
        path (Path): the ONNX file.
        max_opset (int): the highest default-domain opset allowed.
    Raises:
        OnnxCompatError: listing every violation found by ``find_violations``.
    """
    violations = find_violations(onnx.load(str(path)), max_opset)
    if violations:
        raise OnnxCompatError(Path(path), violations)


def load_torch_model(spec: ModelSpec) -> torch.nn.Module:
    """Download the weights and build the eval-mode torch model for ``spec``.

    Args:
        spec (ModelSpec): the model to load.
    Returns:
        torch.nn.Module: the model wrapped in ``InverseDepthNet``, in eval mode on the CPU.
    """
    if spec.source == "transformers":
        # Eager attention: the TorchScript exporter has no opset-13 symbolic for scaled_dot_product_attention.
        net = AutoModelForDepthEstimation.from_pretrained(spec.model_id, attn_implementation="eager")
    else:
        for repo in spec.trusted_hub_repos:
            torch.hub.list(repo, trust_repo=True)
        net = torch.hub.load(spec.model_id, spec.entrypoint, pretrained=False, trust_repo=True)
        state = torch.hub.load_state_dict_from_url(spec.weights_url, map_location="cpu", check_hash=True)
        net.load_state_dict(state)
    return InverseDepthNet(net, spec.source).eval()


def export_model(spec: ModelSpec, opset: int, output_dir: Path) -> Path:
    """Export, simplify, annotate and check one model.

    Args:
        spec (ModelSpec): the model to export.
        opset (int): the ONNX opset to export at (at most 13).
        output_dir (Path): where ``<spec.name>.onnx`` is written.
    Returns:
        Path: the written ONNX file.
    Raises:
        OnnxCompatError: if the simplified model is not TensorRT 8.0 compatible.
        RuntimeError: if onnxsim cannot validate the simplified model.
    """
    output_dir.mkdir(parents=True, exist_ok=True)
    path = output_dir / f"{spec.name}.onnx"
    raw_path = output_dir / f"{spec.name}.raw.onnx"
    model = load_torch_model(spec)
    dummy = torch.rand(1, 3, spec.height, spec.width)
    LOG.info("exporting %s (%s) at %dx%d, opset %d", spec.name, spec.model_id, spec.height, spec.width, opset)
    with torch.no_grad():
        torch.onnx.export(
            model,
            (dummy,),
            str(raw_path),
            opset_version=opset,
            dynamo=False,
            do_constant_folding=True,
            input_names=[INPUT_NAME],
            output_names=[OUTPUT_NAME],
        )
    simplified, ok = simplify(onnx.load(str(raw_path)))
    if not ok:
        raise RuntimeError(f"{spec.name}: onnxsim could not validate the simplified model")
    onnx.helper.set_model_props(
        simplified,
        {
            "model_id": spec.model_id,
            "mean": ",".join(str(v) for v in spec.mean),
            "std": ",".join(str(v) for v in spec.std),
            "output": "relative inverse depth, larger is nearer",
        },
    )
    onnx.save(simplified, str(path))
    raw_path.unlink()
    check_onnx(path, opset)
    LOG.info("wrote %s (%.1f MB)", path, path.stat().st_size / 1e6)
    return path


def export_all(config: ExportConfig) -> list[Path]:
    """Export every model in ``config``, in config order.

    Args:
        config (ExportConfig): the loaded config.
    Returns:
        list[Path]: the written ONNX files.
    """
    return [export_model(spec, config.opset, config.output_dir) for spec in config.models]


def main(argv: list[str] | None = None) -> int:
    """Command line entry point.

    Args:
        argv (list[str] | None): arguments, defaults to ``sys.argv[1:]``.
    Returns:
        int: process exit code, 0 when every model is exported or checked clean.
    """
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("command", nargs="?", default="export", choices=["export", "check"], help="default: export")
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("paths", nargs="*", type=Path, help="ONNX files to check (default: the configured outputs)")
    args = parser.parse_args(argv)
    logging.basicConfig(level=logging.INFO, format="%(levelname)s %(name)s: %(message)s")
    config = load_config(args.config)
    if args.command == "export":
        export_all(config)
        return 0
    paths = args.paths or [config.output_dir / f"{spec.name}.onnx" for spec in config.models]
    failed = 0
    for path in paths:
        try:
            check_onnx(path, config.opset)
            LOG.info("%s: ok", path)
        except OnnxCompatError as err:
            LOG.error("%s", err)
            failed += 1
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
