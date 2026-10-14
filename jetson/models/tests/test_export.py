"""Tests for export_onnx: the TensorRT 8.0 compatibility checker and the real model exports."""

from pathlib import Path

import numpy as np
import onnx
import onnxruntime as ort
import pytest
import torch
from onnx import TensorProto, helper

import export_onnx
from export_onnx import OnnxCompatError, check_onnx, find_violations, load_config

MODELS_DIR = Path(__file__).resolve().parent.parent


def _graph_model(nodes, inputs, outputs, opset=13, initializers=()):
    graph = helper.make_graph(nodes, "synthetic", inputs, outputs, initializer=list(initializers))
    return helper.make_model(graph, opset_imports=[helper.make_opsetid("", opset)])


def _tensor(name, shape):
    return helper.make_tensor_value_info(name, TensorProto.FLOAT, shape)


class _Tiny(torch.nn.Module):
    def __init__(self):
        super().__init__()
        self.conv = torch.nn.Conv2d(3, 1, 3, padding=1)

    def forward(self, image):
        x = self.conv(image)
        return torch.nn.functional.interpolate(x, scale_factor=2.0, mode="bilinear", align_corners=True)


def _export_tiny(path: Path, dynamic: bool = False) -> Path:
    torch.onnx.export(
        _Tiny().eval(),
        (torch.zeros(1, 3, 14, 14),),
        str(path),
        opset_version=13,
        dynamo=False,
        input_names=["image"],
        output_names=["inverse_depth"],
        dynamic_axes={"image": {0: "batch"}} if dynamic else None,
    )
    return path


def test_checker_accepts_tiny_exported_model(tmp_path):
    path = _export_tiny(tmp_path / "tiny.onnx")
    check_onnx(path)
    assert find_violations(onnx.load(str(path))) == []


def test_checker_rejects_dynamic_input_dim_from_export(tmp_path):
    path = _export_tiny(tmp_path / "tiny_dynamic.onnx", dynamic=True)
    with pytest.raises(OnnxCompatError, match="dynamic"):
        check_onnx(path)


def test_checker_rejects_unknown_input_dim():
    node = helper.make_node("Relu", ["x"], ["y"])
    model = _graph_model([node], [_tensor("x", [1, 3, None, 8])], [_tensor("y", [1, 3, None, 8])])
    violations = find_violations(model)
    assert len(violations) == 1
    assert "dynamic" in violations[0]


def test_checker_rejects_opset_above_13():
    node = helper.make_node("Relu", ["x"], ["y"])
    model = _graph_model([node], [_tensor("x", [1, 3, 8, 8])], [_tensor("y", [1, 3, 8, 8])], opset=14)
    violations = find_violations(model)
    assert len(violations) == 1
    assert "opset 14" in violations[0]


def _einsum():
    node = helper.make_node("Einsum", ["a", "b"], ["y"], equation="ij,jk->ik")
    return [node], [_tensor("a", [2, 3]), _tensor("b", [3, 4])], [_tensor("y", [2, 4])]


def _gather_nd():
    node = helper.make_node("GatherND", ["x", "i"], ["y"])
    idx = helper.make_tensor_value_info("i", TensorProto.INT64, [2, 1])
    return [node], [_tensor("x", [4, 3]), idx], [_tensor("y", [2, 3])]


def _scatter_nd():
    node = helper.make_node("ScatterND", ["x", "i", "u"], ["y"])
    idx = helper.make_tensor_value_info("i", TensorProto.INT64, [2, 1])
    return [node], [_tensor("x", [4, 3]), idx, _tensor("u", [2, 3])], [_tensor("y", [4, 3])]


def _grid_sample():
    node = helper.make_node("GridSample", ["x", "g"], ["y"])
    return [node], [_tensor("x", [1, 1, 4, 4]), _tensor("g", [1, 4, 4, 2])], [_tensor("y", [1, 1, 4, 4])]


def _resize(mode):
    def build():
        scales = helper.make_tensor("scales", TensorProto.FLOAT, [4], [1.0, 1.0, 2.0, 2.0])
        roi = helper.make_tensor("roi", TensorProto.FLOAT, [0], [])
        node = helper.make_node("Resize", ["x", "roi", "scales"], ["y"], mode=mode)
        return [node], [_tensor("x", [1, 1, 4, 4])], [_tensor("y", [1, 1, 8, 8])], [roi, scales]

    return build


@pytest.mark.parametrize(
    ("builder", "needle"),
    [
        (_einsum, "Einsum"),
        (_gather_nd, "GatherND"),
        (_scatter_nd, "ScatterND"),
        (_grid_sample, "GridSample"),
        (_resize("cubic"), "cubic"),
    ],
)
def test_checker_rejects_each_forbidden_op(tmp_path, builder, needle):
    built = builder()
    model = _graph_model(*built[:3], initializers=built[3] if len(built) > 3 else ())
    violations = find_violations(model)
    assert len(violations) == 1
    assert needle in violations[0]
    path = tmp_path / "forbidden.onnx"
    onnx.save(model, str(path))
    with pytest.raises(OnnxCompatError, match=needle):
        check_onnx(path)


def test_checker_accepts_linear_resize():
    model = _graph_model(*_resize("linear")()[:3], initializers=_resize("linear")()[3])
    assert find_violations(model) == []


def test_checker_finds_forbidden_op_inside_subgraph():
    inner_nodes, _, inner_out = _einsum()
    then_graph = helper.make_graph(inner_nodes, "then", [], inner_out)
    else_graph = helper.make_graph([helper.make_node("Identity", ["a"], ["y"])], "else", [], [_tensor("y", [2, 3])])
    cond = helper.make_tensor_value_info("c", TensorProto.BOOL, [])
    node = helper.make_node("If", ["c"], ["y"], then_branch=then_graph, else_branch=else_graph)
    model = _graph_model([node], [cond, _tensor("a", [2, 3]), _tensor("b", [3, 4])], [_tensor("y", [2, 4])])
    violations = find_violations(model)
    assert any("Einsum" in v for v in violations)


def test_config_lists_both_models_at_opset_13():
    config = load_config(MODELS_DIR / "config.yaml")
    assert config.opset == 13
    by_name = {m.name: m for m in config.models}
    assert by_name["dav2_small_266"].source == "transformers"
    assert by_name["dav2_small_266"].model_id == "depth-anything/Depth-Anything-V2-Small-hf"
    assert (by_name["dav2_small_266"].height, by_name["dav2_small_266"].width) == (266, 266)
    assert by_name["dav2_small_266"].height % 14 == 0
    assert by_name["midas_small_256"].source == "torch_hub"
    assert (by_name["midas_small_256"].height, by_name["midas_small_256"].width) == (256, 256)


def test_config_rejects_opset_above_13(tmp_path):
    bad = tmp_path / "config.yaml"
    bad.write_text("opset: 14\noutput_dir: build\nmodels: []\n")
    with pytest.raises(ValueError, match="opset"):
        load_config(bad)


def test_cli_without_command_exports(monkeypatch):
    # T18's verify runs `python export_onnx.py` with no arguments: that must export.
    calls = []
    monkeypatch.setattr(export_onnx, "export_all", lambda config: calls.append(config) or [])
    assert export_onnx.main([]) == 0
    assert len(calls) == 1


def test_cli_check_on_explicit_path(tmp_path):
    path = _export_tiny(tmp_path / "tiny.onnx")
    assert export_onnx.main(["check", str(path)]) == 0


def _dims(value_info):
    return [d.dim_value for d in value_info.type.tensor_type.shape.dim]


def test_exports_both_real_models_and_they_pass_the_checker():
    """Downloads the real weights, exports both models into build/ and checks them. Always runs."""
    config = load_config(MODELS_DIR / "config.yaml")
    paths = export_onnx.export_all(config)
    assert {p.name for p in paths} == {"dav2_small_266.onnx", "midas_small_256.onnx"}
    for spec, path in zip(config.models, paths, strict=True):
        assert path.parent == config.output_dir
        assert path.stat().st_size > 1_000_000
        check_onnx(path)
        model = onnx.load(str(path))
        assert [i.name for i in model.graph.input] == ["image"]
        assert [o.name for o in model.graph.output] == ["inverse_depth"]
        assert _dims(model.graph.input[0]) == [1, 3, spec.height, spec.width]
        assert _dims(model.graph.output[0]) == [1, 1, spec.height, spec.width]
        meta = {p.key: p.value for p in model.metadata_props}
        assert meta["mean"] == "0.485,0.456,0.406"
        assert meta["std"] == "0.229,0.224,0.225"
        assert_matches_torch(spec, path)


def assert_matches_torch(spec, path):
    """The exported graph computes what the torch model computes on the same normalised image."""
    image = torch.rand(1, 3, spec.height, spec.width, generator=torch.Generator().manual_seed(0))
    mean = torch.tensor(spec.mean).view(1, 3, 1, 1)
    std = torch.tensor(spec.std).view(1, 3, 1, 1)
    image = (image - mean) / std
    with torch.no_grad():
        expected = export_onnx.load_torch_model(spec)(image).numpy()
    session = ort.InferenceSession(str(path), providers=["CPUExecutionProvider"])
    (actual,) = session.run(["inverse_depth"], {"image": image.numpy()})
    assert actual.shape == expected.shape
    scale = float(np.abs(expected).max())
    assert scale > 0
    assert float(np.abs(actual - expected).max()) <= 1e-3 * scale
