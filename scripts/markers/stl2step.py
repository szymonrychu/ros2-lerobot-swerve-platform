"""Convert a marker STL mesh to a STEP solid for CAD editing (e.g. Fusion 360).

The mesh is sewn into a shell, closed into a solid, and coplanar triangles are merged into planar faces
(ShapeUpgrade_UnifySameDomain); curved surfaces stay faceted. Needs OpenCascade (cadquery-ocp), which loads libGL, so
run it on a desktop:

    uv run --no-project --with cadquery-ocp python scripts/markers/stl2step.py markers/charuco-arm-6x5-28mm-base.stl \
        charuco-arm-6x5-28mm-base.step
"""

import argparse
import logging
import sys

from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeSolid, BRepBuilderAPI_Sewing
from OCP.IFSelect import IFSelect_RetDone
from OCP.ShapeUpgrade import ShapeUpgrade_UnifySameDomain
from OCP.STEPControl import STEPControl_AsIs, STEPControl_Writer
from OCP.StlAPI import StlAPI_Reader
from OCP.TopAbs import TopAbs_FACE, TopAbs_ShapeEnum, TopAbs_SHELL
from OCP.TopExp import TopExp_Explorer
from OCP.TopoDS import TopoDS, TopoDS_Shape

SEWING_TOLERANCE_MM = 1e-3

LOGGER = logging.getLogger(__name__)


def count_subshapes(shape: TopoDS_Shape, kind: TopAbs_ShapeEnum) -> int:
    """Count the sub-shapes of one kind.

    Args:
        shape (TopoDS_Shape): Shape to explore.
        kind (TopAbs_ShapeEnum): Sub-shape kind (e.g. TopAbs_FACE).

    Returns:
        int: Number of sub-shapes of that kind.
    """
    explorer, count = TopExp_Explorer(shape, kind), 0
    while explorer.More():
        count += 1
        explorer.Next()
    return count


def mesh_to_solid(mesh: TopoDS_Shape) -> TopoDS_Shape:
    """Sew a triangle mesh into a solid and merge coplanar faces.

    Args:
        mesh (TopoDS_Shape): Triangles as read by StlAPI_Reader.

    Returns:
        TopoDS_Shape: Solid with coplanar triangles merged into planar faces.
    """
    sewing = BRepBuilderAPI_Sewing(SEWING_TOLERANCE_MM)
    sewing.Add(mesh)
    sewing.Perform()
    solid_maker = BRepBuilderAPI_MakeSolid()
    explorer = TopExp_Explorer(sewing.SewedShape(), TopAbs_SHELL)
    while explorer.More():
        solid_maker.Add(TopoDS.Shell_s(explorer.Current()))
        explorer.Next()
    unify = ShapeUpgrade_UnifySameDomain(solid_maker.Solid(), True, True, True)
    unify.Build()
    return unify.Shape()


def stl_to_step(src: str, dst: str) -> tuple[int, int]:
    """Convert an STL file to a STEP file.

    Args:
        src (str): Input STL path.
        dst (str): Output STEP path.

    Returns:
        tuple[int, int]: (mesh triangle faces, STEP faces after merging).

    Raises:
        RuntimeError: The STL cannot be read or the STEP cannot be written.
    """
    mesh = TopoDS_Shape()
    if not StlAPI_Reader().Read(mesh, src):
        raise RuntimeError(f"cannot read STL {src}")
    solid = mesh_to_solid(mesh)
    writer = STEPControl_Writer()
    writer.Transfer(solid, STEPControl_AsIs)
    if writer.Write(dst) != IFSelect_RetDone:
        raise RuntimeError(f"cannot write STEP {dst}")
    return count_subshapes(mesh, TopAbs_FACE), count_subshapes(solid, TopAbs_FACE)


def main() -> int:
    """Command line entry point.

    Returns:
        int: Process exit code.
    """
    logging.basicConfig(level=logging.INFO, format="%(message)s")
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("src", help="input STL")
    parser.add_argument("dst", help="output STEP")
    args = parser.parse_args()
    before, after = stl_to_step(args.src, args.dst)
    LOGGER.info("%s: %d mesh faces -> %d STEP faces in %s", args.src, before, after, args.dst)
    return 0


if __name__ == "__main__":
    sys.exit(main())
