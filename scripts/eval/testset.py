"""Fixed evaluation test set: 10 objects x 3 placements, case expansion per mode and prompt rendering.

A test set file is versioned and never edited in place; a change gets a new version file (testset_v2.yaml).
"""

import string
from pathlib import Path
from typing import Literal

import yaml
from pydantic import BaseModel, Field, model_validator

TESTSET_V1 = Path(__file__).resolve().parent / "testset_v1.yaml"
TESTSET_WORKFLOWS_V1 = Path(__file__).resolve().parent / "testset_workflows_v1.yaml"
KIND_WORKFLOWS = "workflows"
KIND_PICK_AND_PLACE = "pick_and_place"
OBJECT_COUNT = 10
PLACEMENT_COUNT = 3
TOTAL_CASES = OBJECT_COUNT * PLACEMENT_COUNT
MODE_ARM_ONLY = "arm_only"
MODE_FULL = "full"
MODES = (MODE_ARM_ONLY, MODE_FULL)
PROMPT_FIELDS = {"object_name", "placement_description", "place_description"}

Mode = Literal["arm_only", "full"]


class PlaceTarget(BaseModel):
    """Where every object is put (the container)."""

    model_config = {"extra": "forbid"}

    id: str
    description: str


class TestObject(BaseModel):
    """One object of the test set."""

    __test__ = False  # not a pytest test class
    model_config = {"extra": "forbid"}

    id: str
    name: str
    description: str
    soft: bool = False


class Placement(BaseModel):
    """Where an object lies at the start of a trial."""

    model_config = {"extra": "forbid"}

    id: str
    description: str
    arm_only: bool


class Case(BaseModel):
    """One (object, placement) pair; frozen so it can key dicts."""

    model_config = {"extra": "forbid", "frozen": True}

    object_id: str
    placement_id: str

    @property
    def key(self) -> str:
        """Case key used in file names and run.yaml subsets.

        Returns:
            str: ``<object_id>/<placement_id>``.
        """
        return f"{self.object_id}/{self.placement_id}"


class TestSet(BaseModel):
    """Validated test set: exactly 10 objects and 3 placements with unique ids, at least one arm_only placement."""

    __test__ = False  # not a pytest test class
    model_config = {"extra": "forbid"}

    version: int
    place_target: PlaceTarget
    objects: list[TestObject]
    placements: list[Placement]
    prompt_template: str
    arm_only_suffix: str

    @model_validator(mode="after")
    def check_shape(self) -> "TestSet":
        """Enforce the 10x3 shape, unique ids, an arm_only placement and the template placeholders.

        Returns:
            TestSet: The validated test set.
        """
        if len(self.objects) != OBJECT_COUNT:
            raise ValueError(f"exactly {OBJECT_COUNT} objects required, got {len(self.objects)}")
        if len(self.placements) != PLACEMENT_COUNT:
            raise ValueError(f"exactly {PLACEMENT_COUNT} placements required, got {len(self.placements)}")
        for kind, ids in (("object", [o.id for o in self.objects]), ("placement", [p.id for p in self.placements])):
            duplicates = sorted({i for i in ids if ids.count(i) > 1})
            if duplicates:
                raise ValueError(f"duplicate {kind} id(s): {', '.join(duplicates)}")
        if not any(p.arm_only for p in self.placements):
            raise ValueError("at least one arm_only placement required")
        fields = {name for _, name, _, _ in string.Formatter().parse(self.prompt_template) if name is not None}
        if fields - PROMPT_FIELDS:
            raise ValueError(
                f"prompt_template uses unknown placeholder(s) {sorted(fields - PROMPT_FIELDS)}; "
                f"allowed: {sorted(PROMPT_FIELDS)}"
            )
        return self

    def cases(self, mode: str) -> list[Case]:
        """Expand the test set into cases, object-major; arm_only keeps only arm_only placements.

        Args:
            mode (str): ``arm_only`` or ``full``.

        Returns:
            list[Case]: Cases in a deterministic order.

        Raises:
            ValueError: For an unknown mode.
        """
        if mode not in MODES:
            raise ValueError(f"unknown mode {mode!r}; expected one of {MODES}")
        placements = [p for p in self.placements if mode == MODE_FULL or p.arm_only]
        return [Case(object_id=o.id, placement_id=p.id) for o in self.objects for p in placements]

    def render_prompt(self, case: Case, mode: str) -> str:
        """Render the instruction text of a case.

        Args:
            case (Case): The case.
            mode (str): ``arm_only`` appends arm_only_suffix.

        Returns:
            str: The instruction sent to the agent.

        Raises:
            KeyError: When the case names an unknown object or placement.
        """
        obj = {o.id: o for o in self.objects}[case.object_id]
        placement = {p.id: p for p in self.placements}[case.placement_id]
        text = self.prompt_template.format(
            object_name=obj.name,
            placement_description=placement.description,
            place_description=self.place_target.description,
        )
        return text + self.arm_only_suffix if mode == MODE_ARM_ONLY else text


class WorkflowCase(BaseModel):
    """One free-text job of a workflow test set."""

    model_config = {"extra": "forbid", "frozen": True}

    id: str
    setup: str = Field(min_length=1, description="What the operator prepares before the trial")
    prompt: str = Field(min_length=1, description="Free text sent to the agent")
    expected_workflow: str = Field(min_length=1, description="start_workflow template the agent should pick")

    @property
    def key(self) -> str:
        """Case key used in file names and run.yaml subsets.

        Returns:
            str: The case id.
        """
        return self.id


class WorkflowTestSet(BaseModel):
    """Validated workflow test set: unique case ids."""

    model_config = {"extra": "forbid"}

    kind: Literal["workflows"]
    version: int
    cases: list[WorkflowCase] = Field(min_length=1)

    @model_validator(mode="after")
    def check_unique_ids(self) -> "WorkflowTestSet":
        """Reject duplicate case ids.

        Returns:
            WorkflowTestSet: The validated test set.
        """
        ids = [c.id for c in self.cases]
        duplicates = sorted({i for i in ids if ids.count(i) > 1})
        if duplicates:
            raise ValueError(f"duplicate case id(s): {', '.join(duplicates)}")
        return self


def load_testset(path: str | Path) -> TestSet:
    """Load and validate a test set YAML file.

    Args:
        path (str | Path): YAML file.

    Returns:
        TestSet: The validated test set.
    """
    return TestSet.model_validate(yaml.safe_load(Path(path).read_text()))


def load_any_testset(path: str | Path) -> TestSet | WorkflowTestSet:
    """Load a test set, dispatching on its ``kind`` (missing kind is the v1 pick-and-place set).

    Args:
        path (str | Path): YAML file.

    Returns:
        TestSet | WorkflowTestSet: The validated test set.

    Raises:
        ValueError: For an unknown kind.
    """
    data = yaml.safe_load(Path(path).read_text())
    kind = data.get("kind", KIND_PICK_AND_PLACE) if isinstance(data, dict) else KIND_PICK_AND_PLACE
    if kind == KIND_WORKFLOWS:
        return WorkflowTestSet.model_validate(data)
    if kind == KIND_PICK_AND_PLACE:
        return TestSet.model_validate({k: v for k, v in data.items() if k != "kind"})
    raise ValueError(f"unknown test set kind {kind!r}; expected {KIND_WORKFLOWS!r} or {KIND_PICK_AND_PLACE!r}")
