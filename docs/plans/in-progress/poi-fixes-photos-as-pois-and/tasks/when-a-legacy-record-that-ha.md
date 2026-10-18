---
title: "When a legacy record that has no stored view goes through photo_as_poi and back through poi_as_record, the result carries a filled-in view. The record that comes back differs from the original in its view field, though the photo POI form is the same."
status: todo
repo: ros2-lerobot-sverve-platform
value: "A map save then load can rewrite index lines with the filled-in view. It does no harm, but the round trip is not byte-identical at the record level."
files: []
verify: ""
depends_on: []
owns: []
uses: []
model: ""
estimate: ""
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# When a legacy record that has no stored view goes through photo_as_poi and back through poi_as_record, the result carries a filled-in view. The record that comes back differs from the original in its view field, though the photo POI form is the same.

## Requirement

When a legacy record that has no stored view goes through photo_as_poi and back through poi_as_record, the result carries a filled-in view. The record that comes back differs from the original in its view field, though the photo POI form is the same.

Where: shared/ros2_common/photos.py:poi_as_record (owned by photo-store-sync-ops)

Why: A map save then load can rewrite index lines with the filled-in view. It does no harm, but the round trip is not byte-identical at the record level.

## Acceptance conditions

- AC1: When a legacy record that has no stored view goes through photo_as_poi and back through poi_as_record, the result carries a filled-in view. The record that comes back differs from the original in its view field, though the photo POI form is the same. at shared/ros2_common/photos.py:poi_as_record (owned by photo-store-sync-ops) is resolved [evidence: judgement]

## Files



## Interfaces

Keep the interfaces around shared/ros2_common/photos.py:poi_as_record (owned by photo-store-sync-ops) unchanged unless resolving it requires otherwise: When a legacy record that has no stored view goes through photo_as_poi and back through poi_as_record, the result carries a filled-in view. The record that comes back differs from the original in its view field, though the photo POI form is the same..

## Expected outcome

When a legacy record that has no stored view goes through photo_as_poi and back through poi_as_record, the result carries a filled-in view. The record that comes back differs from the original in its view field, though the photo POI form is the same. at shared/ros2_common/photos.py:poi_as_record (owned by photo-store-sync-ops) is resolved, and A map save then load can rewrite index lines with the filled-in view. It does no harm, but the round trip is not byte-identical at the record level. no longer applies.

## Steps

1. Write a failing test that shows: When a legacy record that has no stored view goes through photo_as_poi and back through poi_as_record, the result carries a filled-in view. The record that comes back differs from the original in its view field, though the photo POI form is the same..
2. Fix it at shared/ros2_common/photos.py:poi_as_record (owned by photo-store-sync-ops).
3. Run the task verify command and confirm it passes.

## Notes

Discovered in task `poi-replace-export-photos` during run `poi-fixes-photos-as-pois-and`.

