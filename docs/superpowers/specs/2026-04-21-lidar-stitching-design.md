# LiDAR Multi-Scan Stitching — Design Spec
**Date:** 2026-04-21
**Branch:** dannydev

---

## Problem

The current `stitch_two_ply_files()` function uses ICP (Iterative Closest Point) with an identity matrix as the initial transformation guess. ICP is a local optimizer — it only converges correctly when two scans are already within a few percent of their true alignment. For scans taken from different positions in the same room (handheld pole, no fixed reference), the initial guess is far from correct, so ICP diverges or locks onto the wrong result.

---

## Goals

- Fully automatic stitching: no manual alignment, no user input beyond selecting files
- Support N scans of the same room from any scanner position/orientation
- Fill in occluded spots across scans (complete coverage)
- Preserve and increase point density in overlapping areas, then downsample uniformly
- Remove outlier/noise points before and after merging
- Zero UI changes required — existing "Stitch" button workflow stays intact

---

## Non-Goals

- Manual pre-alignment UI controls
- Real-time / streaming stitching
- Outdoor or large-scale (multi-room) registration

---

## Context

- **Scanner:** Livox360 (360° horizontal, full-room capture from a single position)
- **Operator:** SWAT personnel — zero expectation of technical knowledge
- **Scan overlap:** High — each scan captures most of the room; scans share substantial geometry
- **Existing floor detection:** `stabilize_scan_to_floor_frame_numpy()` already normalizes scans to floor-up orientation; this reduces the registration problem from 6DOF to 3DOF (X, Y, yaw only)

---

## Pipeline

Each pair of scans goes through this sequence. For N scans, registration is sequential: scan 2 aligns to scan 1, scan 3 aligns to the merged 1+2 cloud, and so on.

```
For each scan being added to the running merged cloud:

1. Floor-normalize both scans
   └─ stabilize_scan_to_floor_frame_numpy() (existing)
   └─ aligns gravity axis → reduces 6DOF to 3DOF

2. Statistical outlier removal (pre-registration)
   └─ o3d.geometry.PointCloud.remove_statistical_outlier()
   └─ removes floating noise and scanner artifacts before feature extraction
   └─ prevents RANSAC from wasting iterations matching noise points

3. Voxel downsample to coarse grid (default: 5 cm)
   └─ operates on a copy — full-res cloud preserved for final merge
   └─ makes FPFH extraction tractable on dense Livox360 clouds

4. Estimate surface normals on coarse cloud
   └─ required for FPFH and Point-to-Plane ICP

5. Extract FPFH features (Fast Point Feature Histograms)
   └─ o3d.pipelines.registration.compute_fpfh_feature()
   └─ 33-dimensional geometry fingerprint per point

6. RANSAC global registration
   └─ o3d.pipelines.registration.registration_ransac_based_on_feature_matching()
   └─ matches FPFH fingerprints → finds rough transform with no initial guess
   └─ works from any scanner position or orientation

7. Point-to-Plane ICP refinement
   └─ o3d.pipelines.registration.registration_icp() with TransformationEstimationPointToPlane
   └─ takes RANSAC result as initial guess
   └─ refines alignment to ~1 cm precision on full-res cloud

8. Apply final transform, merge with running cloud

9. Statistical outlier removal (post-merge)
   └─ cleans seam artifacts introduced at merge boundaries

10. Voxel downsample merged result
    └─ uniform density across full room
    └─ removes duplicate points in high-overlap regions
```

---

## Code Changes

All changes are inside `interface.py`. No other files are modified.

### Replace `stitch_two_ply_files()`

Current signature (kept for compatibility):
```python
def stitch_two_ply_files(source_path, target_path, output_path, distance_threshold=0.2) -> str
```

New implementation replaces the body with the full pipeline above. Key parameters:
- `voxel_size: float = 0.05` — coarse grid for FPFH (meters); controls speed vs. accuracy trade-off
- `distance_threshold: float = 0.02` — ICP convergence threshold (meters); tighter than current 0.2
- `nb_neighbors: int = 30`, `std_ratio: float = 2.0` — outlier removal parameters (consistent with existing processing pipeline)

### `stitch_multiple_ply_files()` — no signature change

Already chains `stitch_two_ply_files()` correctly. No changes needed.

### `StitchWorker`, `_stitch_clicked`, UI — no changes

The worker thread and all UI wiring remain exactly as-is.

---

## Parameters

| Parameter | Default | Meaning |
|-----------|---------|---------|
| `voxel_size` | 0.05 m | Coarse grid for FPFH feature extraction |
| `distance_threshold` | 0.02 m | ICP convergence distance |
| `nb_neighbors` | 30 | Outlier removal: neighbor count |
| `std_ratio` | 2.0 | Outlier removal: std deviation multiplier |
| `ransac_max_iter` | 100,000 | RANSAC iteration budget |

---

## Success Criteria

1. Two scans of the same room taken from different positions align correctly (walls, floor, ceiling match)
2. Occluded spots in one scan are filled in from the other
3. No floating noise points in the merged output
4. Existing "Stitch" button triggers the new pipeline with no UI changes
5. Works correctly for 3+ scans chained together
