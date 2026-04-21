# LiDAR Multi-Scan Stitching Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the identity-guess ICP stitcher in `interface.py` with a fully automatic FPFH + RANSAC + Point-to-Plane ICP pipeline that correctly aligns same-room scans taken from any position or orientation.

**Architecture:** Each pair of scans is floor-normalized (gravity aligned), cleaned of outliers, then globally registered via FPFH feature matching + RANSAC, and finally refined with Point-to-Plane ICP. For N scans, the pairwise function is chained sequentially (same as today). No UI changes.

**Tech Stack:** Python 3, Open3D (`o3d.pipelines.registration`), NumPy, PyQt6 (unchanged)

---

## File Map

| File | Change |
|------|--------|
| `interface.py:1184-1213` | Replace body of `stitch_two_ply_files()` with FPFH+RANSAC+ICP pipeline |
| `interface.py:1216` | Update `distance_threshold` default from `0.2` → `0.02` in `stitch_multiple_ply_files()` |

No new files. No other files touched.

---

## Task 1: Replace `stitch_two_ply_files()` with the FPFH+RANSAC+ICP pipeline

**Files:**
- Modify: `interface.py:1184-1213`

This is the only substantive code change. The function signature stays identical so `stitch_multiple_ply_files()`, `StitchWorker`, and all UI wiring continue to work without modification.

**Pipeline recap:**
1. Load both PLY files with Open3D
2. Floor-normalize both (numpy → `stabilize_scan_to_floor_frame_numpy()` → back to Open3D cloud)
3. Statistical outlier removal (pre-registration)
4. Voxel downsample to 5 cm grid (copy — full-res preserved)
5. Estimate surface normals on downsampled clouds
6. Extract FPFH features
7. RANSAC global registration → rough transform
8. Point-to-Plane ICP on full-res clean clouds → refined transform
9. Apply transform, merge
10. Statistical outlier removal (post-merge)
11. Voxel downsample merged result
12. Save

- [ ] **Step 1: Replace `stitch_two_ply_files()` in `interface.py`**

Find and replace the entire function at `interface.py:1184-1213`. The new implementation:

```python
def stitch_two_ply_files(source_path: str, target_path: str, output_path: str, distance_threshold: float = 0.02) -> str:
    #fpfh+ransac global registration followed by point-to-plane icp for same-room scans from any position
    o3d = _import_open3d()

    source_raw = o3d.io.read_point_cloud(source_path)
    target_raw = o3d.io.read_point_cloud(target_path)

    if source_raw.is_empty():
        raise ValueError(f"source cloud is empty: {os.path.basename(source_path)}")
    if target_raw.is_empty():
        raise ValueError(f"target cloud is empty: {os.path.basename(target_path)}")

    voxel_size = 0.05
    nb_neighbors = 30
    std_ratio = 2.0

    def _floor_normalize_o3d(cloud: "o3d.geometry.PointCloud") -> "o3d.geometry.PointCloud":
        xyz = np.asarray(cloud.points, dtype=np.float32)
        result = stabilize_scan_to_floor_frame_numpy(xyz)
        normalized = o3d.geometry.PointCloud()
        normalized.points = o3d.utility.Vector3dVector(result["xyz"])
        if cloud.has_colors():
            normalized.colors = cloud.colors
        return normalized

    def _clean(cloud: "o3d.geometry.PointCloud") -> "o3d.geometry.PointCloud":
        cleaned, _ = cloud.remove_statistical_outlier(nb_neighbors=nb_neighbors, std_ratio=std_ratio)
        return cleaned

    def _fpfh(cloud: "o3d.geometry.PointCloud") -> tuple:
        down = cloud.voxel_down_sample(voxel_size)
        down.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 2, max_nn=30))
        features = o3d.pipelines.registration.compute_fpfh_feature(
            down,
            o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 5, max_nn=100),
        )
        return down, features

    source = _clean(_floor_normalize_o3d(source_raw))
    target = _clean(_floor_normalize_o3d(target_raw))

    source_down, source_fpfh = _fpfh(source)
    target_down, target_fpfh = _fpfh(target)

    ransac_result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
        source_down,
        target_down,
        source_fpfh,
        target_fpfh,
        mutual_filter=True,
        max_correspondence_distance=voxel_size * 1.5,
        estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
        ransac_n=3,
        checkers=[
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(voxel_size * 1.5),
        ],
        criteria=o3d.pipelines.registration.RANSACConvergenceCriteria(100000, 0.999),
    )

    source.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 2, max_nn=30))
    target.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 2, max_nn=30))

    icp_result = o3d.pipelines.registration.registration_icp(
        source,
        target,
        distance_threshold,
        ransac_result.transformation,
        o3d.pipelines.registration.TransformationEstimationPointToPlane(),
    )

    source_aligned = copy.deepcopy(source)
    source_aligned.transform(icp_result.transformation)

    merged = source_aligned + target
    merged = _clean(merged)
    merged = merged.voxel_down_sample(voxel_size)

    if not o3d.io.write_point_cloud(output_path, merged):
        raise IOError(f"failed to save stitched model to {output_path}")
    return output_path
```

- [ ] **Step 2: Update `distance_threshold` default in `stitch_multiple_ply_files()`**

At `interface.py:1216`, change the default from `0.2` to `0.02` so it matches the new ICP threshold:

```python
def stitch_multiple_ply_files(input_paths: List[str], output_path: str, distance_threshold: float = 0.02) -> str:
```

- [ ] **Step 3: Verify the app still launches**

Run:
```bash
python interface.py
```

Expected: window opens, no import errors, no tracebacks. The "Stitch" button should be visible and enabled.

- [ ] **Step 4: Manual stitch test with two PLY files**

1. In the app, import two PLY files of the same room via the Import button
2. Select both files in the queue (Ctrl+click)
3. Click "Stitch"
4. Watch the log area — expected sequence:
   ```
   [stitch] stitching 2 selected file(s)...
   [stitch] 1. <filename1>.ply
   [stitch] 2. <filename2>.ply
   [stitch] stitched file created: <output>.ply
   ```
5. The stitched cloud loads into the 3D viewer
6. Walls, floor, and ceiling from both scans should align (not offset or doubled)
7. No large floating noise clusters should be visible

- [ ] **Step 5: Commit**

```bash
git add interface.py
git commit -m "Replace identity-guess ICP stitcher with FPFH+RANSAC+Point-to-Plane ICP pipeline

- Floor-normalize both scans before registration (reduces 6DOF to 3DOF)
- Statistical outlier removal before and after merge
- FPFH feature extraction + RANSAC for global initial alignment
- Point-to-Plane ICP refinement on full-res cleaned clouds
- Voxel downsample merged result for uniform density

Co-Authored-By: Claude Sonnet 4.6 <noreply@anthropic.com>"
```

---

## Self-Review Notes

- Spec requires: floor normalize ✓, pre-registration outlier removal ✓, FPFH ✓, RANSAC ✓, Point-to-Plane ICP ✓, post-merge outlier removal ✓, voxel downsample ✓
- Spec requires: no UI changes ✓ (StitchWorker and _stitch_clicked untouched)
- Spec requires: same function signature ✓
- `stabilize_scan_to_floor_frame_numpy` is defined in the same file and is directly callable — no import needed
- `copy` is already imported at the top of `interface.py`
- `np` (numpy) is already imported at the top of `interface.py`
- `o3d` is loaded lazily via `_import_open3d()` — inner function references use the local `o3d` variable, which is correct
- The `_floor_normalize_o3d` helper is defined inside the function scope to keep Open3D dependency lazy — this matches the existing pattern
