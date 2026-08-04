from __future__ import annotations
import random
import time

import config
from common.schema import CLASS_NAMES, Detection, SortResult

from integration.pipeline import Pipeline

pipeline = Pipeline(use_dummy=False, use_gui=False)
pipeline.start()

BIAS_PROB = 0.5      # 이 비율만큼 위험 구간(WORKSPACE_X 하한 근처)에서 좌표를 뽑음
BIAS_X_RANGE = (config.WORKSPACE_X[0], config.WORKSPACE_X[0] + 0.05)  # 하한 근처 5cm


def sample_xy() -> tuple[float, float, str]:
    if random.random() < BIAS_PROB:
        x = random.uniform(*BIAS_X_RANGE)
        zone = "edge"
    else:
        x = random.uniform(*config.WORKSPACE_X)
        zone = "normal"
    y = random.uniform(*config.WORKSPACE_Y)
    return x, y, zone


zone_stats: dict[str, dict[str, int]] = {
    "edge": {"total": 0, "success": 0},
    "normal": {"total": 0, "success": 0},
}

for category in ["pet", "can", "general"]:
    for i in range(20):
        x, y, zone = sample_xy()

        det = Detection(
            category=category, 
            class_id=CLASS_NAMES.index(category), 
            pixel_x=0, pixel_y=0, 
            confidence=0.9, 
            bbox=(0, 0, 0, 0))

        task = pipeline.spawner.spawn(det, (x, y))

        ok = pipeline.arm.execute_task(task)
        result = SortResult(
            task=task,
            success=ok,
            fail_reason=pipeline.arm.last_result.fail_reason if not ok else None,
            t_capture_ms=0,
            t_detect_ms=0, 
            t_transform_ms=0,
            t_ik_ms=pipeline.arm.last_result.t_ik_ms,
            t_execute_ms=pipeline.arm.last_result.t_execute_ms)
        
        pipeline.logger.record(result)
        pipeline.spawner.remove(task.body_id)

        zone_stats[zone]["total"] += 1
        if ok:
            zone_stats[zone]["success"] += 1
        print(f"{category:8s} {zone:6s} ({x:.2f},{y:.2f}) -> {'OK' if ok else 'FAIL: ' + str(result.fail_reason)}")

print(pipeline.logger.summary())
for zone, stats in zone_stats.items():
    total = stats["total"]
    rate = stats["success"] / total if total else 0.0
    print(f"  {zone}: {stats['success']}/{total} ({rate:.0%})")