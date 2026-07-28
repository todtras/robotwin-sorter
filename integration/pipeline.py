"""
integration/pipeline.py — 메인 진입점
담당: 진선우 | Day 3 ~ Day 4

실행::

    python -m integration.pipeline

전체 시스템을 이 파일 하나로 돌립니다. Day 4 통합의 무대입니다.
"""

from __future__ import annotations

import config


def main() -> None:
    """메인 루프.

    TODO(선우) Day 3~4:

        camera     = Camera();            camera.open()
        detector   = TrashDetector()
        calibrator = Calibrator()
        scene      = Scene();             scene.build()
        spawner    = ObjectSpawner()
        arm        = ArmController(scene.robot_id)
        logger     = SortLogger()

        while True:
            frame = camera.read()
            detections = detector.detect(frame)

            for det in detections:
                if not stability.is_stable(det.pixel_x, det.pixel_y):
                    continue                       # 아직 사람 손 위
                if is_duplicate(det):
                    continue                       # 이미 처리 중인 물체
                wx, wy = calibrator.pixel_to_world(det.pixel_x, det.pixel_y)
                if not calibrator.is_in_workspace(wx, wy):
                    logger.record_failure(det, "out_of_workspace")
                    continue
                task = spawner.spawn(det, (wx, wy))
                ok   = arm.execute_task(task)
                logger.record(task, ok, timings)
                spawner.remove(task.body_id)

            scene.step()

    ★ 동시성 주의: 로봇이 처리하는 동안에도 검출은 계속 들어옵니다.
      초기 버전은 큐에 쌓아두고 **순차 처리**로 단순하게 가세요.
      멀티스레드는 Day 8 이후 여유 있을 때만. 지금 하면 디버깅이 지옥입니다.

    ★ Day 4는 이 파이프라인에서 detector만 색상 기반 검출기로 바꿔
      좌표 변환 + 로봇 제어 연결만 먼저 검증합니다.
      (모델의 불확실성을 배제해야 실패 원인 범위가 좁아집니다)
    """
    raise NotImplementedError


if __name__ == "__main__":
    main()
