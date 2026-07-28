"""
common/logger.py — 실험 CSV 로깅
담당: 진선우 | ★ Day 2에 완성

보고서의 모든 숫자가 이 파일에서 나옵니다. 나중에 만들면 Day 6~7에
데이터를 다시 수집해야 합니다. 미루지 마세요.
"""

from __future__ import annotations

from pathlib import Path

import config
from common.schema import Detection, FailReason, SortResult, SortTask


class SortLogger:
    """한 사이클당 CSV 한 줄을 기록합니다.

    컬럼 순서는 config.CSV_COLUMNS 참조.
    """

    def __init__(self, log_dir: Path = config.LOG_DIR) -> None:
        """TODO(선우) Day 2:
          - log_dir이 없으면 생성
          - 파일명은 실행 시각 기준 (예: run_20260728_1430.csv)
            덮어쓰면 앞선 실험 데이터가 날아갑니다
          - 헤더 한 줄(config.CSV_COLUMNS) 먼저 기록
        """
        self.log_dir = log_dir
        self.path: Path | None = None

    def record(self, result: SortResult) -> None:
        """한 사이클 결과를 기록.

        TODO(선우): SortResult에서 값을 뽑아 config.CSV_COLUMNS 순서로 씁니다.
          result.task.source가 None일 수 있으니 픽셀 좌표는 None 체크 필요.
        """
        raise NotImplementedError

    def record_failure(self, detection: Detection, reason: FailReason) -> None:
        """로봇까지 못 간 실패(작업영역 밖 등)를 기록.

        ★ 이런 케이스를 빼먹으면 실험 5의 실패 사유 분포가 왜곡됩니다.
        """
        raise NotImplementedError

    def summary(self) -> dict:
        """성공률과 실패 사유별 개수를 집계. 발표 슬라이드에 바로 씁니다."""
        raise NotImplementedError
