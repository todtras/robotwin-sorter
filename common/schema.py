"""
common/schema.py — 모듈 간 데이터 계약 (Data Contract)
=======================================================

★★★ 이 파일은 3인 공동 소유입니다. 합의 없이 수정하지 마세요. ★★★

세 사람이 각자 다른 모듈을 병렬로 개발할 수 있는 이유는, 서로의 내부 구현을
전혀 모르는 대신 "무엇을 주고받을지"만 여기서 못박아 두기 때문입니다.
이 파일이 흔들리면 Day 4 통합이 그대로 무너집니다.

데이터 흐름
-----------
    [웹캠]
      |  BGR numpy 배열 (480, 640, 3)
      v
    (1) vision.detector.TrashDetector        담당: 윤주연
      |  list[Detection]        <- 픽셀 좌표계
      v
    (2) integration.spawner.ObjectSpawner    담당: 진선우
      |  SortTask               <- 월드 좌표계 (미터)
      v
    (3) robot.arm_controller.ArmController   담당: 김태익
      |  bool (성공/실패)
      v
    (4) common.logger.SortLogger             담당: 진선우

변경 절차
---------
1. 스탠드업에서 제안 -> 3인 구두 합의
2. 이 파일 수정 후 즉시 커밋 & 푸시
3. 슬랙/카톡으로 "schema 바뀜, pull 해라" 공지
   (말로만 합의하고 코드를 안 고치면 반드시 사고가 납니다)
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Literal, Optional

# ---------------------------------------------------------------------------
# 1. 타입 별칭 (Type Aliases)
# ---------------------------------------------------------------------------
# Literal을 쓰는 이유: 그냥 str로 두면 "pet"을 "PET"이나 "petbottle"로 잘못 적어도
# 실행 시점까지 아무도 모릅니다. Literal로 좁혀두면 IDE와 타입 체커가 바로 잡아줍니다.

Category = Literal["can", "general", "pet"]
"""검출 대상 클래스. 3개로 고정. 유리/종이 추가 금지 (9일 일정에 라벨링이 안 끝남)."""

BinName = Literal["blue_bin", "green_bin", "gray_bin"]
"""분리수거함 식별자. 실제 좌표는 config.BIN_POSITIONS 참조."""

FailReason = Literal[
    "ik_failed",        # 로봇: 역기구학 해가 없음 (도달 불가능한 자세)
    "timeout",          # 로봇: 5초 안에 목표 지점 도달 실패
    "grasp_lost",       # 로봇: 이동 중 물체를 놓침 (제약 방식에선 거의 없어야 정상)
    "out_of_workspace", # 통합: 변환된 월드 좌표가 작업영역 밖
    "misclassified",    # 비전: 검출은 됐으나 클래스가 틀림 (사람이 눈으로 판정)
    "not_detected",     # 비전: 물체를 올렸는데 아예 검출 안 됨
    "unstable",         # 비전: 좌표가 계속 흔들려 안정화 판정을 통과 못 함
]
"""실패 사유 코드. 실험 5의 파이차트가 이 값들의 분포로 그려집니다.
새 사유가 필요하면 여기 추가하고 팀에 공지하세요."""


# ---------------------------------------------------------------------------
# 2. 클래스 매핑 (단일 진실 공급원, Single Source of Truth)
# ---------------------------------------------------------------------------

CLASS_NAMES: list[Category] = ["can", "general", "pet"]
"""YOLO 클래스 ID -> 클래스명 매핑.

    CLASS_NAMES[0] == "can"
    CLASS_NAMES[1] == "general"
    CLASS_NAMES[2] == "pet"

★ 이 순서는 pet/can/general이 아니라 can/general/pet입니다. Roboflow가
클래스를 알파벳순으로 강제 정렬해서 내보내기 때문에 실제 학습 데이터의
클래스 ID가 이 순서로 고정됩니다. "논리적인 순서"로 임의로 바꾸지 마세요.

★ 경고: dataset/data.yaml의 `names` 순서와 반드시 일치해야 합니다.
어긋나면 페트병을 캔으로 분류하는 버그가 나는데, 모델도 정상이고 코드도
정상이라 원인 찾는 데 반나절이 날아갑니다. 학습 직전에 두 파일을 나란히
띄워놓고 눈으로 대조하세요.
"""

CATEGORY_TO_BIN: dict[Category, BinName] = {
    "pet": "blue_bin",       # 투명 페트병, 플라스틱 음료병 -> 파란 통
    "can": "green_bin",      # 알루미늄 캔, 통조림캔       -> 초록 통
    "general": "gray_bin",   # 종이컵/비닐/티슈/일회용 수저 -> 회색 통
}
"""클래스 -> 수거함 매핑.

이 딕셔너리 하나만 고치면 전체 시스템의 분류 규칙이 바뀝니다.
로봇 모듈이나 통합 모듈에서 if문으로 하드코딩하지 마세요.
"""


# ---------------------------------------------------------------------------
# 3. Detection — 비전(1) -> 통합(2)
# ---------------------------------------------------------------------------

@dataclass
class Detection:
    """웹캠 프레임 한 장에서 검출된 물체 하나.

    좌표계: **이미지 좌표계**
        - 해상도 640 x 480 고정 (config.FRAME_WIDTH / FRAME_HEIGHT)
        - 원점 (0, 0)은 화면 **좌상단**
        - x는 오른쪽으로 증가, y는 **아래쪽**으로 증가
        - 단위는 픽셀(정수)

    ★ 함정: 이미지 y는 아래로 증가하는데 PyBullet 월드 Y는 로봇 기준 왼쪽으로
      증가합니다. 부호가 반대입니다. 호모그래피(Calibrator)가 자동으로 처리해
      주지만, 직접 선형식을 짜면 여기서 반드시 틀립니다.
      "로봇이 엉뚱한 방향으로 간다" -> 90%는 이 문제입니다.

    생성 예시 (vision/detector.py 안에서)::

        Detection(
            category="pet",
            class_id=2,
            pixel_x=320, pixel_y=240,
            confidence=0.87,
            bbox=(280, 200, 360, 280),
        )
    """

    category: Category
    """판별된 클래스명. CLASS_NAMES[class_id]와 항상 같아야 합니다."""

    class_id: int
    """YOLO가 뱉은 원본 클래스 ID. 0=can, 1=general, 2=pet.
    (Roboflow가 알파벳순으로 강제 정렬한 순서입니다.)"""

    pixel_x: int
    """바운딩박스 **중심**의 x 좌표. 범위 0 ~ 639.
    좌상단 모서리가 아니라 중심입니다. (x1 + x2) // 2 로 계산하세요."""

    pixel_y: int
    """바운딩박스 **중심**의 y 좌표. 범위 0 ~ 479."""

    confidence: float
    """모델 신뢰도. 0.0 ~ 1.0.
    config.CONF_THRESHOLD 미만은 detector 단계에서 이미 걸러져서 올라옵니다."""

    bbox: tuple[int, int, int, int]
    """(x1, y1, x2, y2) 좌상단/우하단 픽셀 좌표.
    좌표 변환에는 안 쓰이고 디버깅 화면에 사각형 그릴 때만 씁니다.
    물체 크기 추정(작으면 캔, 크면 페트병 식)에 쓰고 싶으면 여기서 뽑으세요."""

    timestamp: float = field(default_factory=time.time)
    """검출된 시각 (Unix epoch 초, float).
    지연시간 실험(실험 3)에서 캡처~검출 구간 측정의 기준점이 됩니다.
    직접 넣지 말고 기본값에 맡기세요."""

    def __post_init__(self) -> None:
        """생성 즉시 자기 검증. 잘못된 데이터가 파이프라인에 흘러드는 걸 막습니다.

        여기서 예외가 나면 detector 구현이 잘못된 겁니다. 통합 단계에서
        원인 모를 오작동을 겪는 것보다 지금 터지는 게 훨씬 낫습니다.
        """
        if self.category != CLASS_NAMES[self.class_id]:
            raise ValueError(
                f"category와 class_id 불일치: "
                f"category={self.category!r}, class_id={self.class_id} "
                f"(CLASS_NAMES[{self.class_id}]={CLASS_NAMES[self.class_id]!r}). "
                f"data.yaml의 클래스 순서를 확인하세요."
            )
        if not 0.0 <= self.confidence <= 1.0:
            raise ValueError(f"confidence는 0~1이어야 합니다: {self.confidence}")


# ---------------------------------------------------------------------------
# 4. SortTask — 통합(2) -> 로봇(3)
# ---------------------------------------------------------------------------

@dataclass
class SortTask:
    """로봇이 수행할 작업 하나. "이 물체를 집어서 저 통에 넣어라".

    좌표계: **PyBullet 월드 좌표계**
        - 원점 (0, 0, 0)은 로봇팔 베이스 중심
        - +X: 로봇 정면 / +Y: 로봇 기준 왼쪽 / +Z: 위쪽
        - 단위는 **미터**. 밀리미터로 넣으면 로봇이 1km 밖으로 팔을 뻗습니다.
        - 테이블 상면이 Z = 0.0

    로봇 모듈(태익)은 이 객체 하나만 받으면 다른 모듈 없이도 개발할 수 있습니다.
    tests/dummy_vision.py가 이걸 가짜로 만들어 줍니다.
    """

    body_id: int
    """PyBullet이 발급한 객체 고유 ID.
    p.createMultiBody()의 반환값이며, grasp()의 createConstraint에서
    childBodyUniqueId로 그대로 넘깁니다."""

    target_xyz: tuple[float, float, float]
    """집을 위치 (x, y, z), 단위 미터.
    z는 물체 **중심** 높이입니다. 4cm 정육면체면 z = 0.02.
    로봇은 여기서 바로 하강하지 말고 config.APPROACH_HEIGHT만큼 위로
    먼저 이동한 뒤 수직 하강하세요."""

    target_bin: BinName
    """투입할 수거함 이름. 실제 좌표는 config.BIN_POSITIONS[target_bin]에서 조회.
    좌표를 여기 직접 넣지 않는 이유: 수거함 위치를 옮길 때 config만 고치면
    되도록 하기 위함입니다."""

    category: Category
    """물체의 클래스. 로봇 동작에는 영향이 없고 로깅/디버깅용입니다.
    target_bin == CATEGORY_TO_BIN[category] 관계가 항상 성립해야 합니다."""

    source: Optional[Detection] = None
    """이 작업을 만들어낸 원본 검출 데이터.

    로깅할 때 픽셀 좌표와 confidence를 남기려면 필요합니다.
    더미 테스트에서는 None으로 둬도 됩니다. 따라서 이 값을 읽는 코드는
    반드시 None 체크를 하세요."""

    created_at: float = field(default_factory=time.time)
    """태스크 생성 시각. Detection.timestamp와의 차이가 좌표 변환 소요 시간."""

    def __post_init__(self) -> None:
        """카테고리와 수거함이 어긋나면 즉시 실패시킵니다.

        이게 없으면 "페트병인데 초록 통에 들어간다" 같은 버그를 데모 중에
        눈으로 발견하게 됩니다.
        """
        expected = CATEGORY_TO_BIN[self.category]
        if self.target_bin != expected:
            raise ValueError(
                f"category={self.category!r}는 {expected!r}로 가야 하는데 "
                f"target_bin={self.target_bin!r}로 지정됐습니다. "
                f"CATEGORY_TO_BIN을 사용해 매핑하세요."
            )
        if len(self.target_xyz) != 3:
            raise ValueError(f"target_xyz는 (x, y, z) 3원소여야 합니다: {self.target_xyz}")


# ---------------------------------------------------------------------------
# 5. SortResult — 로봇(3) -> 로거(4)
# ---------------------------------------------------------------------------

@dataclass
class SortResult:
    """한 사이클의 최종 결과. CSV 한 줄이 됩니다.

    ArmController.execute_task()는 계약상 bool을 반환하지만, 로거가 실패
    사유와 단계별 시간을 알아야 하므로 이 객체를 별도로 채워서 넘깁니다.
    (bool 반환은 유지하되, controller.last_result 같은 속성으로 노출하거나
     execute_task가 이 객체를 함께 만들어 두는 방식 중 편한 쪽으로.
     Day 3 스탠드업에서 태익-선우가 정하세요.)
    """

    task: SortTask
    """어떤 작업이었는지."""

    success: bool
    """수거함에 제대로 들어갔는가. 사람이 눈으로 판정해도 됩니다."""

    fail_reason: Optional[FailReason] = None
    """실패했다면 사유. success=True면 None."""

    # --- 단계별 소요 시간 (밀리초) ---------------------------------------
    # 실험 3(지연시간 분해)의 원자료입니다.
    # 측정 코드는 Day 3에 미리 심어두고 Day 7에 데이터만 모읍니다.
    t_capture_ms: float = 0.0    # 웹캠 프레임 읽기
    t_detect_ms: float = 0.0     # YOLO 추론 (보통 여기가 병목)
    t_transform_ms: float = 0.0  # 픽셀 -> 월드 좌표 변환
    t_ik_ms: float = 0.0         # 역기구학 계산
    t_execute_ms: float = 0.0    # 실제 팔 동작 (FSM 전체)

    @property
    def t_total_ms(self) -> float:
        """전체 소요 시간. 개별 값을 더한 것이므로 따로 저장하지 않습니다."""
        return (
            self.t_capture_ms
            + self.t_detect_ms
            + self.t_transform_ms
            + self.t_ik_ms
            + self.t_execute_ms
        )
