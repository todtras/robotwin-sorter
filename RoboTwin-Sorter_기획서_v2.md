# RoboTwin-Sorter 프로젝트 기획서 (v2)
### 커스텀 YOLO & 디지털 트윈 기반 분리수거 자동 분류 시스템

| 항목 | 내용 |
|---|---|
| 프로젝트명 | RoboTwin-Sorter |
| 기간 | 2주 단기 인턴 (기획·PPT 1일 제외, 실개발 9일) |
| 근무 | 월~금 09:00 ~ 18:00 |
| 팀 구성 | 김태익(로봇 제어), 윤주연(AI 비전), 진선우(디지털 트윈 통합) |
| 산출물 | 최종 발표 + 결과 보고서 + 데모 영상 + 자체 구축 데이터셋 + GitHub 저장소 |
| 예산 | 0원 (개인 노트북 + 웹캠 + 사무실 폐기물) |

> **v1 → v2 주요 변경**
> 1. 분류 체계: 가연성/불가연성/재활용 → **PET / CAN / 일반** (분리수거 기준)
> 2. 인식 모델: COCO 사전학습 클래스 사용 → **자체 데이터셋 구축 후 파인튜닝**
> 3. 위 변경에 따라 일정·실험·리스크 전면 재조정 (5장 신설)

---

## 0. 이 문서를 읽는 법

- **2장**은 전원 필독. 좌표계와 인터페이스 규약이 여기 있고, 어긋나면 통합이 실패합니다.
- **5장(데이터셋 구축)** 은 윤주연 담당이지만, **라벨링 작업은 3명이 함께** 합니다. 전원 확인 필요.
- **6장 일정표**의 **DoD(Definition of Done)** 는 그날 퇴근 전 통과 기준입니다. 미달이면 다음 날 스탠드업에서 조정합니다.
- 결정을 바꿨다면 그날 안에 이 문서를 고치고 커밋하세요. **문서가 최신이 아니면 통합이 깨집니다.**

---

## 1. 프로젝트 개요

### 1.1 한 줄 정의
사무실에서 실제로 나오는 폐기물을 직접 촬영·라벨링해 **자체 데이터셋을 구축**하고, 이를 학습시킨 YOLOv8 모델로 웹캠 영상에서 PET·캔·일반쓰레기를 판별한 뒤, PyBullet 가상 환경(디지털 트윈)의 로봇팔이 해당 분리수거함으로 자동 투입하는 시스템.

### 1.2 왜 이 주제인가
- **추가 부품 0원**: 실물 로봇팔 없이 시뮬레이터로 대체하되, 입력은 진짜 카메라이므로 "현실 → 가상" 연동 서사가 성립.
- **3인 역할이 자연스럽게 분리**: 비전(데이터+모델) / 좌표변환·통합 / 로봇제어가 독립 모듈이며 인터페이스만 맞추면 병렬 개발 가능.
- **자체 데이터셋이 곧 산출물**: 사전학습 모델을 그대로 쓰는 것보다 기여도가 명확하고, 보고서에 데이터 수집·라벨링·학습·평가 전 과정을 서술할 수 있음.
- **정량 평가가 쉬움**: mAP, 클래스별 정밀도/재현율, 좌표 오차, 지연시간, 분류 성공률이 전부 숫자로 나옴.

### 1.3 분류 체계

| 클래스 ID | 클래스명 | 한글 | 대상 물체 | 수거함 |
|---|---|---|---|---|
| 0 | `pet` | 페트 | 투명 페트병 (라벨 제거/부착 무관), 플라스틱 음료병 | `blue_bin` |
| 1 | `can` | 캔 | 알루미늄 음료캔, 통조림캔 | `green_bin` |
| 2 | `general` | 일반 | 종이컵, 과자봉지, 비닐, 티슈, 일회용 수저 | `gray_bin` |

> **클래스는 3개로 고정합니다.** 유리병·종이류를 추가하고 싶어도 참으세요. 클래스가 늘면 필요한 이미지 수가 비례해서 늘고, 9일 일정에서 라벨링에 잡아먹힙니다. 확장은 보고서 "향후 과제"로 서술합니다.

### 1.4 성공 기준

| 등급 | 기준 |
|---|---|
| **필수 (Must)** | 자체 학습 모델의 검증셋 mAP50 ≥ 0.70. 물체 1개를 올리면 → 스폰 → 올바른 수거함 투입까지 10회 중 7회 이상 성공 |
| **목표 (Should)** | 3개 클래스 모두 실환경 인식 성공. 다중 물체(3개 이상) 순차 분류 |
| **도전 (Could)** | 고전 CV 기법과의 성능 비교 실험, 물체 겹침·조명 변화 대응 |

> 필수 기준은 **Day 7 종료 시점**까지 달성 목표입니다.

---

## 2. 시스템 설계 (전원 필독)

### 2.1 데이터 흐름

```
 [실물 웹캠]
      │  BGR 프레임 (640x480)
      ▼
 ┌──────────────────────────────┐
 │ ① 비전 모듈 (윤주연)          │  커스텀 YOLOv8n 추론 → 클래스 판별 → 좌표 안정화
 │    모델: runs/.../best.pt     │
 └──────────────────────────────┘
      │  Detection 객체 리스트 (픽셀 좌표)
      ▼
 ┌─────────────────────────┐
 │ ② 통합 모듈 (진선우)      │  픽셀→월드 좌표 변환 → PyBullet 객체 스폰
 └─────────────────────────┘
      │  SortTask 객체 (월드 좌표 + 목표 수거함)
      ▼
 ┌─────────────────────┐
 │ ③ 로봇 모듈 (김태익)  │  IK 계산 → FSM 기반 집기/이동/투하
 └─────────────────────┘
      │  성공/실패 결과
      ▼
 ┌─────────────────────┐
 │ ④ 로거 (공용)        │  CSV 기록 → 실험 데이터
 └─────────────────────┘
```

### 2.2 좌표계 정의 ★ 가장 중요

**세 사람이 같은 좌표계를 쓰는지 Day 1에 눈으로 확인하세요.** 통합 실패의 대부분이 여기서 발생합니다.

**(a) 이미지 좌표계 — 비전 모듈**
- 해상도: `640 x 480` **고정** (학습 데이터 촬영 해상도와 반드시 동일해야 함)
- 원점: 좌상단 `(0, 0)`, x는 우측(+), y는 하단(+), 단위 픽셀

**(b) PyBullet 월드 좌표계 — 통합·로봇 모듈**
- 원점: 로봇팔 베이스 중심 `(0, 0, 0)`
- `+X`: 로봇 정면 / `+Y`: 로봇 기준 좌측 / `+Z`: 위쪽, 단위 미터(m)
- 테이블 상면: `Z = 0.0`

**(c) 작업 영역 (Pick Area)**
```
X: 0.35 ~ 0.65 m
Y: -0.25 ~ 0.25 m
Z: 0.02 m  (물체 중심 높이)
```

**(d) 수거함 위치 (`config.py`에 정의)**

| 수거함 | 클래스 | 색상 | 좌표 (x, y, z) |
|---|---|---|---|
| `blue_bin` | `pet` | 파랑 | `(0.45, 0.45, 0.0)` |
| `green_bin` | `can` | 초록 | `(0.0, 0.60, 0.0)` |
| `gray_bin` | `general` | 진회색 | `(-0.45, 0.45, 0.0)` |

> KUKA iiwa의 도달거리는 약 0.8m입니다. 위 좌표는 모두 사정권이지만, 변경 시 **반드시 도달 가능 여부를 먼저 테스트**하세요.

### 2.3 인터페이스 규약 (Data Contract)

**Day 1 종료 전 `common/schema.py`로 코드화하고, 이후 변경 시 3인 합의를 거칩니다.**

```python
# common/schema.py
from dataclasses import dataclass, field
from typing import Literal, Optional
import time

Category = Literal["pet", "can", "general"]
BinName  = Literal["blue_bin", "green_bin", "gray_bin"]

# 클래스 → 수거함 매핑 (단일 진실 공급원)
CATEGORY_TO_BIN: dict[Category, BinName] = {
    "pet":     "blue_bin",
    "can":     "green_bin",
    "general": "gray_bin",
}

# 학습 시 클래스 ID 순서 — data.yaml과 반드시 일치시킬 것
CLASS_NAMES: list[Category] = ["pet", "can", "general"]


@dataclass
class Detection:
    """① 비전 → ② 통합 으로 전달되는 단위 데이터"""
    category: Category          # 판별된 클래스
    class_id: int               # 0=pet, 1=can, 2=general
    pixel_x: int                # 바운딩박스 중심 x (0~639)
    pixel_y: int                # 바운딩박스 중심 y (0~479)
    confidence: float           # 0.0 ~ 1.0
    bbox: tuple[int, int, int, int]  # (x1, y1, x2, y2) — 디버깅·시각화용
    timestamp: float = field(default_factory=time.time)


@dataclass
class SortTask:
    """② 통합 → ③ 로봇 으로 전달되는 단위 데이터"""
    body_id: int                            # PyBullet 객체 고유 ID
    target_xyz: tuple[float, float, float]  # 집을 위치 (월드 좌표, m)
    target_bin: BinName                     # 투입할 수거함
    category: Category                      # 클래스 (로깅용)
    source: Optional[Detection]             # 원본 검출 데이터 (추적용)
```

### 2.4 모듈별 함수 시그니처 (계약)

**서로 이 시그니처만 믿고 개발합니다. 내부 구현은 자유이되, 시그니처 변경은 합의 필요.**

```python
# vision/detector.py  ── 담당: 윤주연
class TrashDetector:
    def __init__(self, model_path: str = "models/best.pt", conf_threshold: float = 0.5): ...
    def detect(self, frame) -> list[Detection]:
        """BGR 프레임을 받아 Detection 리스트 반환. 검출 없으면 빈 리스트."""


# integration/calibration.py  ── 담당: 진선우
class Calibrator:
    def __init__(self, image_points: list, world_points: list): ...
    def pixel_to_world(self, px: int, py: int) -> tuple[float, float]: ...
    def is_in_workspace(self, x: float, y: float) -> bool: ...


# integration/spawner.py  ── 담당: 진선우
class ObjectSpawner:
    def spawn(self, detection: Detection, world_xy: tuple[float, float]) -> SortTask: ...
    def remove(self, body_id: int) -> None: ...


# robot/arm_controller.py  ── 담당: 김태익
class ArmController:
    def __init__(self, urdf_path: str = "kuka_iiwa/model.urdf"): ...
    def execute_task(self, task: SortTask) -> bool: ...
    def move_to(self, xyz: tuple, timeout: float = 5.0) -> bool: ...
    def grasp(self, body_id: int) -> bool: ...
    def release(self) -> None: ...
    def go_home(self) -> None: ...
```

---

## 3. 촬영 환경 세팅 (★ 데이터 품질을 좌우함)

**이 절은 Day 1 오전에 3명이 함께, 한 번에 확정합니다. 여기서 대충 하면 이후 전부 다시 해야 합니다.**

### 3.1 필수 조건

1. **웹캠을 책상 위 수직으로 고정** — 책 더미, 모니터 암, 삼각대 무엇이든 좋으나 **절대 움직이지 않게** 테이프로 고정. 움직이면 캘리브레이션을 다시 해야 하고, 학습 데이터와 실환경이 달라져 인식률이 떨어집니다.
2. **작업 영역에 어두운 배경 깔기** — 검은 도화지, 검은 천, 어두운 마우스패드 등. **이건 선택이 아니라 필수입니다.**
   - **투명 페트병은 배경이 비쳐 보여서** 밝은 책상 위에서는 경계가 거의 안 잡힙니다. 어두운 배경에서 대비가 극적으로 좋아집니다.
   - **알루미늄 캔은 반사광(하이라이트)** 때문에 밝은 배경에서 형태가 뭉개집니다.
3. **작업 영역 네 귀퉁이에 마커** — 검은 테이프나 색 포스트잇. 캘리브레이션 대응점으로 사용.
4. **조명 고정** — 형광등 켠 상태로 통일. 스탠드를 쓴다면 위치를 고정하고 사진을 찍어 기록.

### 3.2 물체 준비

| 클래스 | 최소 개체 수 | 조달 방법 |
|---|---|---|
| `pet` | 서로 다른 페트병 4~5종 | 사무실 음료 페트병 세척. 라벨 붙은 것/뗀 것 섞기 |
| `can` | 서로 다른 캔 4~5종 | 음료캔. 찌그러진 것도 1~2개 포함 |
| `general` | 5~6종 | 종이컵, 과자봉지, 비닐, 티슈, 일회용 수저 |

> **개체 다양성이 이미지 장수보다 중요합니다.** 같은 페트병 하나를 100장 찍는 것보다, 5종을 20장씩 찍는 게 훨씬 잘 학습됩니다.

---

## 4. 팀원별 상세 구현 명세

### 🤖 4.1 김태익 — 로봇 제어 & 시뮬레이션

**책임 범위:** PyBullet 씬 구성 / IK 제어 / 파지 로직 / FSM

> v1 대비 변경 없음. 수거함 색상·이름만 2.2절 (d)로 교체하면 됩니다.

#### Step 1. 씬 구성 (`robot/scene.py`)
- `p.connect(p.GUI)` → `p.setAdditionalSearchPath(pybullet_data.getDataPath())`
- `plane.urdf` 로드 후 `kuka_iiwa/model.urdf`를 `useFixedBase=True`로 로드
- 수거함 3개는 `p.createVisualShape(p.GEOM_BOX)` + `p.createMultiBody(baseMass=0)`로 색상 박스 배치
  - 실제로 "담기는" 물리 구현은 불필요. 상공에서 떨어뜨리면 성공으로 간주

> **로봇 선택:** `franka_panda/panda.urdf`도 `pybullet_data`에 있고 그리퍼가 달려 더 현실적이지만 제어가 까다롭습니다. **KUKA iiwa로 먼저 완성한 뒤 여유가 있으면 교체**하세요. URDF 경로와 엔드이펙터 인덱스만 바꾸면 되도록 설계해 두면 됩니다.

#### Step 2. IK 이동 (`robot/arm_controller.py`)
```python
joint_poses = p.calculateInverseKinematics(
    bodyUniqueId=self.robot_id,
    endEffectorLinkIndex=self.ee_index,
    targetPosition=target_xyz,
    targetOrientation=p.getQuaternionFromEuler([0, math.pi, 0]),  # 그리퍼 수직 하향
    maxNumIterations=100,
    residualThreshold=1e-4,
)
for i, pose in enumerate(joint_poses):
    p.setJointMotorControl2(self.robot_id, i, p.POSITION_CONTROL,
                            targetPosition=pose, force=500)
```
- **도달 판정:** `p.getLinkState()`로 실제 위치를 읽어 목표와 거리가 `1cm` 이내면 도달
- **타임아웃:** 5초 내 미도달 시 `False` 반환 후 `ERROR` 전이. 무한 대기 금지

#### Step 3. 파지 (★ 함정)

물리 마찰 기반 파지는 물체가 미끄러져 계속 실패합니다. **1차 구현은 반드시 제약(Constraint) 방식으로.**

```python
def grasp(self, body_id):
    self.constraint_id = p.createConstraint(
        parentBodyUniqueId=self.robot_id,
        parentLinkIndex=self.ee_index,
        childBodyUniqueId=body_id,
        childLinkIndex=-1,
        jointType=p.JOINT_FIXED,
        jointAxis=[0, 0, 0],
        parentFramePosition=[0, 0, 0.05],
        childFramePosition=[0, 0, 0],
    )
    return True

def release(self):
    if self.constraint_id is not None:
        p.removeConstraint(self.constraint_id)
        self.constraint_id = None
```
> 마찰 방식도 한 번은 시도해 실패 데이터를 남기세요. 보고서에 "마찰 기반 파지 시도 → 안정성 문제 → 제약 기반 전환"으로 쓰면 좋은 기술적 의사결정 서술이 됩니다.

#### Step 4. FSM (`robot/fsm.py`)

```python
from enum import Enum, auto

class RobotState(Enum):
    IDLE        = auto()  # 새 SortTask 수신 대기
    APPROACH    = auto()  # 물체 위 10cm 상공으로 이동
    DESCEND     = auto()  # 물체 높이까지 수직 하강
    GRASP       = auto()  # 제약 생성 (집기)
    LIFT        = auto()  # 15cm 상승 (이동 중 충돌 방지)
    MOVE_TO_BIN = auto()  # 목표 수거함 상공으로 이동
    RELEASE     = auto()  # 제약 해제 (놓기)
    RETURN      = auto()  # 홈 포지션 복귀
    ERROR       = auto()  # IK 실패·타임아웃 → 홈 복귀 후 IDLE
```

| 전이 | 조건 |
|---|---|
| IDLE → APPROACH | `SortTask` 수신 & 목표가 작업영역 내 |
| APPROACH → DESCEND | 상공 도달 (오차 1cm 이내) |
| DESCEND → GRASP | 물체 높이 도달 |
| GRASP → LIFT | 제약 생성 완료 |
| LIFT → MOVE_TO_BIN | 안전 높이 도달 |
| MOVE_TO_BIN → RELEASE | 수거함 상공 도달 |
| RELEASE → RETURN | 제약 해제 완료 |
| RETURN → IDLE | 홈 포지션 도달 |
| 임의 → ERROR | IK 실패 또는 5초 타임아웃 |
| ERROR → IDLE | 홈 복귀 완료 (해당 태스크는 실패 기록) |

**출력물:** `ArmController.execute_task(task) -> bool`

**독립 개발 방법:** 다른 모듈 없이 `SortTask`를 손으로 만들어 테스트하세요.
```python
fake_task = SortTask(body_id=spawn_test_box(), target_xyz=(0.5, 0.1, 0.02),
                     target_bin="blue_bin", category="pet", source=None)
controller.execute_task(fake_task)
```

---

### 👁️ 4.2 윤주연 — 데이터셋 구축 & 커스텀 YOLO

**책임 범위:** 데이터 수집·라벨링 총괄 / 모델 학습 / 추론 파이프라인 / 좌표 안정화

이 역할은 **5장(데이터셋 구축 및 학습)** 이 사실상 전부이므로, 5장을 정독하고 그대로 따라가면 됩니다. 여기서는 학습 이후의 추론 부분만 다룹니다.

#### Step 1. 웹캠 입력 (`vision/camera.py`)
```python
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
```
**학습 데이터 촬영 때와 완전히 같은 설정**이어야 합니다. 해상도가 다르면 인식률이 눈에 띄게 떨어집니다.

#### Step 2. 추론 (`vision/detector.py`)

```python
from ultralytics import YOLO
from common.schema import Detection, CLASS_NAMES

class TrashDetector:
    def __init__(self, model_path="models/best.pt", conf_threshold=0.5):
        self.model = YOLO(model_path)
        self.conf = conf_threshold

    def detect(self, frame) -> list[Detection]:
        results = self.model(frame, imgsz=320, conf=self.conf, verbose=False)
        detections = []
        for box in results[0].boxes:
            cls_id = int(box.cls[0])
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            detections.append(Detection(
                category=CLASS_NAMES[cls_id],
                class_id=cls_id,
                pixel_x=(x1 + x2) // 2,
                pixel_y=(y1 + y2) // 2,
                confidence=float(box.conf[0]),
                bbox=(x1, y1, x2, y2),
            ))
        return detections
```

#### Step 3. 좌표 안정화 (`vision/stabilizer.py`)

```python
from collections import deque

class MovingAverageFilter:
    def __init__(self, window: int = 5):
        self.buf_x = deque(maxlen=window)
        self.buf_y = deque(maxlen=window)

    def update(self, px, py):
        self.buf_x.append(px)
        self.buf_y.append(py)
        return int(sum(self.buf_x) / len(self.buf_x)), int(sum(self.buf_y) / len(self.buf_y))
```

- **정지 판정 (필수):** 사람이 물체를 손에서 놓기 전에 로봇이 출발하면 안 됩니다. 최근 10프레임 동안 중심 좌표 변화가 `5px` 미만일 때만 "안정됨"으로 판단해 통합 모듈에 전달하세요. 이게 없으면 데모 중 오작동이 반복됩니다.
- **중복 방지:** 같은 물체가 매 프레임 새 태스크로 전달되지 않도록, 처리 중인 좌표 반경 `5cm` 이내는 무시. 진선우와 협의해 어느 쪽에서 처리할지 결정하세요.

#### Step 4. 성능 튜닝
- 커스텀 `yolov8n`은 CPU·320 해상도에서 대략 8~15 FPS 예상
- 부담되면 3프레임당 1회만 추론하고 사이는 이전 결과 유지

**출력물:** 학습된 `models/best.pt` + `TrashDetector.detect(frame) -> list[Detection]`

---

### 🔗 4.3 진선우 — 디지털 트윈 & 시스템 통합

**책임 범위:** 좌표 캘리브레이션 / 객체 스폰 / 전체 파이프라인 / 로깅

#### Step 1. 좌표 변환 (`integration/calibration.py`)

카메라를 완벽한 수직으로 고정하기는 어렵기 때문에 선형 보간 대신 **호모그래피**를 씁니다. 코드량은 같고 기울기를 자동 보정합니다.

```python
import cv2, numpy as np

class Calibrator:
    def __init__(self, image_points, world_points):
        """image_points: 작업영역 네 귀퉁이의 픽셀 좌표 4개
           world_points: 대응하는 월드 좌표 4개 (미터)"""
        self.H = cv2.getPerspectiveTransform(
            np.float32(image_points), np.float32(world_points))

    def pixel_to_world(self, px, py):
        pt = np.float32([[[px, py]]])
        out = cv2.perspectiveTransform(pt, self.H)
        return float(out[0][0][0]), float(out[0][0][1])
```

**대응점 잡는 법:** 3.1절에서 붙인 네 귀퉁이 마커의 픽셀 좌표를 화면에서 읽고, 월드 좌표는 2.2절 (c)의 작업 영역 값을 씁니다.
```python
image_points = [(120, 90), (520, 90), (520, 400), (120, 400)]   # ← 실측값으로 교체
world_points = [(0.35, 0.25), (0.65, 0.25), (0.65, -0.25), (0.35, -0.25)]
```

**검증:** 자로 잰 실제 위치에 물체를 놓고 변환 결과와 비교. **오차 2cm 이내** 목표. 이 데이터가 실험 2의 원자료가 됩니다.

#### Step 2. 동적 객체 스폰 (`integration/spawner.py`)

```python
CATEGORY_COLOR = {
    "pet":     [0.2, 0.4, 1.0, 1],   # 파랑
    "can":     [0.2, 0.8, 0.3, 1],   # 초록
    "general": [0.4, 0.4, 0.4, 1],   # 진회색
}

def spawn(self, detection, world_xy):
    half = 0.02  # 한 변 4cm 정육면체
    vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[half]*3,
                              rgbaColor=CATEGORY_COLOR[detection.category])
    col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[half]*3)
    body_id = p.createMultiBody(
        baseMass=0.1,
        baseCollisionShapeIndex=col,
        baseVisualShapeIndex=vis,
        basePosition=[world_xy[0], world_xy[1], half],
    )
    return SortTask(
        body_id=body_id,
        target_xyz=(world_xy[0], world_xy[1], half),
        target_bin=CATEGORY_TO_BIN[detection.category],
        category=detection.category,
        source=detection,
    )
```

> **여유가 되면:** 페트는 원기둥(`GEOM_CYLINDER`), 캔은 짧은 원기둥, 일반은 박스로 형상을 구분하면 디지털 트윈다운 화면이 나와 데모 인상이 좋아집니다. 우선순위는 낮으니 Day 8 이후에.

#### Step 3. 파이프라인 통제 (`integration/pipeline.py`)

```
while True:
    frame = camera.read()
    detections = detector.detect(frame)          # ① 비전
    for det in detections:
        if 이미_처리중인_좌표(det): continue
        wx, wy = calibrator.pixel_to_world(det.pixel_x, det.pixel_y)
        if not calibrator.is_in_workspace(wx, wy): continue
        task = spawner.spawn(det, (wx, wy))       # ② 스폰
        ok = arm.execute_task(task)               # ③ 로봇
        logger.record(task, ok, latencies)        # ④ 로깅
        spawner.remove(task.body_id)
    p.stepSimulation()
```

- **동시성 주의:** 로봇이 처리하는 동안에도 검출은 계속 들어옵니다. 초기 버전은 **큐에 쌓아두고 순차 처리**로 단순하게. 멀티스레드는 Day 8 이후 여유 있을 때만.

#### Step 4. 로깅 (`common/logger.py`)

**보고서의 모든 숫자가 여기서 나옵니다. Day 2에 완성하세요.**

```
timestamp, class_name, class_id, confidence,
pixel_x, pixel_y, world_x, world_y,
t_capture_ms, t_detect_ms, t_transform_ms, t_ik_ms, t_execute_ms, t_total_ms,
success, fail_reason
```
`fail_reason` 예시: `ik_failed`, `timeout`, `out_of_workspace`, `misclassified`, `not_detected`, `grasp_lost`

**출력물:** `python -m integration.pipeline` 실행 가능 + 실험 CSV

**독립 개발 방법 (★ Day 2 오전 최우선):** 비전·로봇 모듈 없이도 개발 가능하도록 **더미 모듈을 먼저 만드세요.**
```python
# tests/dummy_vision.py — 랜덤 Detection 생성 (pet/can/general 무작위)
# tests/dummy_robot.py  — 항상 True 반환하고 1초 sleep
```
이게 있으면 세 사람이 Day 2부터 서로를 기다리지 않습니다.

---

## 5. 데이터셋 구축 및 모델 학습 ★신설

> **총괄: 윤주연 / 라벨링 작업: 3인 공동**
> 이 장이 v2의 핵심이자 가장 큰 신규 작업량입니다. 예상 소요 **약 1.5일**.

### 5.1 전략: 파인튜닝 (밑바닥 학습 아님)

**`yolov8n.pt` 가중치에서 출발해 우리 3개 클래스로 파인튜닝합니다.**

- 밑바닥 학습(`yolov8n.yaml`)은 수만 장이 필요해 9일 일정에서 불가능합니다.
- 파인튜닝은 사전학습된 특징 추출 능력을 물려받으므로 **300장 수준으로도 충분히 동작**합니다.
- COCO의 기존 80개 클래스는 전부 버리고 우리 3개 클래스만 남습니다. 즉 **"기성 모델을 그대로 쓰는 것"이 아니라 우리 데이터로 만든 우리 모델**입니다. 보고서에도 전이학습(Transfer Learning) 기법으로 명확히 서술할 수 있습니다.

### 5.2 데이터 수집

**목표 수량**

| 클래스 | 원본 이미지 | 비고 |
|---|---|---|
| `pet` | 100장 이상 | 개체 4~5종 × 위치·각도 변화 |
| `can` | 100장 이상 | 개체 4~5종 |
| `general` | 100장 이상 | 개체 5~6종 |
| **합계** | **300장 이상** | 증강 후 약 900장 |

**촬영 규칙 (이걸 지켜야 인식률이 나옵니다)**

1. **실제 배포 환경 그대로 촬영.** 3장에서 고정한 웹캠·조명·배경을 그대로 씁니다. 다른 곳에서 찍은 사진이나 인터넷 이미지를 섞으면 오히려 성능이 떨어집니다.
2. **작업 영역 전체에 골고루 배치.** 한쪽에만 놓고 찍으면 반대편에서 인식이 안 됩니다.
3. **회전 다양성 확보.** 위에서 내려다보는 뷰이므로 물체를 여러 방향으로 돌려가며 촬영.
4. **1장에 1~3개 물체.** 전부 단독 촬영하면 다중 물체 상황에서 약해집니다. 30% 정도는 2~3개를 함께 놓고 촬영하세요.
5. **일부는 살짝 겹치게.** 완전히 분리된 상태만 학습하면 실환경에서 무너집니다.
6. **찌그러진 캔, 구겨진 봉지 등 변형된 상태도 포함.**

**수집 스크립트 (`tools/capture.py`)**

스페이스바를 누를 때마다 프레임을 저장하는 간단한 도구를 만들어 쓰세요. 물체 배치를 바꾸고 스페이스를 누르는 식으로 반복하면 **한 시간에 200~300장** 수집됩니다.

```python
import cv2, os, time

os.makedirs("dataset/raw", exist_ok=True)
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
count = 0

while True:
    ret, frame = cap.read()
    if not ret:
        break
    cv2.imshow("capture (SPACE=save, Q=quit)", frame)
    key = cv2.waitKey(1) & 0xFF
    if key == ord(' '):
        path = f"dataset/raw/img_{int(time.time()*1000)}.jpg"
        cv2.imwrite(path, frame)
        count += 1
        print(f"[{count}] saved: {path}")
    elif key == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
```

### 5.3 라벨링

**도구 선택**

| 도구 | 방식 | 장점 | 비고 |
|---|---|---|---|
| **Roboflow** | 웹 브라우저 | 설치 불필요, 증강·분할·YOLO 포맷 내보내기 자동, 3명 동시 작업 가능 | **1순위 추천.** 무료 티어 조건은 가입 시 확인 필요 |
| labelImg | 로컬 설치 | 오프라인, 가벼움 | 설치가 정책에 막힐 수 있음 |
| CVAT | 웹/로컬 | 기능 풍부 | 3명·300장 규모엔 과합니다 |

> **Roboflow를 1순위로 두는 이유:** 노트북에서 애플리케이션 제어 정책 차단 이력이 있으므로, 브라우저에서 도는 도구가 리스크가 가장 적습니다. 게다가 3명이 계정을 공유해 동시에 라벨링할 수 있어 작업 시간이 1/3로 줄어듭니다.

**라벨링 규칙 (3명이 동일하게 지켜야 함)**

1. 바운딩박스는 **물체에 딱 맞게.** 여유를 두지 마세요.
2. **일부만 보이는 물체도 보이는 부분만 박스 처리.** 아예 빼면 모델이 "여긴 아무것도 없다"로 학습해 버립니다.
3. 물체가 화면 가장자리에 절반 이상 잘려 있으면 그 이미지는 폐기.
4. **애매한 것은 즉시 팀에 물어보고 결정을 기록.** 예: "투명 플라스틱 컵은 pet인가 general인가" → 결정 후 `docs/labeling_rules.md`에 적어두세요. 사람마다 다르게 라벨링하면 모델이 혼란스러워합니다.

**작업 분배:** 300장을 3명이 100장씩. 개인당 대략 1~1.5시간. **Day 1 오후에 다 같이 앉아서 한 번에 끝내세요.** 나눠서 며칠에 걸치면 기준이 흔들립니다.

### 5.4 데이터 증강 및 분할

Roboflow에서 자동 처리하거나 직접 설정합니다.

**증강 (원본 1장 → 3장)**
- 회전: ±90° (탑다운 뷰이므로 **회전 증강이 특히 효과적**)
- 밝기: ±25%
- 노출/대비: ±15%
- 블러: 약간 (모션 블러 대응)

> **좌우/상하 반전은 넣어도 무방**합니다. 탑다운 뷰라 반전된 형태도 실제로 나타날 수 있습니다.

**분할**
```
train : val : test = 70 : 20 : 10
```
`test` 셋은 **학습·튜닝에 절대 쓰지 말고**, 최종 성능 보고용으로만 씁니다. 보고서에 "테스트셋 mAP"를 쓰려면 이 원칙을 지켜야 합니다.

### 5.5 `data.yaml`

```yaml
# dataset/data.yaml
path: ../dataset
train: images/train
val: images/val
test: images/test

nc: 3
names:
  0: pet
  1: can
  2: general
```

> `names`의 순서가 `common/schema.py`의 `CLASS_NAMES`와 **반드시 일치**해야 합니다. 어긋나면 페트를 캔으로 분류하는 버그가 나고, 원인 찾기가 매우 어렵습니다.

### 5.6 학습

**방법 A — Google Colab 무료 GPU (권장)**

브라우저에서 돌아가므로 노트북 정책 문제를 완전히 우회합니다. 학습 시간 **5~15분**.

```python
# Colab 노트북
!pip install ultralytics
from google.colab import files
# dataset.zip 업로드 후 압축 해제
!unzip -q dataset.zip -d /content/

!yolo detect train \
    data=/content/dataset/data.yaml \
    model=yolov8n.pt \
    epochs=100 \
    imgsz=320 \
    batch=16 \
    patience=20 \
    project=/content/runs

# 학습 완료 후 best.pt 다운로드
files.download('/content/runs/train/weights/best.pt')
```

받은 `best.pt`를 로컬 `models/best.pt`에 두고 추론만 로컬 CPU로 돌립니다. **추론은 CPU로도 충분히 빠릅니다.**

**방법 B — 로컬 CPU 학습 (Colab이 막힐 경우)**

```bash
yolo detect train data=dataset/data.yaml model=yolov8n.pt \
     epochs=50 imgsz=320 batch=8 patience=15 device=cpu
```
- 예상 소요: **1~3시간** (노트북 사양에 따라 편차 큼)
- **점심시간이나 퇴근 후에 돌려두세요.** 근무 시간을 학습 대기로 날리면 안 됩니다.
- `epochs`를 50으로 줄이고 `patience`로 조기 종료를 걸어 시간을 아낍니다.

**하이퍼파라미터 근거**

| 파라미터 | 값 | 이유 |
|---|---|---|
| `model` | `yolov8n.pt` | 가장 작은 모델. CPU 추론 속도 확보 |
| `imgsz` | `320` | 640 대비 약 2배 빠름. 물체가 크게 찍히는 탑다운 뷰라 320으로 충분 |
| `epochs` | `100` (Colab) / `50` (CPU) | 소규모 데이터셋이라 과적합 주의 |
| `patience` | `20` | 개선 없으면 조기 종료 |
| `batch` | `16` (GPU) / `8` (CPU) | 메모리에 맞춰 조정 |

### 5.7 평가 및 재학습 판단

학습이 끝나면 `runs/train/` 아래에 결과가 자동 생성됩니다. **이 파일들이 그대로 보고서 그림이 됩니다.**

| 파일 | 내용 | 보고서 활용 |
|---|---|---|
| `results.png` | 손실·mAP 학습 곡선 | 학습 과정 서술 |
| `confusion_matrix.png` | 클래스 혼동 행렬 | **어떤 클래스가 서로 헷갈리는지** — 가장 좋은 분석 재료 |
| `PR_curve.png` | 정밀도-재현율 곡선 | 임계값 선택 근거 |
| `val_batch*_pred.jpg` | 검증셋 예측 시각화 | 정성 평가 |

**판단 기준**

| mAP50 | 판단 | 조치 |
|---|---|---|
| ≥ 0.85 | 양호 | 그대로 진행 |
| 0.70 ~ 0.85 | 사용 가능 | 진행하되 혼동 행렬 보고 약한 클래스 이미지 보강 |
| < 0.70 | 부족 | **데이터를 더 모으세요.** 하이퍼파라미터를 만지는 것보다 이미지 100장 추가가 훨씬 효과적입니다 |

> **`general` 클래스가 가장 낮게 나올 가능성이 큽니다.** 종이컵·비닐·티슈처럼 생김새가 제각각인 것들을 한 클래스로 묶었기 때문입니다. 이건 실패가 아니라 **분석할 거리**입니다. 보고서에 "클래스 내 형태 다양성이 큰 general 클래스의 성능이 낮았으며, 이는 세분화된 클래스 설계가 필요함을 시사한다"고 쓰면 좋은 고찰이 됩니다.

---

## 6. 개발 일정 (9일)

> 기획·PPT 작성일은 완료. 아래 Day 1이 실개발 첫날입니다.
> 매일 **09:00~09:15 스탠드업**, **17:30~18:00 커밋 + 일지 기록**.

### Day 1 (화) — 환경 구축 · 데이터 수집 · 라벨링

| | 오전 | 오후 |
|---|---|---|
| **공통** | 저장소 생성, `common/schema.py` 작성·커밋, **촬영 환경 세팅(3장)** | **★ 전원 라벨링 작업 (100장씩)** |
| 태익 | PyBullet 동작 확인, KUKA 로드 | 라벨링 → 완료 후 수거함 배치 |
| 주연 | **`ultralytics` 설치 가능 여부 확인 (최우선)**, 촬영 스크립트 작성 후 300장 수집 | 라벨링 총괄, 규칙 문서화 |
| 선우 | 개발 환경 점검, `requirements.txt` 확정 | 라벨링 → 완료 후 캘리브레이션 4점 실측 |

> **DoD:** ① `schema.py` 커밋 후 전원 pull ② 전원 `import pybullet` 성공 ③ 원본 이미지 300장 수집 완료 ④ 라벨링 80% 이상 진척

### Day 2 (수) — 학습 + 코어 모듈 착수

| | 오전 | 오후 |
|---|---|---|
| 주연 | 라벨링 마무리, 증강·분할, `data.yaml` 작성 | **Colab 학습 실행** → `best.pt` 확보, 지표 확인 |
| 태익 | `move_to()` IK 이동 구현 | 제약 기반 `grasp()`/`release()` 구현 |
| 선우 | **더미 모듈 2종 작성 (최우선)** | `Calibrator` 구현 + 오차 실측 |

> **DoD:** ① 학습 완료, mAP50 확인 ② 태익·선우 각자 모듈 단독 동작

### Day 3 (목) — 모듈 완성

| | 오전 | 오후 |
|---|---|---|
| 주연 | `TrashDetector` 구현, 실시간 추론 화면 확인 | 이동평균 필터 + 정지 판정, FPS 측정 |
| 태익 | FSM 구현, `execute_task()` 완성 | 더미 태스크 10회 반복 → 성공률 확인 |
| 선우 | `ObjectSpawner` 구현 | `logger.py` 완성, CSV 기록 검증 |

> **DoD:** 주연 — 실물 3종이 화면에서 올바른 라벨로 인식됨 / 태익 — 더미 좌표로 10회 중 8회 성공 / 선우 — 더미 데이터로 CSV 정상 생성

### Day 4 (금) — ★ 1차 MVP 통합

| | 오전 | 오후 |
|---|---|---|
| 전원 | **YOLO 없이** 색상 포스트잇으로 좌표 매핑 → 로봇 이동까지 연결 | 통합 실패 지점 목록화, 원인별 담당 배정 |

색상 기반으로 먼저 붙이는 이유: 모델의 불확실성을 배제하고 **좌표 변환과 로봇 제어의 연결만** 검증하기 위함입니다. 문제가 나오면 원인 범위가 좁습니다.

> **DoD (체크포인트 1):** 포스트잇을 놓으면 박스가 스폰되고 로봇이 수거함에 넣는 사이클 **최소 1회** 성공. 미달 시 주말 전 원인 회의

### Day 5 (월) — 커스텀 모델 연동

| | 오전 | 오후 |
|---|---|---|
| 주연 | 색상 검출기를 `TrashDetector`로 교체 | 실환경 오인식 케이스 수집, 필요 시 이미지 보강 |
| 선우 | 교체에 따른 파이프라인 수정 | 좌표 오차 재측정, 캘리브레이션 미세조정 |
| 태익 | 실물 좌표 기준 IK 실패 대응 | 작업영역 경계 예외처리 |

> **DoD:** PET·CAN·일반 각각 최소 1회씩 올바른 수거함으로 분류 성공

### Day 6 (화) — 안정화 및 다중 물체

| | 오전 | 오후 |
|---|---|---|
| 전원 | 연속 20회 반복 테스트, 실패 케이스 분류·수정 | 다중 물체 순차 분류 로직 완성 |
| 주연 | (필요 시) 보강 데이터로 재학습 | 재학습 모델 교체·검증 |
| 선우 | 실패 사유별 로깅 강화 | 지연시간 단계별 측정 코드 삽입 |

> **DoD (체크포인트 2):** 필수 성공 기준(10회 중 7회) 달성. 미달 시 Day 7에 **스코프 축소** — 다중 물체를 포기하고 단일 물체 안정화에 집중

### Day 7 (수) — 예외 처리 및 실험 데이터 수집

| | 오전 | 오후 |
|---|---|---|
| 태익 | 특이점·관절 한계 예외처리, ERROR 복구 검증 | 실험 자동화 지원 |
| 주연 | 실험 1(모델 성능) 데이터 정리 | 실험 4(고전 CV 비교) — 여유 시 |
| 선우 | 실험 3(지연시간) 30회 수집 | 실험 2 데이터 취합, 그래프 초안 |

> **DoD:** 7장의 실험 1·2·3 데이터 확보

### Day 8 (목) — 데모 영상 촬영 및 보고서

| | 오전 | 오후 |
|---|---|---|
| 전원 | **데모 영상 촬영** (웹캠 + PyBullet 나란히, 10~15초) | 각자 담당 파트 보고서 초안 |
| 선우 | 영상 편집, 그래프 최종화 | 보고서 취합, 문체 통일 |

> 촬영은 **오전에 반드시 끝내세요.** 마지막 날로 미루면 장비 문제 시 대응 시간이 없습니다. OBS나 Windows 게임바(`Win+G`)로 화면 녹화. 실패 장면도 몇 개 남겨 보고서 "한계점"에 활용하세요.

### Day 9 (금) — 랩업 및 발표

| | 오전 | 오후 |
|---|---|---|
| 전원 | 코드 주석 정리, README 작성, 최종 리허설 2회 | **최종 발표 + 보고서 제출** |

> **발표 백업:** 라이브 데모는 실패할 수 있다고 가정하고, Day 8에 찍은 영상을 슬라이드에 삽입해 두세요.

---

## 7. 실험 계획 (보고서 핵심)

측정 코드는 Day 3에 미리 심어두고 Day 7에 데이터만 수집합니다.

### 실험 1. 커스텀 모델 성능 평가 ★신규
- **측정:** 전체 mAP50 / mAP50-95, 클래스별 정밀도·재현율, 혼동 행렬
- **방법:** 학습 시 분리해 둔 테스트셋(전체의 10%)으로 `yolo detect val` 실행
- **결과물:** 학습 곡선, 혼동 행렬, PR 곡선, 클래스별 성능 표
- **분석 포인트:** 어떤 클래스가 서로 혼동되는가. PET↔일반(투명 플라스틱류), CAN↔PET(원기둥 형태)의 혼동이 예상됩니다

### 실험 2. 좌표 변환 정확도
- **측정:** 자로 잰 실제 위치 vs 계산된 월드 좌표의 오차(mm)
- **방법:** 작업영역 격자 9지점(3×3)에 물체를 놓고 각 5회, 총 45회
- **비교축:** 선형 보간 vs 호모그래피
- **결과물:** 지점별 오차 히트맵, 평균/최대 오차

### 실험 3. 파이프라인 지연시간 분해
- **측정:** 캡처 / 추론 / 좌표변환 / IK / 실행 각 단계 소요 시간(ms)
- **방법:** 30회 사이클의 단계별 타임스탬프 로깅
- **결과물:** 누적 막대그래프로 병목 시각화
- **부가 비교:** `imgsz=320` vs `imgsz=640`의 FPS·정확도 트레이드오프

### 실험 4. 고전 CV 대비 성능 (여유 시) ★신규
- **측정:** HSV 색상+형태 기반 분류기 vs 커스텀 YOLO의 정확도·속도
- **의의:** "딥러닝이 왜 필요한가"를 데이터로 보여주는 대조군. Day 4에서 이미 색상 기반 검출기를 만들어 두므로 추가 구현 비용이 거의 없습니다
- **결과물:** 정확도·FPS·CPU 점유율 비교표

### 실험 5. 종단간 분류 성공률
- **측정:** 클래스별 성공률, 실패 사유 분포
- **방법:** 클래스 3종 × 각 20회 = 60회
- **결과물:** 실패 사유별 파이차트 (`not_detected`, `misclassified`, `ik_failed`, `grasp_lost` 등)

---

## 8. 개발 환경 및 저장소 구조

### 8.1 설치

```bash
python -m pip install pybullet opencv-python numpy matplotlib ultralytics
```

> Python **3.11 권장**. 3.13에서는 PyBullet 휠이 없어 소스 빌드로 넘어가 실패할 수 있습니다.

### 8.2 폴더 구조

```
robotwin-sorter/
├── README.md
├── requirements.txt
├── config.py                  # 좌표계·수거함 위치·임계값
├── common/
│   ├── schema.py              # Detection, SortTask ★공용 계약
│   └── logger.py              # CSV 로깅            [선우]
├── vision/                    # [주연]
│   ├── camera.py
│   ├── detector.py
│   └── stabilizer.py
├── integration/               # [선우]
│   ├── calibration.py
│   ├── spawner.py
│   └── pipeline.py            # 메인 진입점
├── robot/                     # [태익]
│   ├── scene.py
│   ├── arm_controller.py
│   └── fsm.py
├── tools/                     # [주연]
│   ├── capture.py             # 데이터 수집 스크립트
│   └── train_colab.ipynb      # 학습 노트북
├── dataset/                   # ★ Git LFS 또는 .gitignore 처리
│   ├── data.yaml
│   ├── images/{train,val,test}/
│   └── labels/{train,val,test}/
├── models/
│   └── best.pt                # 학습된 가중치
├── tests/
│   ├── dummy_vision.py
│   ├── dummy_robot.py
│   └── test_calibration.py
├── data/logs/                 # 실험 CSV
└── docs/
    ├── 기획서.md
    ├── labeling_rules.md      # 라벨링 기준 결정 기록
    └── 일지.md
```

> **`dataset/` 폴더는 용량이 크므로 Git에 그대로 올리지 마세요.** `.gitignore`에 추가하고 Google Drive나 Roboflow에 보관한 뒤, README에 다운로드 링크를 적어두는 방식을 권합니다. `models/best.pt`는 6MB 정도라 커밋해도 무방합니다.

### 8.3 Git 규칙

- 브랜치: `main` / `feat/vision` / `feat/robot` / `feat/integration`
- **Day 4 통합 시점에 전원 `main`으로 머지**
- 커밋 메시지: `[vision] 커스텀 모델 추론 파이프라인 구현`
- `common/schema.py`, `config.py`, `dataset/data.yaml`은 **합의 없이 수정 금지**

---

## 9. 회의 및 소통 규칙

| 시점 | 내용 | 소요 |
|---|---|---|
| 매일 09:00 | 스탠드업 — 어제/오늘/막힌 것 | 15분 |
| 매일 17:30 | 커밋 + `docs/일지.md` 1~2줄 기록 | 30분 |
| Day 4 / Day 6 | 체크포인트 회의 — 스코프 조정 판단 | 30분 |

- **30분 규칙:** 혼자 30분 이상 막히면 팀에 공유하세요. 9일 프로젝트에서 반나절 삽질은 전체의 5%입니다.
- 인터페이스 결정은 말로 하지 말고 **문서/코드에 반영 후 커밋**하세요.

---

## 10. 리스크 및 대응

| # | 리스크 | 가능성 | 대응 (Plan B) |
|---|---|---|---|
| 1 | 노트북에서 PyBullet import 차단 (애플리케이션 제어 정책) | 중 | 이미 1회 발생 이력. numpy/opencv는 정상. 메인 개발기 1대 지정. 최악의 경우 2링크 평면 로봇팔(순수 Python IK)로 대체 |
| 2 | `ultralytics` 설치 실패 (2GB+, 정책 차단) | 중 | **학습은 Colab에서 하므로 로컬엔 추론만 필요.** 그래도 막히면 ONNX 내보내기 후 `onnxruntime`으로 추론(용량 훨씬 작음). 최후엔 HSV 색상 기반 분류로 대체하고 "딥러닝 대비 한계"를 보고서 주제로 전환 |
| 3 | **투명 페트병 인식률이 낮음** | **높음** | 3.1절의 어두운 배경이 1차 대응. 그래도 낮으면 라벨 붙은 페트병 위주로 데이터 보강. 보고서에는 "투명 객체 인식의 근본적 어려움"으로 서술 (좋은 고찰 소재) |
| 4 | **캔 반사광으로 형태가 뭉개짐** | 중 | 직사광·스팟 조명 피하고 확산광 사용. 어두운 배경 병행 |
| 5 | mAP50이 0.70 미만 | 중 | **하이퍼파라미터보다 데이터 추가가 효과적.** Day 6 오전을 이미지 100장 보강+재학습에 배정 (일정에 이미 반영됨) |
| 6 | 라벨링 기준이 사람마다 달라 성능 저하 | 중 | `docs/labeling_rules.md`에 결정 즉시 기록. 애매한 건 그 자리에서 합의 |
| 7 | 로컬 CPU 학습이 근무시간을 잡아먹음 | 중 | Colab 우선. 로컬 학습 시 점심·퇴근 후 실행 |
| 8 | 그리퍼 파지 시 물체 미끄러짐 | 높음 | `p.createConstraint()` 방식 우선 (4.1 Step 3) |
| 9 | Day 4 통합 실패 | 중 | Day 4를 색상 기반 최소 연결로 설계한 이유. 실패 시 Day 5 오전 재배정, 모델 연동 하루 순연 |
| 10 | 웹캠 위치가 틀어짐 | 중 | 테이프로 완전 고정. **틀어지면 학습 데이터와 환경이 달라져 인식률까지 떨어집니다.** 캘리브레이션 재실행을 1분 내 가능하도록 스크립트화 |
| 11 | 발표 당일 라이브 데모 실패 | 중 | Day 8 촬영 영상을 슬라이드에 미리 삽입 |

---

## 11. 체크포인트 요약

| 시점 | 판단 기준 | 미달 시 조치 |
|---|---|---|
| **Day 1 종료** | 이미지 300장 수집 + 라벨링 80% + 전원 환경 구축 | 다음 날 오전을 라벨링에 추가 배정 |
| **Day 2 종료** | 학습 완료, mAP50 확인 | 0.70 미만이면 Day 6 재학습 계획 확정 |
| **Day 3 종료** | 3개 모듈 단독 동작 | 다음 날 오전을 보충에 사용 |
| **Day 4 종료** | 색상 기반 최소 파이프라인 1회 성공 | 주말 전 원인 회의, Day 5 오전 재배정 |
| **Day 6 종료** | 필수 성공 기준(10회 중 7회) 달성 | 다중 물체 포기, 단일 물체 안정화로 축소 |
| **Day 7 종료** | 실험 1·2·3 데이터 확보 | 실험 4·5 포기, 확보분으로 보고서 작성 |
| **Day 8 오전** | 데모 영상 촬영 완료 | 최우선 처리. 이것만은 미루지 않음 |

---

## 12. 부록 — 자주 겪는 문제

**Q. 학습은 잘 됐는데 웹캠 실시간에서는 인식이 안 됩니다.**
가장 흔한 원인은 촬영 환경 불일치입니다. 학습 때와 조명·배경·카메라 위치·해상도가 같은지 확인하세요. 웹캠이 움직였다면 데이터를 다시 찍어야 할 수도 있습니다.

**Q. 모든 물체를 한 클래스로만 예측합니다.**
클래스 불균형(한 클래스 이미지가 압도적으로 많음)이거나 `data.yaml`의 클래스 순서가 라벨과 어긋난 경우입니다. `confusion_matrix.png`를 먼저 확인하세요.

**Q. IK는 계산되는데 팔이 목표에 도달하지 않습니다.**
`force`가 부족하거나 `stepSimulation()` 호출이 모자란 경우입니다. 도달 판정 루프에서 매 반복 `stepSimulation()`을 호출하는지 확인하세요.

**Q. 스폰한 물체가 바닥을 뚫고 떨어집니다.**
`baseCollisionShapeIndex`를 지정하지 않았거나 `basePosition`의 Z가 물체 절반 높이보다 낮습니다.

**Q. 웹캠 좌표는 맞는데 로봇이 엉뚱한 곳으로 갑니다.**
Y축 부호를 확인하세요. 이미지 y는 아래로 증가하지만 월드 Y는 좌측으로 증가합니다. 호모그래피 대응점을 제대로 잡았다면 자동 처리되지만, 선형 변환을 직접 짰다면 부호가 뒤집히기 쉽습니다.

**Q. 같은 물체를 로봇이 반복해서 집으려 합니다.**
중복 검출 방지 로직(4.2 Step 3)이 빠졌습니다. 처리 중인 좌표를 집합으로 관리하세요.

**Q. `p.GUI` 창이 안 뜹니다.**
`p.DIRECT`로 바꾸면 창 없이 계산만 수행되며, `p.getCameraImage()`로 이미지를 뽑아 확인할 수 있습니다.
