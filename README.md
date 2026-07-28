# RoboTwin-Sorter

커스텀 YOLO & 디지털 트윈 기반 분리수거 자동 분류 시스템

사무실에서 나오는 실제 폐기물을 직접 촬영·라벨링해 자체 데이터셋을 구축하고,
학습시킨 YOLOv8 모델로 웹캠 영상에서 PET·캔·일반쓰레기를 판별한 뒤,
PyBullet 가상 환경의 로봇팔이 해당 분리수거함으로 자동 투입합니다.

| | |
|---|---|
| 기간 | 2주 단기 인턴 (실개발 9일) |
| 팀 | 김태익(로봇 제어) · 윤주연(AI 비전) · 진선우(디지털 트윈 통합) |
| 예산 | 0원 (개인 노트북 + 웹캠 + 사무실 폐기물) |

---

## 빠른 시작

```bash
git clone <저장소 주소>
cd robotwin-sorter

python -m venv .venv
.venv\Scripts\activate          # Windows
# source .venv/bin/activate     # macOS / Linux

pip install -r requirements.txt

# 설치 확인 (Day 1 DoD)
python -c "import pybullet; print('pybullet OK')"
python -c "import cv2; print('opencv OK')"
```

Python **3.11 권장**. 3.13에서는 PyBullet 휠이 없어 소스 빌드로 넘어가 실패할 수 있습니다.

### 실행

```bash
python -m robot.scene          # 씬만 띄워 확인      [태익]
python -m tools.capture        # 학습 데이터 수집    [주연]
python -m tests.test_calibration  # 좌표 변환 검증   [선우]
python -m integration.pipeline    # ★ 전체 시스템
```

---

## 데이터 흐름

```
[웹캠] --BGR 640x480--> vision/detector.py    [주연]
                            |
                     list[Detection]  (픽셀 좌표)
                            v
                     integration/       [선우]
                     calibration.py -> spawner.py
                            |
                        SortTask  (월드 좌표, 미터)
                            v
                     robot/arm_controller.py   [태익]
                            |
                          bool
                            v
                     common/logger.py -> CSV   [선우]
```

---

## 폴더 구조

```
robotwin-sorter/
├── config.py                ★ 전역 설정 (좌표·임계값·수거함 위치)  [공용]
├── common/
│   ├── schema.py            ★ Detection / SortTask 데이터 계약     [공용]
│   └── logger.py            CSV 로깅                              [선우]
├── vision/                                                        [주연]
│   ├── camera.py            웹캠 입력
│   ├── detector.py          YOLO 추론
│   └── stabilizer.py        좌표 안정화 · 정지 판정
├── integration/                                                   [선우]
│   ├── calibration.py       픽셀 -> 월드 (호모그래피)
│   ├── spawner.py           PyBullet 객체 스폰
│   └── pipeline.py          ★ 메인 진입점
├── robot/                                                         [태익]
│   ├── scene.py             씬 구성 (바닥·로봇·수거함)
│   ├── arm_controller.py    IK 이동 · 제약 기반 파지
│   └── fsm.py               9개 상태 기계
├── tools/                                                         [주연]
│   └── capture.py           데이터 수집 스크립트
├── tests/
│   ├── dummy_vision.py      가짜 Detection 생성 (Day 2 최우선)
│   ├── dummy_robot.py       항상 성공하는 가짜 로봇
│   └── test_calibration.py  좌표 변환 검증 · 실험 2
├── dataset/                 ★ .gitignore 처리 (Drive/Roboflow 공유)
├── models/best.pt           학습된 가중치 (~6MB, 커밋 허용)
├── data/logs/               실험 CSV
└── docs/
    ├── labeling_rules.md    라벨링 기준 결정 기록
    └── 일지.md              매일 17:30 기록
```

> `dataset/` 은 커밋되지 않습니다. 팀원 간 공유는 Google Drive나 Roboflow로 하고
> 링크를 아래에 적어두세요.
>
> - 데이터셋 다운로드: _(링크 채우기)_
> - Colab 학습 노트북: _(링크 채우기)_

---

## 협업 규칙

### 브랜치

| 브랜치 | 담당 | 작업 영역 |
|---|---|---|
| `main` | 공용 | Day 4 통합 시점에 전원 머지 |
| `feat/robot` | 태익 | `robot/` |
| `feat/vision` | 주연 | `vision/`, `tools/` |
| `feat/integration` | 선우 | `integration/`, `common/logger.py` |

```bash
git checkout -b feat/robot        # 최초 1회
git add robot/
git commit -m "[robot] IK 이동 구현"
git push -u origin feat/robot     # 첫 push만 -u
```

커밋 메시지 형식: `[robot] 내용` / `[vision] 내용` / `[integration] 내용`

### 합의 없이 수정 금지

- `common/schema.py`
- `config.py`
- `dataset/data.yaml`

이 세 파일은 세 사람의 계약서입니다. 바꿔야 한다면 스탠드업에서 합의하고,
고친 즉시 커밋 + 공지하세요. **말로만 합의하고 코드를 안 고치면 반드시 사고가 납니다.**

### 회의

| 시점 | 내용 | 소요 |
|---|---|---|
| 매일 09:00 | 스탠드업 — 어제 / 오늘 / 막힌 것 | 15분 |
| 매일 17:30 | 커밋 + `docs/일지.md` 기록 | 30분 |
| Day 4 / Day 6 | 체크포인트 — 스코프 조정 판단 | 30분 |

**30분 규칙:** 혼자 30분 이상 막히면 팀에 공유하세요. 9일 프로젝트에서 반나절 삽질은 전체의 5%입니다.

---

## 클래스 정의

| ID | 클래스 | 대상 | 수거함 | 좌표 |
|---|---|---|---|---|
| 0 | `pet` | 투명 페트병, 플라스틱 음료병 | `blue_bin` | (0.45, 0.45, 0) |
| 1 | `can` | 알루미늄 캔, 통조림캔 | `green_bin` | (0.00, 0.60, 0) |
| 2 | `general` | 종이컵, 비닐, 티슈, 일회용 수저 | `gray_bin` | (-0.45, 0.45, 0) |

**클래스는 3개로 고정합니다.** 늘리면 필요한 이미지 수가 비례해 늘어 9일 일정에서 라벨링에 잡아먹힙니다.

---

## 좌표계

**이미지 좌표계 (비전)** — 640×480, 원점 좌상단, x 우측(+), y **하단(+)**, 단위 픽셀

**월드 좌표계 (통합·로봇)** — 원점은 로봇 베이스, +X 정면 / +Y 좌측 / +Z 위, 단위 **미터**, 테이블 상면 Z=0

작업 영역: X `0.35~0.65`, Y `-0.25~0.25`, Z `0.02`

> ★ 이미지 y는 아래로 증가하고 월드 Y는 왼쪽으로 증가해 **부호가 반대**입니다.
> 호모그래피가 자동 처리하지만, 직접 선형식을 짜면 여기서 반드시 틀립니다.
> "로봇이 엉뚱한 방향으로 간다"의 90%는 이 문제입니다.

---

## 성공 기준

| 등급 | 기준 |
|---|---|
| 필수 | 검증셋 mAP50 ≥ 0.70 / 종단간 분류 10회 중 7회 성공 (Day 7까지) |
| 목표 | 3개 클래스 모두 실환경 인식 / 다중 물체 순차 분류 |
| 도전 | 고전 CV 비교 실험 / 물체 겹침·조명 변화 대응 |

---

## 자주 겪는 문제

**학습은 잘 됐는데 웹캠 실시간에서 인식이 안 됩니다**
촬영 환경 불일치입니다. 조명·배경·카메라 위치·해상도가 학습 때와 같은지 확인하세요.

**모든 물체를 한 클래스로만 예측합니다**
클래스 불균형이거나 `data.yaml`의 순서가 라벨과 어긋난 경우. `confusion_matrix.png`부터 보세요.

**IK는 계산되는데 팔이 목표에 도달하지 않습니다**
도달 판정 루프에서 매 반복 `p.stepSimulation()`을 호출하는지 확인하세요. 아니면 `JOINT_FORCE` 부족.

**스폰한 물체가 바닥을 뚫고 떨어집니다**
`baseCollisionShapeIndex`를 안 넣었거나 `basePosition`의 Z가 물체 절반 높이보다 낮습니다.

**웹캠 좌표는 맞는데 로봇이 엉뚱한 곳으로 갑니다**
Y축 부호. 위 좌표계 절 참고.

**같은 물체를 로봇이 반복해서 집으려 합니다**
중복 검출 방지 로직이 빠졌습니다. 처리 중인 좌표를 집합으로 관리하세요.

**`p.GUI` 창이 안 뜹니다**
`config.USE_GUI = False`로 두면 창 없이 계산만 하고, `p.getCameraImage()`로 확인할 수 있습니다.
