# MAPF Simulator (CBS)

Multi-Agent Path Finding 시뮬레이터.  
알고리즘: **Conflict-Based Search (CBS)**  
맵 포맷: **PGM + YAML** (ROS map_server 표준 호환)

---

## 실행

```bash
pip install -r requirements.txt
python main.py
```

예시 맵 생성 (최초 1회):

```bash
python make_maps.py
```

---

## 프로젝트 구조

```
MAPF_concept_develop/
├── main.py           실행 진입점
├── grid.py           Grid 클래스 + PGM/YAML 로드·저장
├── astar.py          Space-time A* (CBS 로우레벨 플래너)
├── cbs.py            CBS 알고리즘 (하이레벨 플래너)
├── simulator.py      Pygame UI (에디터 + 시뮬레이터 통합)
├── make_maps.py      예시 맵 생성 스크립트
├── requirements.txt
└── maps/
    ├── empty_40x30.pgm / .yaml     빈 40×30 맵
    ├── warehouse_40x30.pgm / .yaml 창고형 맵
    └── random_32x24.pgm / .yaml    랜덤 장애물 맵
```

---

## PGM + YAML 맵 포맷

```yaml
# example.yaml
image: example.pgm      # PGM 파일명 (같은 디렉토리)
resolution: 0.05        # m/pixel
origin: [0.0, 0.0, 0.0]
occupied_thresh: 0.65   # 점유 확률 임계값
free_thresh: 0.196      # 자유 확률 임계값
negate: 0
```

PGM 규칙:  
- 흰색(255) = 자유 공간  
- 검은색(0)  = 장애물

---

## 조작 방법

### 모드 전환

| 키 | 기능 |
|---|---|
| `1` | 장애물 편집 모드 |
| `2` | 에이전트 편집 모드 |
| `Esc` | 현재 서브 동작 취소 |

### 장애물 편집 (모드 `1`)

| 조작 | 기능 |
|---|---|
| 좌클릭 / 드래그 | 장애물 배치 |
| 우클릭 / 드래그 | 장애물 제거 |

### 에이전트 편집 (모드 `2`)

| 조작 | 기능 |
|---|---|
| 사이드바 `+Agent` | 에이전트 추가 (최대 8개) |
| 사이드바 `-Agent` | 선택된 에이전트 삭제 |
| `s` 또는 `SetStart` 버튼 후 그리드 클릭 | 시작 지점 설정 |
| `g` 또는 `SetGoal` 버튼 후 그리드 클릭 | 목표 지점 설정 |
| `F1` ~ `F8` | 에이전트 선택 |
| 그리드에서 에이전트 위치 클릭 | 해당 에이전트 선택 |

### 시뮬레이션

| 키 / 버튼 | 기능 |
|---|---|
| `Space` / `SOLVE` | CBS 풀기 (백그라운드 스레드) |
| `r` / `Reset Sim` | 시뮬레이션 초기화 |
| `+` / `-` | 재생 속도 조절 (0.5x ~ 20x) |

### 화면 이동 / 줌

| 조작 | 기능 |
|---|---|
| 마우스 휠 | 줌 인/아웃 (셀 크기 8 ~ 40 px) |
| 중간 버튼(휠 클릭) 드래그 | 맵 패닝 |

### 파일

| 버튼 | 기능 |
|---|---|
| `Save Map` | 현재 맵을 PGM+YAML로 저장 |
| `Load Map` | PGM+YAML 맵 불러오기 |
| `Clear Agents` | 에이전트만 초기화 |
| `Clear All` | 맵 + 에이전트 전체 초기화 |

---

## 시각 표현

| 요소 | 표현 |
|---|---|
| 에이전트 현재 위치 | 색상 원 |
| 에이전트 목표 지점 | 동색 빈 사각형 |
| 경로 | 동색 얇은 선 |
| 선택된 에이전트 | 흰색 테두리 |

에이전트마다 고유 색상 (빨강, 초록, 파랑, 노랑, 보라, 청록, 주황, 연두 순).
