# MAPF Simulator 개발 일지

---

## 2026-05-14

### 개요
CBS(Conflict-Based Search) 기반 MAPF 시뮬레이터를 처음부터 설계·구현.

---

### 구현 내용

#### 1. 핵심 알고리즘

**`astar.py` — Space-time A***
- 상태: `(col, row, time)`
- 제약 조건 처리: vertex constraint `(c, r, t)` / edge constraint `(c1, r1, c2, r2, t)`
- `safe_positions` 파라미터로 에이전트 반경에 따른 장애물 회피 지원
- 목표 지점 도달 후 미래 vertex constraint를 고려한 `goal_free_from` 처리

**`cbs.py` — Conflict-Based Search**
- 하이레벨: 충돌 트리 탐색 (min-heap, cost = sum of path lengths)
- 충돌 감지: vertex conflict (거리 기반) + edge conflict (위치 교환)
- **경로 조합 메모이제이션**: 동일한 경로 조합은 재탐색하지 않음 (중복 루프 방지)
- 종료 구분:
  - 오픈 리스트 소진 → `[PROVEN]` 해 없음 확정
  - 노드 한도 초과 → `[NODE LIMIT]`
  - 타임아웃 → `[TIMEOUT]`
  - 사용자 취소 → `Cancelled`
- `stop_event` (threading.Event)로 비동기 취소 지원
- `progress` dict로 실시간 탐색 상태 공유

**`grid.py` — Grid & Map I/O**
- PGM + YAML 포맷 (ROS map_server 표준 호환)
- P2/P5 PGM 파싱, `occupied_thresh` / `negate` 처리
- `compute_safe_positions(grid, agent_radius)`: 반경 내 장애물 없는 위치 사전 계산

---

#### 2. 시뮬레이터 UI (`simulator.py`)

**에디터**
- 장애물 모드: 좌클릭/드래그 배치, 우클릭 제거
- 에이전트 모드: 시작·목표 지점 클릭 배치, F1-F8 선택
- New Map 다이얼로그: 그리드 크기·셀 크기 실시간 변경

**뷰**
- 마우스 휠 줌 (8–40 px/cell), 중간 버튼 패닝
- 기본 셀 크기 8px (이전 구현 대비 소형화)

**물리 에이전트 반경**
- `agent_radius` (기본 1.5 cells) — 셀보다 큰 원형 몸통
- CBS 로우레벨: `safe_positions`로 장애물 클리어런스 강제
- CBS 충돌 감지: 두 에이전트 거리 < `2 × radius`이면 충돌
- `z`/`x` 키로 실시간 조절

**CBS 실시간 시각화**
- 탐색 중 현재 경로 후보를 흐릿하게 표시
- 충돌 위치: 빨간 맥동 원 + timestep `t=N` 표시
- 사이드바: nodes 수, open 리스트 크기, 경과 시간, 타임아웃 잔여

**솔버 제어**
- Timeout: 5–300초 (기본 30초), 사이드바 `[-][+]` 조절
- Max nodes: 500–100,000 (기본 5,000), 사이드바 `[-][+]` 조절
- SOLVE 중 Space/버튼 → CANCEL로 즉시 중단

---

#### 3. 맵

| 파일 | 크기 | 내용 |
|---|---|---|
| `empty_40x30` | 40×30 | 경계벽만 있는 빈 맵 |
| `warehouse_40x30` | 40×30 | 수직 선반 배치 |
| `warehouse_200x100` | 200×100 | 가로 선반 + 10셀 간격 수직 통로 |
| `random_32x24` | 32×24 | 랜덤 20% 장애물 |

---

### 주요 문제 및 해결

| 문제 | 원인 | 해결 |
|---|---|---|
| 에이전트 인덱스 불일치 | CBS 결과 키가 valid 리스트 기준이었으나 `self.agents` 기준으로 렌더링 | `_valid_indices` 저장 후 결과 리매핑 |
| CBS 먹통 (무한 대기) | 타임아웃·취소 없음 | `stop_event` + 자동 타임아웃 구현 |
| 같은 충돌 반복 (루프) | 중복 상태 감지 없음 | 경로 조합 해시 메모이제이션으로 재탐색 방지 |
| 사이드바 UI 겹침 | 레이아웃 높이 미확보 | `_rebuild_btns` 간격 재계산 |
| 충돌이 같은 위치처럼 보임 | 시각화가 (col,row)만 표시 | timestep `t=N` 추가 표시 |

---

### 알게 된 것

- CBS는 이론상 완전(complete)·최적(optimal)이지만 에이전트 수에 대해 지수 복잡도를 가짐.
- 에이전트 반경이 커지면 (1) safe_positions 감소, (2) 충돌 감지 거리 확대 → 좁은 복도에서 2대 이상 통과 불가능.
  - 통과 조건: 복도 너비 > `4 × radius`
- 같은 환경에서 반경만 줄이면 해가 생기는 이유: 반경이 작을수록 safe_positions가 넓어져 우회 경로가 생김.
- CBS 중복 루프의 근본 원인: 다른 제약 조건 집합이 동일한 경로 조합을 생성할 수 있음 → 경로 조합 레벨에서 메모이제이션해야 완전히 해결됨.

---

### 다음 과제 (아이디어)

- [ ] 에이전트 수 확장 (현재 최대 8개)
- [ ] ECBS (Enhanced CBS) — 충돌 우선순위로 탐색 효율 향상
- [ ] 시나리오 파일 (YAML) 로드/저장 — 에이전트 시작·목표 지점 포함
- [ ] 통계 그래프 — makespan, SOC(sum of costs) 시각화
- [ ] 다양한 이동 모델 — 대각 이동, 회전 반경 고려
