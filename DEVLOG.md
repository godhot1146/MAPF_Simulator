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

- [x] 에이전트 수 확장 → v2에서 99대로 확장
- [ ] ECBS (Enhanced CBS) — 충돌 우선순위로 탐색 효율 향상
- [ ] 시나리오 파일 (YAML) 로드/저장 — 에이전트 시작·목표 지점 포함
- [ ] 통계 그래프 — makespan, SOC(sum of costs) 시각화
- [ ] 다양한 이동 모델 — 대각 이동, 회전 반경 고려

---

## 2026-05-14 (v2 개발)

### 개요
v1 기반에서 확장성·연속 운용 기능 추가. 에이전트 99대, Auto Mode (A\* / CBS), 랜덤 배치 등.

---

### 구현 내용

#### 1. 에이전트 확장 (최대 99대)

- 색상: 8색 고정 팔레트 → 황금비 HSV 동적 생성 (`agent_color(idx)`)
  - `h = (idx * 0.618...) % 1.0` → 어떤 인덱스도 구별 가능
- 사이드바 에이전트 목록: 스크롤 지원, 선택 에이전트 자동 포커스

#### 2. Random Agents 버튼

- 개수 입력 다이얼로그
- `_spread_sample`: 모든 선택 위치가 `2 × radius + 0.5` 이상 떨어지도록 그리디 샘플링
  - `random.sample` 방식으로 start/goal 간 완전한 중복 없음 보장
- `compute_safe_positions` 기반 — 장애물 클리어런스 + 에이전트 간 거리 동시 고려

#### 3. Auto Mode — 연속 운용

두 가지 리플랜 방식을 선택 가능 (`Replan: A* [c]` / `Replan: CBS [c]` 버튼 토글):

**Auto A\* 모드**
- 누군가 도착 → 해당 로봇에게만 새 목표 배정
- **전체 로봇 A\* 재계획** (최신 상황 반영)
- 다른 로봇들은 기존 목표 유지, 경로만 갱신
- 소프트 제약: 다른 로봇의 미래 위치 `× 2*radius` 이내 셀 전부 차단

**Auto CBS 모드**
- 전원 도착까지 대기 → 도착한 로봇들만 새 목표 배정 → CBS 1회 실행
- CBS 실행 중 로봇들은 목표 지점에서 정지 대기
- CBS 결과 적용 시 로봇이 정지 상태이므로 순간이동 없음
- CBS 진행 상황도 동일하게 시각화 (경로 후보선 + 충돌 마커)

#### 4. New Map 다이얼로그

- 실행 중 cols × rows × cell 크기를 변경 가능
- CLI: `python main.py --cols N --rows N --cell N`

#### 5. 사이드바 제어 추가

| 컨트롤 | 기능 |
|---|---|
| Timeout `[-][+]` | 5–300초 조절 |
| Max nodes `[-][+]` | 500–100,000 조절 |
| `a` 키 | Auto Mode 토글 |
| `c` 키 | Replan 방식 토글 (A\* ↔ CBS) |

---

### 주요 문제 및 해결

| 문제 | 원인 | 해결 |
|---|---|---|
| Auto Mode 0 agents | `_reset_sim()`이 `_cont_mode=True` 이전에 호출되어 `_cont_agents` 초기화 | `_cont_mode=True`를 `_reset_sim()` 앞으로 이동 |
| CBS Auto에서 순간이동 | `local_t=0` 리셋 시 스냅샷 위치로 점프 | 로봇 정지 후 CBS 적용으로 구조 변경 (순간이동 원천 차단) |
| Random 배치 겹침 | `random.sample` 사용해도 start/goal 풀이 달라서 가능 | 하나의 풀에서 2n개 뽑기, 앞 n=start, 뒤 n=goal |
| Auto A\*에서 로봇 겹침 | 소프트 제약이 셀 중심만 피함 | `2×radius` 이내 모든 셀을 vertex constraint로 추가 |
| 도착 시 타 로봇도 목표 변경 | `_cont_replan_agent`가 목표 배정+경로 계획 혼재 | 목표 배정 분리, `_cont_replan_agent`는 현재 목표로 경로만 재계획 |
| 레이아웃 겹침 | Timeout/MaxNodes UI 높이 미확보 | `_rebuild_btns` 여백 재조정 |

---

### 알게 된 것 (v2)

**개별 A\* 리플랜의 근본 한계**
- 재계획 시점의 스냅샷만 보기 때문에 이후 타 로봇이 재계획하면 즉시 무효화
- 재계획 시점과 이동 중의 상황 변화는 반영 불가 → 충돌 100% 방지 불가
- 완전한 충돌 방지는 중앙화된 CBS만 보장

**CBS 연속 운용의 트레이드오프**
- CBS는 정적 시나리오 전용 설계 → 연속 운용에 쓰려면 "전원 도착 후 재계획" 라운드 구조 필요
- 실시간 CBS는 계산 시간 동안 로봇 이동으로 스냅샷이 오래됨 → 전원 정지 후 계획이 현실적
- 실제 웨어하우스 시스템(Amazon Kiva 등)은 CBS 대신 RHCR·ORCA 계열 사용

**빠름 vs 안전 트레이드오프 정리**

| 모드 | 충돌 보장 | 속도 | 적합 용도 |
|---|---|---|---|
| SOLVE CBS | 수학적 보장 | 느림 | 정적 시나리오 검증 |
| Auto CBS | 라운드 단위 보장 | 중간 | 정확한 연속 시뮬레이션 |
| Auto A\* | 보장 안 됨 | 빠름 | 빠른 데모·대규모 테스트 |

---

### 다음 과제 (아이디어)

- [ ] RHCR (Rolling Horizon Collision Resolution) — 주기적 윈도우 기반 CBS
- [ ] ORCA / 속도 장애물 기반 실시간 충돌 회피
- [ ] 통계 패널 — makespan, SOC, 충돌 횟수, 도착 횟수/초
- [ ] 시나리오 저장/불러오기 (에이전트 위치 포함 YAML)
- [ ] ECBS (Enhanced CBS) — 충돌 우선순위 기반 탐색 효율 향상
