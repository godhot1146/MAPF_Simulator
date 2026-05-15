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

- [x] RHCR (Rolling Horizon Collision Resolution) → v3에서 구현
- [ ] ORCA / 속도 장애물 기반 실시간 충돌 회피
- [ ] 통계 패널 — makespan, SOC, 충돌 횟수, 도착 횟수/초
- [ ] 시나리오 저장/불러오기 (에이전트 위치 포함 YAML)
- [ ] ECBS (Enhanced CBS) — 충돌 우선순위 기반 탐색 효율 향상

---

## 2026-05-15 (v3 개발 — RHCR)

### 개요

RHCR(Rolling Horizon Collision Resolution) 구현 및 Auto Mode 버튼 구조 개편.
CBS 내부 버그 수정, A\* 부분 경로(partial path) 지원 추가.

---

### 구현 내용

#### 1. RHCR (Rolling Horizon Collision Resolution)

**개념**

| 파라미터 | 의미 |
|---|---|
| W (window) | CBS가 충돌을 감지하는 시간 범위 (스텝 수) |
| H (horizon) | 실제 실행 후 재계획하는 주기 (H ≤ W) |

동작 흐름:
1. CBS 실행 — 전체 경로 계획, 단 충돌 감지는 W 스텝 내로 제한
2. H 스텝 실행 (로봇 이동)
3. H 스텝 완료 → CBS 재발동 (현재 위치에서)
4. CBS 계산 중 로봇 동결 (순간이동 방지)
5. CBS 결과 적용 → 반복

**핵심 설계 결정**

- A\*는 최종 목적지까지 전체 경로 탐색 (`max_time=300` 유지)
- CBS 충돌 감지만 W 스텝으로 제한 → 탐색 공간 유지, CBS fail 방지
- W 스텝 클리핑 ❌ (→ 중간 웨이포인트에 로봇 집중 → t=0 충돌 → no_solution 유발)
- 이전 CBS가 보장한 위치에서 재출발하므로 다음 CBS도 해가 존재함이 이론적으로 보장

**구현 파일**

- `astar.py`: `partial_path` 파라미터 추가 — W 내 목적지 미도달 시 최근접 위치까지 경로 반환
- `cbs.py`: `horizon` 파라미터 추가 — `detect_first_conflict`를 W 스텝으로 제한
- `simulator.py`: `_rhcr_window` (W 기본값 12), `_rhcr_h` (H 기본값 5) 추가

#### 2. Auto Mode 버튼 구조 개편

기존: `[Auto Mode]` + `[Replan: A*/CBS/RHCR]` 2개 버튼 (토글 방식)

개편: 직관적인 3개 독립 버튼

| 버튼 | 키 | 동작 |
|---|---|---|
| `Auto: A*` | `a` | 개별 A\* 재계획 모드 |
| `Auto: CBS` | `b` | CBS 라운드트립 모드 |
| `Auto: RHCR` | `h` | 윈도우 기반 CBS 재계획 |

- 같은 버튼 재클릭 → 정지
- 다른 버튼 클릭 → 모드 전환 후 재시작

#### 3. 버그 수정

| 버그 | 원인 | 해결 |
|---|---|---|
| CBS replan fail 빈번 | 수동 SOLVE 취소 후 `stop_event`가 set 상태로 남음 → Auto CBS가 즉시 종료 | `_cont_replan_cbs_all()` 진입 시 `stop_event.clear()` 추가 |
| 홀딩 로봇 통과 | A\* 폴백에서 frozen 에이전트를 `t+1` 한 스텝만 제약 | frozen 에이전트를 `horizon`(30) 스텝 전체에 정적 장애물로 처리 |
| RHCR 스터터 (뒤로 튀기) | CBS 결과 적용 시 `local_t=0` 리셋이 소수점 위치를 정수로 스냅 | CBS 발동 직전 `local_t = int(local_t)`로 스냅해 점프 제거 |

#### 4. RHCR 파라미터 조작

| 키 | 동작 |
|---|---|
| `[` | W 감소 (−5, 최소 H) |
| `]` | W 증가 (+5, 최대 200) |
| `,` | H 감소 (−1, 최소 1) |
| `.` | H 증가 (+1, 최대 W) |

하단 stat bar에 `RHCR W=12 H=5 nxt:3.2steps` 실시간 표시.

---

### 알게 된 것 (v3)

**RHCR의 핵심은 W와 H의 분리**
- W = 계획 지평선 (CBS가 앞을 내다보는 범위)
- H = 실행 지평선 (재계획 주기, H ≤ W)
- W > H이면 "멀리 보고, 자주 갱신" → 데드락 예방 + 반응성 확보

**경로 클리핑의 함정**
- 직관적으로 "W 스텝만큼 잘라서 쓰면 된다"고 생각하기 쉬움
- 실제로는 클리핑된 경로 끝점에 로봇이 집중 → 다음 CBS에서 t=0 충돌 → no_solution
- 올바른 방법: A\*는 전체 탐색, 충돌 감지만 W 스텝 제한

**CBS 실패 원인 분류**

| 메시지 | 원인 | 대응 |
|---|---|---|
| `node_limit` | max_nodes 부족 | Max nodes 증가 |
| `no_solution` | 맵 토폴로지/에이전트 밀도 문제 | 에이전트 수 줄이기 또는 큰 맵 사용 |
| `cancelled` | stop_event 잔류 | (버그 수정됨) |

**RHCR 적용 한계 (33x33 벤치마크 기준)**
- 에이전트 20대 + radius 0.5: edge constraint 버그 수정 전까지 no_solution 빈발 → 수정 후 크게 개선됨
- 권장: 65x65 맵 또는 에이전트 수 조정

---

### 치명적 버그 수정 — CBS edge constraint off-by-1

**처음 커밋부터 존재한 버그.** swap 충돌 방지 명령이 실제로 A\*에 전혀 적용되지 않았음.

#### 원인

CBS가 t=5에서 A↔B 자리 바꾸기(swap) 충돌을 감지하면, A에게 제약을 줌:

```
CBS가 추가한 제약:  (c1,r1 → c2,r2, time=5)   ← 도착 시점
A* 가 확인하는 것:  (c1,r1 → c2,r2, time=4)   ← 출발 시점
```

`5 ≠ 4` → 제약이 무시됨 → A\*가 똑같은 충돌 경로 반환 → CBS 중복 감지 → 자식 노드 0개 → **`no_solution nodes=1`**

#### 수정 (`cbs.py`)

```python
dep_t = conflict.time - 1   # 출발 시점 = 도착 시점 - 1
child.constraints[agent]["e"].add((c1, r1, c2, r2, dep_t))
```

#### 영향

- swap 충돌이 정상적으로 해결됨
- Auto RHCR · Auto CBS 안정성 대폭 향상
- `no_solution nodes=1` 거의 사라짐

---

### 다음 과제 (아이디어)

- [ ] ORCA / 속도 장애물 기반 실시간 충돌 회피 (비교 대상으로)
- [ ] RHCR 성능 지표 — makespan, SOC, throughput 비교 그래프
- [ ] ECBS (Enhanced CBS) — 충돌 우선순위 기반 탐색 효율 향상
- [ ] 시나리오 저장/불러오기 (에이전트 위치 포함 YAML)

---

### 경로 시각화 개선 — 알고리즘 보장 구간 표시

#### 개념

CBS 계획 경로와 단순 A\* 경로를 시각적으로 구분:

| 모드 | 굵은 선 | 얇은 선 |
|---|---|---|
| Auto: A\* | 없음 | 전체 경로 |
| Auto: CBS | 전체 경로 | 없음 |
| Auto: RHCR | CBS 보장 구간 (W 스텝) | 나머지 전체 |
| SOLVE CBS | 전체 경로 | 없음 |

#### RHCR 굵은 선 동작

```
CBS 계획 시점: path[0] ~ path[W] = 굵은 선 (고정)
               path[W] ~ 끝      = 얇은 선

로봇 step=0:  굵은 선 path[0]~path[12] (W=12칸)
로봇 step=3:  굵은 선 path[3]~path[12] (앞쪽 고정, 뒤에서 소비)
로봇 step=5:  CBS 재계획 → 굵은 선 새로 path[0]~path[12]
```

- **앞쪽 끝점은 고정** (CBS 보장 범위 끝) — H 스텝마다만 갱신
- 로봇이 통과하면서 뒤에서 소비되는 방식
- W 칸 이후는 얇은 선으로 방향 힌트만 제공

#### 의미

굵은 선 = "CBS가 이 구간에서 충돌 없음을 수학적으로 보장한 경로"  
얇은 선 = "A\*가 목적지 방향으로 예측한 경로 (보장 없음, 다음 CBS에서 바뀔 수 있음)"
