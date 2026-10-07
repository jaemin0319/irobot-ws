# 결과 (2026-10-07)

> 검증 시점 코드와 현재 커밋 코드의 관계: 아래 검증은 Git 도입 전(2026-10-07 15:12–15:18)에 실행했습니다. `irobot_sim/*.py`의 마지막 수정은 15:14:07이고 GUI 랜덤 실행은 15:14:21에 시작했으므로, 첫 커밋의 시뮬레이션 코드와 같습니다. 이후 커밋에서는 문서, 준비 스크립트(`setup.sh`, `scripts/00`·`10`), `30_convert.py`의 `.tmp` 생성 한 줄만 추가했습니다. 변환 결과는 바이트 단위로 같음을 확인했습니다(아래 §7).

## 요약 표
| 항목 | 설정·조건 | 결과 | 근거 |
|---|---|---|---|
| 모델 검사 | 14항목 | 전부 통과 | `docs/evidence/model_check.json` |
| 고정 목표 headless | (0.45, 0.20, 0.35) m | 1.16 mm / 0.136°, home 0.00166 rad, exit 0 | 기록 미보존(동일 수치를 §7 재현에서 다시 얻음) |
| 고정 목표 GUI | 같음 | 같은 수치, exit 0 | `docs/evidence/run_20261007_151407.log` |
| GUI 랜덤 | seed 20261007, 관측 180 s(하네스 SIGINT), sim 178 s | 38사이클, 실패 0 | `docs/evidence/run_20261007_151421.log`, `gui_random_summary.json` |
| Ctrl+C | 랜덤 실행 15 s 후 SIGINT | exit 0, 약 0.06 s | 기록 미보존(터미널 측정) |
| 창 닫기 | — | 미검증 | — |
| 실패 주입·외부 조작·장시간 | — | 미검증 | — |

완료 기준(사용자 지정): 고정 목표 1회 + GUI 랜덤 반복 3분 + 기본 종료 확인. 장시간 검증·실패 주입 테스트는 생략.

## 1. 완료 여부
| 항목 | 결과 |
|---|---|
| 구현 | 완료 |
| 모델 검사 (`docs/evidence/model_check.json`) | 성공 — 14개 항목 전부 통과 |
| 고정 목표 (headless + GUI) | 성공 |
| GUI 랜덤 반복 3분 | 성공 — 38사이클 완료, 실패 0 |
| Ctrl+C(SIGINT) 종료 | 성공 — exit 0, 약 0.06 s |
| 창 닫기 종료 | **미검증** (xdotool/wmctrl 없음. 코드 경로는 있음) |
| 실패 주입·장시간 검증 | 생략 (사용자 지시) |

## 2. 환경
- 작업공간 `~/irobot_ws` (실제 디렉터리), 사용 262 MB
- Python 3.12.3 venv, mujoco 3.15.0, numpy 2.5.3, trimesh 5.1.1, pycollada 0.9.3, PyYAML (requirements.txt)
- 모델: franka_description 2.9.0 / 7aeeddc4
- `/` 여유(이 PC, 2026-10-07 측정): 시작 15:05 1086 MB → 15:18 625 MB → 15:40 약 830 MB. 작업공간은 262 MB이며 나머지 변동 원인은 미확인. 새 사용자의 최소 요구사양이 아님

## 3. 측정 (GUI 랜덤, seed 20261007, 관측 180 s 후 하네스가 SIGINT)
- 완료 사이클 38, 목표 생성 39(마지막은 종료로 중단), 실패 0
- reach 위치 오차 최대 1.56 mm, 방향 오차 최대 0.145°, 관절속도 최대 0.00075 rad/s
- recovery home 관절 오차 최대 0.0020 rad, 관절속도 최대 0.00075 rad/s
- 토크 포화 0 step
- 후보 62개 중 39개 채택(63%). 거부 원인 전부 `ik_not_converged`(23). 목표 생성 최대 0.13 s
- 채택 목표는 유효성 필터 때문에 원래 상자의 균일 분포와 다름
- 고정 목표: reach 1.16 mm / 0.14°, home 0.0017 rad
- 요약: `docs/evidence/gui_random_summary.json`, 원본: `docs/evidence/run_20261007_151421.log`

## 4. 순서 증거 (docs/evidence/run_20261007_151421.log)
```
15:14:27,553 RECOVERY_CONFIRMED cycle 1  max_q_err 0.00166
15:14:27,572 TARGET_GENERATED   cycle 2  target [0.573, -0.102, 0.322]
15:14:32,178 RECOVERY_CONFIRMED cycle 2  max_q_err 0.00182
15:14:32,201 TARGET_GENERATED   cycle 3  target [0.381, 0.232, 0.246]
```
`tests/check_events.py`로 전 구간 검사: order_ok=true. 모든 사이클이 TARGET_GENERATED → REACH_CONFIRMED → RECOVERY_STARTED → RECOVERY_CONFIRMED 순서.
실제 제어 recovery: live qpos는 시작 시 1회만 설정(`run.py` START 이벤트). 이후 매 스텝 `external_change()`가 qpos/qvel이 직전 mj_step 결과와 다르면 FAILED 처리. recovery는 측정된 q·q̇에서 quintic 궤적을 새로 만들고 경로를 다시 검사한 뒤 토크로 추종(RECOVER_TRAJ 이벤트).

## 5. 계획 대비 변경
- 게인 Kp/Kd 100/20 → 400/40: 원본 URDF의 frictionloss 0.2 때문에 home 오차가 0.0074 rad(기준 0.01 rad에 근접) → 변경 후 0.0017 rad. 판정 기준은 그대로.
- recovery 경로: 실제 상태에서 재계획·재검사(요청 A). 경로 샘플 간격 ≤ 0.01 rad(최소 50점), 1 cm 여유 마진. 유한 샘플 검사라 연속 경로 안전을 보장하지 않음.
- 종료 시 GLFW/GL 정리 단계 segfault(exit 139) → 자원 정리 후 `os._exit(code)`로 우회.
- 화면 오버레이(`set_texts`)는 화면에서 보이지 않음 → 상태는 터미널·로그로 확인.
- 계획의 ~/plan.md 교체·메모리 변경은 요청 E에 따라 이번에는 하지 않음.

## 6. 남은 제한
- 창 닫기 종료, 실패 주입(timeout/고갈/NaN/외부 reset) 미검증
- 뷰어 GUI reset·슬라이더·perturbation 감지는 코드상 구현(qpos/qvel shadow 비교, xfrc_applied 검사). 실제 GUI 조작으로는 미확인
- armature·damping·게인은 제안값
- `/` 여유 공간이 적음(625 MB). 런타임 200 MB 미만이면 disk_low로 중단

## 7. GitHub 정리 시 추가 확인 (2026-10-07 15:3x)
- Git 추적 파일만 clone한 사본(`.tmp/repro`, venv는 기존 것 공유)에서 `10_fetch_model.sh` → `30_convert.py` → `40_inspect_model.py`(ALL OK) → `./run.sh --mode fixed --headless --exit-on-fail`(1.16 mm, home 0.00166 rad, exit 0)을 실행했습니다. 생성된 `fr3_hand_scene.xml`과 `models/meshes/`는 원래 것과 바이트 단위로 같았습니다.
- 새 venv 설치(`00_setup_venv.sh`)와 다른 PC에서의 전체 재현은 하지 않았습니다.
- 참고: 15:20–15:23에 추가 실행 로그 5개(`logs/run_20261007_1520*`, `1522*`, `1523*`)가 있습니다. 모두 SIGINT로 exit 0 종료했고 FAILED는 없습니다(최장: 랜덤 26사이클, sim 118 s). 이 실행들은 GUI 여부 등을 확인하지 못했고 Git에는 넣지 않았습니다.
