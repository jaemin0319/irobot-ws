# irobot_ws — MuJoCo FR3 랜덤 reach → home recovery

MuJoCo에서 Franka FR3 + Franka Hand(손가락 고정)가 매 사이클 다음 순서로 움직이는 시뮬레이션입니다.

**새 유효 랜덤 목표 생성 → reach → 상태 기반 도달 확인 → 실제 제어로 home recovery → 상태 기반 복귀 확인 → 다음 목표 생성**

- 기본 실행(`./run.sh`)은 사용자가 끌 때까지 **횟수 제한 없이** 반복합니다.
- **home**: 공식 SRDF `ready` 관절 자세 `[0, −π/4, 0, −3π/4, 0, π/2, π/4]` rad (joint1–7).
- **recovery**: 측정한 현재 q·q̇에서 home까지 관절 궤적(quintic)을 새로 만들고, 경로를 다시 검사한 뒤 토크 제어(계산 토크 PD + bias)로 추종합니다. 복귀 판정은 실제 관절 오차·속도로 합니다.
- 시작할 때 한 번만 live 상태를 `qpos=home, qvel=0`으로 둡니다(시작 자세 지정). 이후에는 qpos 대입·reset·keyframe 복원으로 움직이지 않습니다. 외부에서 qpos/qvel이 바뀌면 실패로 처리합니다.
- 다음 목표는 home 복귀가 확인된 직후에만 생성합니다.
- 어느 단계든 실패하면 자동 반복을 멈추고(물리 정지, 창 유지) 원인을 `logs/failure_*.json`과 로그에 기록합니다.

## 검증 환경
| 항목 | 값 |
|---|---|
| OS | Ubuntu 24.04.5 LTS, x86_64 |
| Python | 3.12.3 (시스템 python3 + 작업공간 venv) |
| MuJoCo | 3.15.0 (그 외 버전은 `requirements.txt`) |
| GUI | X11(`DISPLAY=:0`), NVIDIA RTX 4060 / 드라이버 580. GUI 없이 `--headless`도 가능 |
| 모델 | 공식 [franka_description](https://github.com/frankarobotics/franka_description) 2.9.0 @ `7aeeddc4` (Apache-2.0) |

다른 환경에서의 전체 재현(새 venv 설치 포함)은 아직 확인하지 않았습니다. 확인한 범위는 [docs/RESULTS.md](docs/RESULTS.md)에 있습니다.

## 처음 준비하기
필요한 것: `python3`(3.10 이상)와 `python3-venv`, `git`, 인터넷(PyPI, GitHub). GUI는 OpenGL과 GLFW가 동작하는 X11 데스크톱이 필요합니다.
ROS는 **필요 없습니다**. xacro 전개 결과(`models/urdf/fr3_hand.{urdf,srdf}`)를 저장소에 넣어 두었습니다.

```bash
# 작업공간이 아직 없을 때만 clone 합니다 (기존 ~/irobot_ws를 덮어쓰지 않음)
[ -e ~/irobot_ws ] && echo "~/irobot_ws 가 이미 있습니다 — 그 폴더를 사용하세요" || git clone https://github.com/jaemin0319/irobot-ws.git ~/irobot_ws
cd ~/irobot_ws
./setup.sh      # venv 생성·설치 → 공식 모델 확보 → MuJoCo 모델 변환 → 모델 검사(ALL OK)
```
`setup.sh`가 하는 일(단계별로 따로 실행할 수도 있습니다):

| 단계 | 명령 | 결과 |
|---|---|---|
| 1 | `bash scripts/00_setup_venv.sh` | `.venv/`에 고정 버전 의존성 설치(pip 캐시 미사용) |
| 2 | `bash scripts/10_fetch_model.sh` | `third_party/franka_description/` (2.9.0 sparse checkout, 커밋 검증) |
| (선택) | `bash scripts/20_expand_xacro.sh` | xacro로 URDF/SRDF 재생성. ROS 2 Jazzy `xacro` 필요 |
| 3 | `./run_py.sh scripts/30_convert.py` | `models/meshes/`(DAE→OBJ 37조각, STL), `models/fr3_hand_scene.xml` |
| 4 | `./run_py.sh scripts/40_inspect_model.py` | `logs/model_check.json` (14개 검사) |

필요 공간: venv 약 220 MB, 모델 원본 34 MB, 변환 결과 11 MB로 측정했습니다(2026-10-07).
코드는 작업공간 기준 상대 경로만 씁니다. `run.sh`와 `run_py.sh`는 셸의 `PYTHONPATH`·`LD_LIBRARY_PATH`(예: ROS)를 지우고 venv만 사용합니다.

## 실행·종료
```bash
cd ~/irobot_ws
./run.sh                          # 랜덤 reach → home 무한 반복 (GUI)
./run.sh --seed 42                # seed 고정: 같은 목표 순서 재현 (RNG는 시작 시 1회 생성)
./run.sh --mode fixed             # 고정 목표 1사이클 검증 후 종료(exit 0)
./run.sh --mode view              # home 확인 후 home 유지
./run.sh --headless               # viewer 없이 실행 (옵션 조합 가능)
./run.sh --exit-on-fail           # 실패 시 창을 유지하지 않고 바로 종료
./run_py.sh tests/check_events.py logs/run_<시각>.log   # 로그 요약 + 이벤트 순서 검사
```
- **Ctrl+C / SIGTERM**: 정리 후 exit 0으로 끝납니다. **확인함**(약 0.06 s).
- **창 닫기**: 같은 정리 경로로 exit 0이 되도록 구현했지만 **실제 조작으로는 아직 확인하지 않았습니다**.
- **실패 후**: 창을 닫거나 Ctrl+C를 누르면 exit 1로 끝납니다.

## 좌표계·목표·판정 (설정: [irobot_sim/config.py](irobot_sim/config.py))
- 좌표계: world = FR3 base(link0) 좌표계. 바닥 z=0입니다. 기준점은 `tcp` site(`fr3_hand_tcp`, hand +z 0.1034 m)입니다.
- 목표: 위치 + 공구 z축이 world −z(아래)를 향하는 방향. yaw는 자유입니다.
  - 랜덤: x 0.30–0.60, y −0.30–0.30, z 0.15–0.55 m, 수평 반경 0.30 m 이상, 균일 샘플.
  - 고정 검증 목표: (0.45, 0.20, 0.35) m.
- 유효성 검사(reach 전): IK 수렴(1 mm, 1°), 관절 제한 여유 0.05 rad, 목표 자세 충돌 없음(1 cm 여유), 손목·손이 바닥에서 5 cm 이상, 경로 검사.
  - 경로 검사는 관절 변화 0.01 rad 간격, 최소 50점입니다. 유한 샘플 검사이므로 연속 경로의 안전을 보장하지는 않습니다.
  - 후보는 최대 50개 또는 3 s까지 봅니다. 넘으면 실패로 처리합니다.
- 성공 판정(아래 조건이 연속으로 유지되어야 함. 조건이 깨지면 유지 타이머를 다시 시작):

| 판정 | 조건 | 유지 시간 |
|---|---|---|
| HOME_CHECK | max\|q−home\| ≤ 0.01 rad, max\|q̇\| ≤ 0.02 rad/s | 0.5 s |
| REACH | 위치 ≤ 5 mm, 방향 ≤ 3°, max\|q̇\| ≤ 0.02 rad/s | 0.3 s |
| RECOVERY | max\|q−home\| ≤ 0.01 rad, max\|q̇\| ≤ 0.02 rad/s | 0.3 s |

- 시간 제한은 실패 감지에만 씁니다(HOME 5 s, 궤적 시간 + 5 s, 모두 시뮬레이션 시간).
- 실행 중 계속 감시하는 항목: NaN, MuJoCo 경고, 관절 제한, 예상하지 않은 접촉, 추종 오차 0.2 rad 초과, 외부 상태 변경(viewer reset·슬라이더·perturbation), 디스크 여유 200 MB 미만.
- 제어: 500 Hz(timestep 0.002 s)이고 실시간 1배속입니다. Kp 400, Kd 40이며 관절 최고 속도는 한계의 40% 이내입니다. 게인·armature·damping은 제안값입니다.

## 저장소 구조
| 경로 | 역할 |
|---|---|
| `run.sh`, `run_py.sh`, `setup.sh` | 실행 진입점, 격리된 Python 래퍼, 전체 준비 |
| `irobot_sim/config.py` | home·목표 범위·허용 오차·게인·시간 제한·로그 예산 |
| `irobot_sim/core.py` | 모델 로드, 제어기, quintic 궤적, IK, 목표·경로 유효성 검사 |
| `irobot_sim/run.py` | 상태 기계, viewer, 실패·종료 처리, 로그 |
| `scripts/` | 00 venv · 10 모델 확보 · 20 xacro(선택) · 30 변환 · 40 모델 검사 |
| `models/urdf/` | 공식 xacro 전개 결과(URDF/SRDF, Apache-2.0, 출처 고지 포함) |
| `tests/check_events.py` | 실행 로그 요약과 이벤트 순서 검사 |
| `docs/` | [RESULTS](docs/RESULTS.md) · [CURRENT_STATE](docs/CURRENT_STATE.md) · [MODEL_SOURCES](docs/MODEL_SOURCES.md) · [MODEL_CONVERSION](docs/MODEL_CONVERSION.md) · 계획·요청서 원문 |
| `docs/evidence/` | 검증 원본 로그와 요약(아래) |

Git에 넣지 않는 것: `.venv/`, `.tmp/`, `third_party/`(스크립트로 확보), `models/meshes/`와 `models/fr3_hand_scene.xml`(변환으로 생성), `logs/`(실행마다 생성).

## 현재 상태 요약 (2026-10-07)
| 항목 | 결과 |
|---|---|
| 모델 검사 14항목 | 통과 |
| 고정 목표 (headless, GUI) | 통과 — 1.16 mm / 0.14°, home 0.0017 rad |
| GUI 랜덤 3분 (seed 20261007) | 38사이클, 실패 0 — 최대 1.56 mm / 0.145°, home 0.0020 rad |
| Ctrl+C 종료 | 통과 (exit 0) |
| 창 닫기, 실패 주입, 외부 조작, 장시간 실행 | **미검증** |

자세한 수치와 근거는 [docs/RESULTS.md](docs/RESULTS.md), 알려진 문제와 다음 할 일은 [docs/CURRENT_STATE.md](docs/CURRENT_STATE.md)에 있습니다.

## 라이선스
- `models/urdf/`와 `scripts/10_fetch_model.sh`로 받는 모델 파일은 Franka Robotics GmbH의 franka_description(Apache-2.0)에서 온 것입니다. [models/urdf/NOTICE.md](models/urdf/NOTICE.md)를 보세요.
- 이 프로젝트 코드의 라이선스는 아직 정하지 않았습니다.
