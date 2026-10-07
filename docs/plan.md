# irobot_ws 구현 계획 — MuJoCo FR3 랜덤 reach → home recovery 무한 반복

> 공개 저장소용으로 로컬 사용자명과 홈 절대 경로를 `<user>`, `~`로 가렸습니다. 그 밖의 내용은 원문 그대로입니다.

- 작성일: 2026-10-07 · 상태: **계획만 작성.** 구현·설치·다운로드·실행은 하지 않았습니다.
- 기준 요청서: `plan_request.md`
- 표기: **[확인]** 이번 조사에서 명령으로 직접 확인함 · **[제안]** 승인 전 권장안 · **[미확인]** 구현 단계에서 검증할 항목 · **[추정]** 계산 또는 경험에 근거한 추정치
- 승인 후 첫 작업: 기존 `~/plan.md`(27 KB, 10/7 13:25)를 `~/plan.md.bak-20261007`로 백업한 다음, 이 계획을 `~/plan.md`로 저장합니다(Work 검토용). 작업공간을 만든 뒤 `~/irobot_ws/docs/plan.md`에도 사본을 둡니다.

---

## 0. Context

MuJoCo viewer에서 Franka FR3 팔이 매 사이클 새 랜덤 목표로 reach합니다. 도달 여부는 실제 상태로 판정합니다. 그 뒤 **제어 입력과 물리 시뮬레이션만으로** 공식 home 관절 자세로 recovery합니다. 이 과정을 사용자가 종료할 때까지 반복합니다. 모델은 제조사 공식 URDF/Xacro와 메시에서 직접 확보합니다. 고정 목표 한 점으로 먼저 검증한 뒤에만 랜덤 반복으로 넘어갑니다.

### 사용자 결정 (이번 계획 중 확정)
| 항목 | 결정 |
|---|---|
| 작업공간 | **`~/irobot_ws`를 실제 디렉터리로 생성**합니다(심볼릭 링크 아님). `/` 여유 공간이 작으므로 §2의 용량 예산과 보호 장치를 반드시 적용합니다 |
| 기종 | **FR3 + Franka Hand, 손가락 고정** |
| 엔드이펙터 조건 | **위치 + 아래 방향.** TCP z축을 world −z에 맞추고, z축 둘레 yaw는 자유로 둡니다 |
| 계획 파일 | 기존 `~/plan.md`를 백업한 뒤 교체합니다 |
| 이전 결정(유지) | `/mnt/isaac/franka_mujoco`와 독립된 프로젝트로 진행합니다. 그 프로젝트의 코드·venv·모델·산출물을 복사하거나 import하지 않습니다 |

---

## 1. 실제 PC·작업공간 확인 결과

### 1.1 확인한 사실 [확인]
| 항목 | 결과 |
|---|---|
| 사용자·홈 | `<user>`(uid 1000, sudo 그룹), `~` = **`~`** (`drwxr-x---`, 소유자 <user>:<user>, 일반 디렉터리, 쓰기 가능) |
| `~/irobot_ws` | **존재하지 않습니다.** `~/*irobot*`, `/mnt/isaac/*irobot*`도 없습니다(이전 작업공간은 삭제됨). 부모 `~`은 사용자 소유이고 쓰기 가능하므로 생성할 수 있습니다 |
| 디스크 | `/`(홈 포함, nvme0n1p5 ext4): 48 G 중 **가용 1.1 G (98% 사용)**, inode 22% 사용. `/mnt/isaac`: 가용 25 G(이번에는 사용하지 않음). `~/.cache`가 1.1 G를 차지함 |
| OS | Ubuntu 24.04.5 LTS, x86_64, 커널 6.8.0-139 |
| CPU·RAM·GPU | 20코어, RAM 62 GiB(가용 약 28 GiB), RTX 4060 8 GB, 드라이버 580.178.04 |
| Python | `/usr/bin/python3` 3.12.3, pip 24.0(시스템), `python3-venv` 설치됨. 활성화된 venv·conda 없음(`~/.conda`, `~/.mamba` 디렉터리만 있음). `uv`·`conda`·`pipx`는 PATH에 없음 |
| MuJoCo | 시스템 Python에 **미설치**(ModuleNotFoundError). PyPI 최신 버전은 **3.15.0**(Python ≥3.10, cp312 manylinux x86_64 wheel 30.2 MB) |
| 디스플레이·GL | `DISPLAY=:0`, `XDG_SESSION_TYPE=x11`, `MUJOCO_GL` 미설정. `libglfw3` 3.3.10 설치됨 |
| ROS | ROS 2 Jazzy가 셸에 source되어 있음. `/opt/ros/jazzy/bin/xacro`(ros-jazzy-xacro 2.1.1) 있음 |
| 셸 오염 요소 | `PYTHONPATH`와 `LD_LIBRARY_PATH`에 `/opt/ros/jazzy/...`가 기본으로 들어 있음 → venv를 격리해야 함 |
| 네트워크 | GitHub(`git ls-remote`, API, raw)과 PyPI에 접근 가능 |

### 1.2 미확인 항목 [미확인]
- venv에 `mujoco` 3.15.0을 설치한 뒤 `mujoco.viewer.launch_passive` 창이 뜨는지(GLFW + NVIDIA GLX) — 실행하지 않았습니다.
- 시스템 xacro가 작업공간 내부 ament prefix로 `$(find franka_description)`을 해석하는지 — 실행하지 않았습니다.
- `fr3.urdf.xacro`의 `tcp_xyz` 기본값이 `""`일 때 hand 쪽 기본값 0.1034 m가 실제로 적용되는지 — 전개 결과로 확인해야 합니다.
- DAE를 OBJ로 변환한 결과의 크기와 재질 보존 여부, 장시간 실행 시 메모리 추이.

---

## 2. 작업공간: 용량 예산·구조·격리

### 2.1 용량 예산 [추정] — `/` 가용 1.1 G 기준
| 항목 | 추정 |
|---|---|
| venv(mujoco 3.15 + numpy + pyopengl·absl·etils·glfw·websockets + 변환용 trimesh·pycollada) | 약 250–300 MB |
| 공식 모델 원본(sparse·shallow·blobless 체크아웃, FR3 + hand 파일만. 전체 저장소는 198 MB라 받지 않음) | 약 35 MB |
| 변환 메시(OBJ/STL)와 MJCF | 약 30–60 MB |
| 로그(회전 상한) | 최대 25 MB |
| **합계** | **약 350–420 MB** → 작업 후 `/` 여유 공간 약 0.7 G가 남을 것으로 추정 |

보호 장치 [제안]:
- **사전 검사**: 단계 2를 시작할 때 `/` 가용 공간이 700 MB 미만이면 설치하지 않고 중단한 뒤 보고합니다.
- **설치 중**: `pip --no-cache-dir`를 쓰고 `TMPDIR`과 `PIP_CACHE_DIR`을 작업공간 안 `.tmp`로 지정합니다. 설치 직후 `.tmp`를 비웁니다.
- **런타임**: 60 s(실제 시간)마다 `shutil.disk_usage`로 확인합니다. 가용 공간이 200 MB 미만이면 **실패 중단**(원인: disk_low)으로 처리합니다. 루트 파일시스템이 가득 차서 OS에 영향을 주는 것을 막기 위함입니다.
- `~/.cache`(1.1 G) 등 작업공간 밖의 파일은 건드리지 않습니다. 공간이 부족하면 사용자에게 정리할지 묻습니다.

### 2.2 디렉터리 구조 [제안]
```
~/irobot_ws/
├── README.md                 # 실행·검증·종료 방법
├── docs/                     # plan.md 사본, MODEL_SOURCES.md, MODEL_CONVERSION.md, CURRENT_STATE.md
├── run.sh                    # 격리된 실행 래퍼
├── requirements.txt          # 버전 고정
├── .venv/                    # 전용 가상환경
├── .tmp/                     # TMPDIR / pip 임시 디렉터리 (설치 후 비움)
├── third_party/
│   ├── franka_description/   # 공식 저장소 2.9.0 sparse 체크아웃 (읽기 전용으로 취급)
│   └── ament_prefix/         # xacro $(find) 해석용 최소 ament index (심볼릭 링크 → 위 저장소)
├── models/
│   ├── urdf/fr3_hand.urdf    # xacro 전개 결과
│   ├── meshes/               # 변환한 visual OBJ + collision STL
│   ├── fr3_hand_raw.xml      # MuJoCo URDF 임포터 결과
│   ├── fr3_hand.xml          # 보완된 로봇 MJCF
│   └── scene.xml             # 바닥·조명·목표 마커 포함 최상위 장면
├── scripts/                  # 10_fetch_model.sh, 20_expand_xacro.sh, 30_convert.py, 40_inspect_model.py
├── irobot_sim/               # config.py, model.py, ik.py, trajectory.py, controller.py,
│                             # targets.py, checks.py, state_machine.py, logging_setup.py, run.py
├── tests/                    # 헤드리스 검증 스크립트
└── logs/                     # run_YYYYmmdd_HHMMSS.log(회전), model_check.json, failure_*.json
```

### 2.3 격리 규칙 [제안]
- 모든 Python 실행은 `run.sh`를 거칩니다: `env -u PYTHONPATH -u LD_LIBRARY_PATH PYTHONNOUSERSITE=1 .venv/bin/python -m irobot_sim.run "$@"`.
- xacro 전개만 ROS 환경에서 실행합니다(별도 셸 단계). 그 결과물인 URDF는 venv에서 읽습니다.
- **작업공간 밖에서 쓰는 항목**: 시스템 `/usr/bin/python3`(venv의 기반), `/opt/ros/jazzy/bin/xacro`, 시스템 GLFW·GL 드라이버. 모두 이미 설치되어 있으며 **변경하지 않습니다.** xacro를 쓰지 못하면 venv에 PyPI `xacro`를 설치하는 것이 대안입니다.

---

## 3. 공식 모델: 출처·확보·변환·검증

### 3.1 출처 [확인]
| 항목 | 내용 |
|---|---|
| 공식 저장소 | https://github.com/frankarobotics/franka_description (Franka Robotics GmbH) |
| 버전 고정 | 태그 **2.9.0** = 커밋 **`7aeeddc449edf8d62b594f9e36a81da53e7796f9`**(현재 HEAD와 동일) |
| 라이선스 | Apache-2.0 (`LICENSE`, `NOTICE`: Copyright 2023 Franka Robotics GmbH) → `docs/MODEL_SOURCES.md`에 기록하고 원본 LICENSE·NOTICE를 보존 |
| 필요 파일 | `robots/fr3/{fr3.urdf.xacro, joint_limits.yaml, inertials.yaml, dynamics.yaml, kinematics.yaml, fr3.srdf.xacro}`, `robots/common/*`, `end_effectors/{common,franka_hand}/*`, `meshes/robots/fr3/{visual/*.dae (약 26.7 MB), collision/*.stl}`, `meshes/robot_ee/franka_hand_white/*`, `package.xml`, `LICENSE`, `NOTICE` |
| 관절 제한(joint_limits.yaml) | j1 ±2.9007, j2 ±1.8361, j3 ±2.9007, j4 [−3.0770, −0.1169], j5 ±2.8763, j6 [0.4398, 4.6216], j7 ±3.0508 rad · 속도 2.62(j1–4) / 5.26, 4.18, 5.26 rad/s · 토크 87(j1–4) / 12(j5–7) N·m |
| home(SRDF `ready`) | `[0, −π/4, 0, −3π/4, 0, π/2, π/4]` rad (`robots/common/group_definition.xacro`) |
| TCP | `hand_tcp` = hand 프레임에서 +z 0.1034 m. hand는 flange에서 rpy (0, 0, −π/4) 회전 |
| 메시 형식 | visual은 **DAE**(MuJoCo가 직접 읽지 못함 → 변환 필요), collision은 STL |
| 참고(비공식) | DeepMind `mujoco_menagerie/franka_fr3`(HEAD `f054586`)는 팔 전용 MJCF입니다. **사용하지 않고**, 액추에이터·armature 값이 비슷한지 교차 확인할 때만 참고합니다 |

### 3.2 확보·변환 절차 [제안] (스크립트로 고정, `docs/MODEL_CONVERSION.md`에 기록)
1. **확보**(`10_fetch_model.sh`): `git clone --filter=blob:none --no-checkout --depth 1 --branch 2.9.0` 후 sparse-checkout으로 위 파일만 받습니다. 받은 뒤 `git rev-parse HEAD`가 `7aeeddc4…`와 같은지 확인하고, 파일 목록과 sha256을 `docs/MODEL_SOURCES.md`에 기록합니다.
2. **xacro 전개**(`20_expand_xacro.sh`, ROS 셸): `third_party/ament_prefix/share/ament_index/resource_index/packages/franka_description`(빈 파일)와 `share/franka_description → ../../franka_description` 링크를 만들고, `AMENT_PREFIX_PATH`를 앞에 붙인 상태로 다음을 실행합니다.
   `xacro fr3.urdf.xacro hand:=true ee_id:=franka_hand with_sc:=false > models/urdf/fr3_hand.urdf`
   결과에서 `hand_tcp` 원점이 0.1034인지 확인합니다.
3. **경로 처리**: `package://franka_description/meshes/...` → `meshes/...` 상대 경로로 바꾸고, `.dae` 참조는 변환한 `.obj`로 바꿉니다.
4. **메시 변환**(`30_convert.py`, venv의 trimesh + pycollada): DAE를 재질별 OBJ 조각으로 나누고 rgba를 보존합니다. 실패하면 대안으로 collision STL을 회색 visual로 씁니다.
5. **MJCF 생성**: URDF에 `<mujoco><compiler meshdir="../meshes" discardvisual="false" fusestatic="false" balanceinertia="false"/></mujoco>`를 넣어 `MjModel.from_xml_path`로 읽은 뒤 `mj_saveLastXML` → `fr3_hand_raw.xml`. `fusestatic="false"`를 써야 hand와 TCP 프레임이 남습니다.
6. **보완**(같은 스크립트로 `fr3_hand.xml` 생성. 수정 내역은 모두 문서화):
   - 손가락 prismatic 관절을 **제거하고 고정 body로 둡니다.** 개방 폭은 각 0.04 m(완전 개방)입니다. 0으로 두면 손가락 끝끼리 겹쳐 접촉으로 잡히므로 피합니다.
   - `site name="tcp"`를 hand body의 +z 0.1034 위치에 둡니다.
   - 관절 7개마다 `motor` 액추에이터를 둡니다(`ctrlrange`와 `forcerange`는 effort 87/12).
   - armature = motor_inertia × gear²를 [제안]으로 둡니다. 계산값은 j1–2 ≈ 0.606, j3–4 ≈ 0.462, j5–7 ≈ 0.206입니다. damping과 frictionloss는 dynamics.yaml 값을 그대로 쓰지 않고 작게 둡니다(damping 0.1). 그 근거와 값을 기록합니다.
   - 인접 링크 쌍에는 `<contact><exclude>`를 두고(SRDF `disable_collisions`와 대응), collision geom은 group 3, visual geom은 group 2로 나눕니다.
7. **장면**(`scene.xml`): 로봇을 include하고 바닥 plane(z=0, base 높이), 조명, 목표 마커를 둡니다. 마커는 mocap body의 반투명 구(r=1 cm)와 방향 표시 막대이며 `contype=0 conaffinity=0`입니다.

### 3.3 모델 검증 방법 (`40_inspect_model.py` → `logs/model_check.json`) [제안]
| 구분 | 검증 | 통과 기준 |
|---|---|---|
| 표시만 확인 | 장면 로드, `nq/nv/nu`, viewer에 메시 표시 | 오류 없이 로드. nq=nv=7, nu=7. 팔과 손이 사람이 보기에 정상 |
| 관절 제한 | `jnt_range`와 joint_limits.yaml 비교 | 7개 모두 1e-4 rad 이내로 일치, `limited=true` |
| 관성 | body 질량·관성과 inertials.yaml 비교 | 질량 상대 오차 1% 이내, 관성이 양정치 |
| 충돌 형상 | home 자세에서 `ncon`, 자기충돌·바닥 접촉 | 접촉 0 |
| 엔드이펙터 | home에서 TCP 위치·방향과 URDF 체인을 numpy로 독립 계산한 FK 결과 비교 | 위치 0.1 mm, 방향 0.01 rad 이내. home TCP는 약 (0.307, 0, 0.487) m로 [추정] |
| 액추에이터 | `actuator_ctrlrange`·`forcerange` | 87/12 N·m와 일치 |
| **제어 가능 상태** | 헤드리스로 중력 보상 + PD로 home을 10 s(시뮬 시간) 유지 | 최대 관절 오차 < 0.005 rad, 속도 < 0.01 rad/s, NaN 없음, 토크 포화 없음 |

---

## 4. 제어·상태 전이·목표 생성·판정

### 4.1 시간과 제어 [제안]
- 물리 `timestep` 0.002 s(500 Hz), 제어도 매 스텝 500 Hz, 적분기 `implicitfast`.
- viewer `sync()`는 약 60 Hz로 호출하고, 실제 시간에 맞춰 1배속으로 진행합니다.
- **제어기**: 관절 공간 계산 토크 형태의 PD + bias로 `τ = M(q)·(Kp·(q_d − q) + Kd·(q̇_d − q̇) + q̈_d) + qfrc_bias`를 계산하고 forcerange로 클립합니다. 기본 게인은 Kp=100 s⁻², Kd=20 s⁻¹(임계 감쇠)로 [제안]합니다.
- **목표 관절값 계산과 실제 이동은 분리합니다.** IK는 별도 scratch `MjData`에서만 계산합니다. live `MjData`의 qpos는 초기 로딩 때 한 번만 설정하고, 그 뒤로는 토크로만 움직입니다.
- **궤적**: 현재 q에서 목표 q까지 quintic(최소 jerk) 관절 보간을 씁니다. 시작과 끝의 속도·가속도는 0입니다. 소요 시간 T = max(2.0 s, max_i |Δq_i| / (0.4·v_max,i) × 1.875)로 두어 각 관절 최대 속도를 한계의 40% 이내로 제한합니다.
- reach와 recovery 모두 같은 궤적 생성기와 제어기를 씁니다. recovery 목표는 home 관절값입니다.

### 4.2 상태 기계
```
INIT_LOAD → HOME_CHECK → GEN_TARGET → REACH(궤적 추종) → REACH_SETTLE(판정)
     → RECOVER(궤적 추종) → RECOVER_SETTLE(판정) → GEN_TARGET → …   (횟수 제한 없음)
어느 단계든 실패 → FAILED(물리 정지, viewer 유지) · 종료 요청 → SHUTDOWN
```
- 초기 로딩 때 `qpos = home`, `qvel = 0`으로 설정하는 것은 **시작 자세 지정**일 뿐입니다. 이것은 문서와 로그에 따로 표시합니다. 이후에는 qpos 대입, reset, keyframe 복원을 **쓰지 않습니다**(코드 리뷰와 정적 검사로 확인).
- 다음 목표는 `RECOVER_SETTLE` 성공 직후에만 샘플링합니다. 미리 샘플링하거나 대기열에 쌓지 않습니다.
- 각 루프 반복과 각 단계 안에서 종료 플래그와 `viewer.is_running()`을 확인합니다.

### 4.3 랜덤 목표 생성과 유효성 [제안]
- **좌표계**: world = FR3 base(link0) 좌표계입니다. 바닥 z=0이고 x는 정면입니다.
- **기준점**: `tcp` site(hand_tcp)입니다.
- **분포**: 매번 독립적인 균일 분포로 x ∈ [0.30, 0.60], y ∈ [−0.30, 0.30], z ∈ [0.15, 0.55] m에서 뽑습니다. 단 반경 √(x²+y²) ≥ 0.30 m입니다. 방향은 TCP z축 = world −z이고 yaw는 IK에 맡깁니다.
- RNG는 `numpy.random.default_rng(seed)` **한 번만 생성**합니다. `--seed`를 주지 않으면 시간 기반 seed를 쓰고 로그에 기록합니다. 실행 중에 다시 seed를 설정하지 않습니다.
- **IK**: scratch 데이터에서 감쇠 최소제곱(λ=0.05)을 씁니다. 작업은 위치 3D + 방향 2D(z축 정렬)이고, null-space로 home 쪽을 향하게 합니다. 시작점은 home 1개와 home 근방 랜덤 2개, 최대 200 반복입니다.
- **후보 유효성 검사**(모두 통과해야 reach 시작):
  1. IK 수렴: 위치 오차 < 1 mm, 방향 오차 < 1°.
  2. 관절 제한 안쪽으로 0.05 rad 이상 여유.
  3. 목표 자세에서 자기충돌과 바닥 접촉 없음(`mj_forward` 후 `ncon`, 제외 쌍 반영).
  4. TCP z ≥ 0.10 m, 손 전체가 바닥에서 5 cm 이상 위.
  5. **경로 검사**: 현재 q(=home)에서 목표 q까지 quintic 경로를 50점 샘플링해 각 점에서 충돌과 제한을 검사. recovery 경로(역방향)도 같은 경로이므로 함께 통과로 봅니다.
  6. 직전 목표와의 거리 ≥ 5 cm(같은 점 반복 방지, 참고용).
- **재샘플링 상한**: 50 후보 또는 실제 시간 3 s. 넘으면 FAILED(원인: target_generation_exhausted). 거부 사유별 개수를 로그에 남깁니다.

### 4.4 성공 판정 (모두 상태 기반, 시간 경과만으로는 성공 처리하지 않음) [제안]
| 단계 | 조건 (모두 동시에 충족) | 안정화 유지 |
|---|---|---|
| HOME_CHECK | max_i |q_i − home_i| ≤ 0.01 rad, max |q̇| ≤ 0.02 rad/s, 접촉 0 | 0.5 s(시뮬) 연속 |
| REACH 성공 | TCP 위치 오차 ≤ 5 mm, 방향(z축) 오차 ≤ 3°, max |q̇| ≤ 0.02 rad/s, 관절 제한 안 | 0.3 s 연속 |
| RECOVER 성공 | max_i |q_i − home_i| ≤ 0.01 rad (≈0.57°), max |q̇| ≤ 0.02 rad/s | 0.3 s 연속 |

- 유지 중 조건이 한 번이라도 깨지면 유지 타이머를 0으로 돌립니다. 그래서 목표를 지나가는 순간은 성공으로 잡히지 않습니다.
- 판정은 궤적 시간 T가 끝난 뒤에만 시작합니다(추종 중 통과를 배제).

### 4.5 실패 감지·처리 [제안]
| 실패 | 기준 |
|---|---|
| 단계 시간 초과 (**실패 감지 전용**) | 시뮬 시간 기준: HOME_CHECK 5 s, REACH·RECOVER는 T + 5 s |
| 시뮬 정지 | 실제 시간 2 s 동안 `data.time`이 증가하지 않음 |
| 외부 reset 감지 | `data.time` 감소. viewer의 Backspace reset은 live 데이터에 씀 |
| 수치 이상 | qpos·qvel·qacc에 NaN/Inf, `mjWARN_BADQACC` 등 MuJoCo 경고 카운터 증가 |
| 제한 위반·충돌 | 관절 제한 초과, 추종 중 예상하지 않은 접촉 |
| 추종 오차 | 궤적 추종 오차 > 0.2 rad |
| 디스크 부족 | `/` 가용 < 200 MB |
| 목표 생성 실패 | §4.3 상한 초과 |

- 처리: **자동 반복을 즉시 중단**합니다. 다음 단계 진행, 자동 reset, 자동 home 복귀는 하지 않습니다. 물리 스텝을 멈추고 viewer는 열어 둔 채 화면 오버레이에 영문 요약을 표시합니다(MuJoCo 폰트는 한글을 렌더링하지 못함).
- 다음 내용을 터미널, 로그, `logs/failure_<시각>.json`에 남깁니다: 단계, 사이클 번호, 목표, 관절 상태 q·q̇, 위치·관절 오차, 시뮬·실제 경과 시간, **확인된 원인**(위반된 조건 값), **추정 원인**(예: IK 해 근처 특이점, 게인 부족).
- 사용자가 창을 닫거나 Ctrl+C를 누르면 정리 후 **exit 1**로 끝납니다. 정상 종료는 0입니다. `--exit-on-fail` 옵션을 주면 viewer를 바로 닫습니다(헤드리스 검증용).

### 4.6 종료 처리 [제안]
- `SIGINT`·`SIGTERM` 핸들러는 플래그만 세웁니다. 메인 루프가 매 스텝 플래그와 `viewer.is_running()`을 확인합니다.
- `try/finally`에서 `viewer.close()`, 로그 flush·close, 종료 요약(사이클 수, 마지막 단계)을 처리합니다. 오버레이 텍스트 갱신은 viewer lock 밖에서 합니다.
- 두 번째 Ctrl+C는 즉시 종료합니다(정리 실패 시 탈출구).

### 4.7 로그 [제안]
- `logging.handlers.RotatingFileHandler`(파일당 5 MB, 백업 4개, 최대 25 MB)를 씁니다.
- 사이클마다 한 줄씩 기록합니다: cycle, seed, target, 재샘플 수, IK 오차, T, reach 오차·소요 시간, recovery 오차·소요 시간.
- 스텝별 상세 데이터는 기본으로 끄고 `--debug`일 때만 1 Hz로 기록합니다.

---

## 5. 구현 순서·통과 기준 (각 단계를 통과해야 다음 단계 진행)

| # | 선행 조건 | 작업 | 확인할 관측값 | 통과 기준 | 실패 시 조치 |
|---|---|---|---|---|---|
| 1 | — | 환경 재점검(이 문서 §1을 다시 확인), `/` 가용 공간 확인 | `df`, python, xacro, DISPLAY | 가용 ≥ 700 MB, §1과 같음 | 공간 부족이면 중단하고 사용자에게 질문 |
| 2 | 1 | `~/irobot_ws` 생성, venv 생성, `pip --no-cache-dir install mujoco==3.15.0 numpy trimesh pycollada` → `requirements.txt` 고정 | `import mujoco; mujoco.__version__`, 설치 용량, `/` 가용 공간 | 3.15.0 import 성공, venv ≤ 350 MB | cp312 wheel 문제가 있으면 직전 마이너 버전을 쓰고 기록 |
| 3 | 2 | 모델 확보 → xacro 전개 → 메시 변환 → MJCF 보완 → `40_inspect_model.py` | §3.3 표 항목 | 표 전 항목 통과 | 불일치 항목을 수정하고 문서에 기록. 원본은 수정하지 않음 |
| 4 | 3 | `run.sh --mode view`: passive viewer, 로봇·마커 표시, HOME_CHECK | 창 표시, HOME_CHECK 로그 | 창이 뜸, HOME_CHECK 성공, 5 s 유지 | GL 문제면 `MUJOCO_GL=glfw` 명시 또는 드라이버 로그를 확인한 뒤 사용자에게 보고 |
| 5 | 4 | **고정 목표 1점** `run.sh --mode fixed`: reach → 판정 → 토크 제어 recovery → home 판정 → 종료(검증 모드만 1회) | TCP 오차, 관절 오차, 속도, 토크 포화, 경과 시간 | §4.4 기준 충족, qpos 대입 없음 | 게인과 T를 조정. 원인은 로그로 확인 |
| 6 | 5 통과 | 랜덤 목표 생성 + 무한 상태 전이 연결 `run.sh`(기본 모드) | 사이클 로그, 목표 분포, 재샘플 수 | 헤드리스 검증 세션(`tests/`에서 시뮬 30분 상당 또는 100 사이클 관측. **검증 하네스 쪽의 관측 한도**이며 프로그램 기본값이 아님)에서 실패 0, GUI로 10사이클 육안 확인 | 실패 로그로 원인 분석. 기준을 바꿀 때는 근거를 남김 |
| 7 | 6 | 실패 주입과 종료 검증 | 아래 §6 항목 | 모두 기대대로 동작 | 수정 후 다시 확인 |

고정 검증 목표 [제안]: TCP (0.45, 0.20, 0.35) m, 아래 방향. 단계 3의 IK와 경로 검사로 유효성을 먼저 확인합니다.

---

## 6. 실행·검증·종료 명령 (구현 후)

```bash
cd ~/irobot_ws
./run.sh --mode view                 # 모델·마커 표시 + HOME_CHECK
./run.sh --mode fixed                # 고정 목표 1회 검증
./run.sh                             # 최종: 랜덤 reach→home 무한 반복 (종료: 창 닫기 또는 Ctrl+C)
./run.sh --seed 42                   # 재현용 seed (RNG는 시작 시 1회만 생성)
./run.sh --headless --exit-on-fail   # 검증용 (viewer 없음)
.venv/bin/python tests/run_cycles.py --sim-minutes 30   # 검증 하네스 (관측 한도는 하네스에만)
.venv/bin/python tests/inject_failure.py --kind {timeout,ik_exhaust,nan,reset}
```

검증할 종료·실패 시나리오:
- 창 닫기와 Ctrl+C(REACH·RECOVER·GEN_TARGET 각 단계에서 시도) → 1 s 안에 종료, exit 0, 로그에 `shutdown reason`, 남은 프로세스 없음(`pgrep -f '[i]robot_sim'`).
- 실패 주입(게인 0으로 시간 초과, 목표 범위를 도달 불가로 설정, NaN 주입, viewer Backspace) → FAILED, 다음 목표를 생성하지 않음, failure json 생성, exit 1.
- 로그 회전: `--debug` 장시간 실행 시 `logs/` ≤ 25 MB.

확인할 로그 항목: `seed`, `cycle`, `phase`, `target_xyz`, `resample_count`·거부 사유, `ik_err`, `traj_T`, `reach_pos_err`·`ori_err`·`max_qd`, `recover_max_q_err`, `sim_t`·`wall_t`, `disk_free_mb`, `fail_reason(confirmed/suspected)`, `shutdown_reason`.

---

## 7. 제안 기본값 요약과 남은 불확실성

| 조건 | 제안 기본값 | 근거 |
|---|---|---|
| home | SRDF `ready` [0, −π/4, 0, −3π/4, 0, π/2, π/4] rad (내부 단위 rad, 로그에 deg 병기) | 제조사 정의 자세이며 모든 관절이 제한 안쪽에 충분한 여유를 둠 |
| TCP·좌표계 | `hand_tcp`, base 좌표계 | 공식 모델이 정의한 TCP |
| 목표 범위 | x 0.30–0.60, y ±0.30, z 0.15–0.55 m, 균일 분포 | FR3 도달 반경 855 mm 안쪽의 정면 작업 영역. 아래 방향 자세로 IK 성공률이 높을 것으로 [추정] |
| 허용 오차 | 위치 5 mm, 방향 3°, 관절 0.01 rad, 속도 0.02 rad/s, 유지 0.3 s(home 확인은 0.5 s) | PD + bias 제어의 정상상태 오차보다 충분히 크고, 시각적으로 "도달"로 보이는 수준 |
| 시간 제한 | HOME 5 s, REACH·RECOVER T + 5 s (시뮬 시간), 정지 감지 2 s (실제 시간) | 실패 감지 전용 |
| 속도·주기 | 관절 속도 한계의 40%, T ≥ 2 s, 500 Hz 물리·제어, 60 Hz 화면 | 급격한 움직임을 피하고 MuJoCo 안정성 확보 |

남은 불확실성:
- `/` 여유 공간(1.1 G)이 다른 프로그램 때문에 줄어들 수 있습니다. 런타임 디스크 검사로 감지만 할 수 있습니다.
- DAE 변환 품질과 크기는 미확인입니다(대안: collision STL을 visual로 사용).
- armature와 damping 값은 공식 dynamics.yaml을 해석한 값이므로 실제 로봇 동특성과 같다고 보장할 수 없습니다. 이 시뮬레이션의 목적은 안정적인 추종이며, 해석 근거를 문서로 남깁니다.
- 위 허용 오차와 범위는 단계 5–6 결과로 조정할 수 있습니다. 조정할 때는 근거와 함께 기록하고 사용자에게 보고합니다.
- viewer 마우스 perturbation(Ctrl+드래그)은 추종 오차나 시간 초과로만 간접 감지됩니다.

## 8. 승인 후 실행 순서 요약
1. `~/plan.md` → `~/plan.md.bak-20261007` 백업 후 이 계획을 `~/plan.md`로 저장합니다.
2. §5의 단계 1–7을 순서대로 진행합니다. 각 단계 결과는 `docs/CURRENT_STATE.md`에 기록합니다.
3. 메모리의 irobot_ws 항목을 "홈 실제 디렉터리, `/` 공간 예산 적용"으로 갱신합니다.
