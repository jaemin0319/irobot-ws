# 모델 변환 기록 (공식 URDF → MuJoCo)

절차: `scripts/20_expand_xacro.sh` (ROS Jazzy xacro, 작업공간 내 최소 ament prefix) → `models/urdf/fr3_hand.{urdf,srdf}`
→ `scripts/30_convert.py` (venv) → `models/fr3_hand_scene.xml` + `models/meshes/{visual,collision}` → `scripts/40_inspect_model.py` → `logs/model_check.json`

xacro 인자: `hand:=true ee_id:=franka_hand with_sc:=false` (나머지 기본값). 전개 결과 `fr3_hand_tcp_joint` z=0.1034 확인.

## 원본 대비 수정 사항
| 항목 | 내용 | 근거 |
|---|---|---|
| 보조 링크 제거 | `base`, `*_accelerometer_*` (질량 없는 고정 링크), `fr3_hand_tcp` 링크 → `tcp` site로 대체(hand +z 0.1034) | 동역학 영향 없음, TCP는 독립 FK로 확인 |
| 손가락 | prismatic → fixed, 각 0.04 m 개방(왼쪽 +y, 오른쪽 −y; 원본 joint 원점 0.0584 유지). 질량·관성은 원본 그대로 | 사용자 결정(손가락 고정) |
| visual | DAE 9개 → 재질별 OBJ 37조각(장면 변환 적용, 단위 m 확인: STL 경계와 일치). 모든 조각을 geom으로 참조, `contype=0 conaffinity=0 group=2 mass=0` | MuJoCo는 DAE 미지원 |
| collision | 공식 STL 9개 + 손가락 box 8개(원본). `contype=conaffinity=1 group=3` | |
| 충돌 제외 | SRDF `disable_collisions` 41쌍 전부 `<exclude>`로 반영(link8 관련 쌍은 geom 없음). 추가: world(바닥)↔fr3_link0 (베이스 설치 접촉, 허용) | SRDF 실제 내용 |
| 바닥 | z=0 plane, link0 원점 z=0 (원본 위치 그대로, 이동 없음) | |
| 액추에이터 | joint1–7 `motor`, gear 1, ctrlrange ±effort (87/12 N·m) | joint_limits.yaml |
| armature | motor_inertia(kg·m², 모터측) × gear_ratio² = 0.606/0.606/0.462/0.462/0.206/0.206/0.206 | dynamics.yaml에서 **유도한 값(공식 관절값 아님)** |
| damping | 0.1 (제안값, 공식값 아님) | |
| frictionloss | 0.2 — URDF `<dynamics friction="0.2">`를 MuJoCo 임포터가 그대로 반영 | 원본 |
| 옵션 | timestep 0.002, implicitfast, compiler fusestatic=false, balanceinertia=false | |

관성 검사: 질량·COM·관성텐서(링크 좌표계) 모두 URDF와 일치(상대오차 < 3e-6). 메시 추정값으로 바뀌지 않음.
