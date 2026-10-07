# 현재 상태

- 기록 날짜: 2026-10-07
- 관련 코드: 커밋 `b5a3fdf`(첫 커밋)의 `irobot_sim/`. 검증 결과는 이 코드와 같은 코드로 얻음(RESULTS.md 머리말 참조)
- 현재 단계: **SIM 데모 구현 완료. GitHub 저장소용 문서·재현 절차 정리 완료**

## 완료한 일
- 공식 franka_description 2.9.0 확보·xacro 전개·MuJoCo 변환·모델 검사(14항목 통과)
- 상태 기계(HOME_CHECK → GEN_TARGET → REACH → REACH_SETTLE → RECOVER → RECOVER_SETTLE → 반복), 계산 토크 제어, IK, 경로 검사, 실패 정지, 로그 보존 정책
- 고정 목표 1회(headless·GUI), GUI 랜덤 3분 38사이클, Ctrl+C 종료 확인
- Git 추적 파일만으로 모델 확보·변환·검사·고정 목표 재현 확인(기존 venv 공유)

## 실행
```bash
./setup.sh                # 처음 한 번
./run.sh                  # 랜덤 무한 반복
./run.sh --mode fixed     # 고정 목표 검증
```

## 알려진 문제·미검증
- 창 닫기 종료: 구현했지만 실제 조작 미검증
- GUI 종료 시 GLFW/GL 정리 단계 segfault(exit 139)를 `os._exit`로 우회함. 근본 원인은 미해결
- 실패 경로(timeout, 목표 고갈, NaN, 외부 reset·perturbation) 실제 유발 시험 미실행
- 화면 오버레이 상태 문구가 보이지 않음(터미널·로그로 확인)
- 장시간 실행(시뮬 30분 이상) 미실행
- armature·damping·게인은 제안값이며 실제 로봇 동특성과 비교하지 않음
- 이 PC의 `/` 여유 공간이 작음(측정값은 RESULTS.md)

## 다음 할 일
1. 창 닫기 종료 확인
2. GL 종료 우회(`os._exit`) 점검: 원인 확인 또는 대안
3. 실패 경로·외부 조작 검증(각 원인을 독립적으로 유발)
4. 디스크·로그 관리 확인(`logs/` 예산, disk_low)
5. 장시간 검증
