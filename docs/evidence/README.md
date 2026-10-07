# 검증 근거 파일

`logs/`(Git 제외)에 있던 원본을 그대로 복사했습니다(수정 없음, 2026-10-07).

| 파일 | 원본 | 내용 |
|---|---|---|
| `model_check.json` | `logs/model_check.json` | `scripts/40_inspect_model.py` 결과(14항목) |
| `run_20261007_151407.log` | 같은 이름 | 고정 목표 GUI 1사이클 전체 이벤트 로그 |
| `run_20261007_151421.log` | 같은 이름 | GUI 랜덤 3분(seed 20261007) 전체 이벤트 로그(64 KB) |
| `gui_random_summary.json` | `logs/gui_random_summary.json` | 위 로그를 `tests/check_events.py`로 요약·순서 검사한 결과 |

요약을 다시 만드는 명령: `./run_py.sh tests/check_events.py docs/evidence/run_20261007_151421.log`
