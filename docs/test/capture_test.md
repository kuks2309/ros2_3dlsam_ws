# 캡처 · 테스트 증거 수집

테스트 동작 검증·디버깅·산출물 보관 시 X11 창 단위 PNG 캡처 절차.

## 1. 도구

- **`capture-test` skill** — [~/.claude/skills/capture-test/SKILL.md](~/.claude/skills/capture-test/SKILL.md)
- Gazebo / RViz / Terminal 등 X11 창을 개별 PNG 으로 저장한다.

## 2. 두 가지 사용 모드

### 2.1 인터랙티브 (`/capture-test`)

- 사용자가 창 목록에서 직접 선택.
- 단발 검증 · 이슈 보고용.

### 2.2 자동 통합 (실험 스크립트 내부 헬퍼)

- 정해진 체크포인트마다 Gazebo 창 + RViz 창 개별 캡처.
- 참조 구현: [experiments/run_forward_align_test.sh](../../experiments/run_forward_align_test.sh) 의 `capture_step()` 함수.

## 3. 저장 위치 · 인용

- 결과 경로: `experiments/capture/YYYYMMDD_HHMMSS_<label>.png`
- 분석 / 이슈 보고 시 해당 경로를 인용한다.

## 4. 주의 사항

- 화면 잠금 상태에선 lock screen 만 캡처됨.
- 무인 장기 실험 시 screen lock 비활성화 필요.
