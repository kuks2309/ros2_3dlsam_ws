# 문서 진입점 (ros2_3dslam_ws)

본 워크스페이스의 모든 도메인 문서는 본 표에서 진입한다. 표 외 자료는 SSOT (Single Source of Truth / 단일 근원) 미등록이며 audit 대상.

## 영역별 진입점

| 영역 | 진입점 |
| --- | --- |
| **Claude 작업 메타 규칙** (★ 최우선) | [claude_guideline/README.md](claude_guideline/README.md) |
| 프로젝트 운영 규칙 (직접 실행 등) | [claude_guideline/local/project_rules.md](claude_guideline/local/project_rules.md) |
| 구조 · 설계 (GPU offload 포함) | [architecture/](architecture/) |
| 설치 · 실행 · 실험 절차 | [usage/](usage/) |
| 테스트 · 캡처 · 시나리오 | [test/](test/) |
| 이슈 · 수정 기록 | [issues_and_fixes/](issues_and_fixes/) |
| 사용자 지시 기록 | [user_instructions/](user_instructions/) |
| 작업 기록 (worklog) | [worklog/](worklog/) |
| Claude 실수 기록 | [claude-mistake/](claude-mistake/) |
| 코드 리뷰 결과 | [code_review/](code_review/) |
| 리팩토링 계획 · 결과 | [refactoring/](refactoring/) |
| 분석 · 리서치 | [analysis/](analysis/) |
| 문제 해결 (troubleshooting) | [troubleshooting/](troubleshooting/) |
| 수동 작성 API (Application Programming Interface) 참조 | [api/](api/) |
| 이미지 · 다이어그램 (바이너리) | [assets/](assets/) |
| RViz 캡처 (legacy 폴더) | [rviz_screenshots/](rviz_screenshots/) |
| superpowers (외부 스킬 자료) | [superpowers/](superpowers/) |

## 주요 도메인 문서 (SSOT)

| 문서 | 내용 |
| --- | --- |
| [architecture/gpu_offload.md](architecture/gpu_offload.md) | GPU 자동 선택 패턴 (`launch_utils.setup_gpu_offload`) |
| [usage/experiment_execution.md](usage/experiment_execution.md) | SIL/HI 실험 시작 절차, `kill_all_ros2.sh`, pkill self-kill 회피 |
| [test/capture_test.md](test/capture_test.md) | `capture-test` skill 사용 모드, 저장 경로, 인용 규칙 |
| [claude_guideline/local/project_rules.md](claude_guideline/local/project_rules.md) | 직접 실행 원칙, 프로젝트 구조 / 시스템 환경 요약 |

## 변경 절차

규칙·SSOT 변경은 해당 파일 수정 전 사용자 승인 필수. CLAUDE.md 작성·변경 규칙은 [claude_guideline/claude_md.md](claude_guideline/claude_md.md) 참조.
