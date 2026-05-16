# Claude 작업 지침

## 핵심 원칙

1. **사용자가 지시한 사항만 수행한다** (범위를 임의로 확장하지 않는다)
2. **임의로 기능을 추가하거나 변경하지 않는다**
3. **객관적 사실로 판단한다** (코드·로그·매뉴얼 인용, 추측·기억·일반 지식 금지)
4. **모르는 것은 "확인되지 않음" 으로 명시한다** (할루시네이션 금지)
5. **관련 이론을 철저히 조사한 후 시작한다** (공식 문서·매뉴얼·기존 코드 우선)
6. **코딩 전 구조를 제시하고 사용자 승인을 받는다**
7. **검증·보고는 파일·줄·실제 출력을 인용한다** ("동일합니다" 금지)
8. **모든 의도와 결정을 즉시 기록한다** (의도 부채 방지)

## 프로젝트 성격

ROS2 Humble + Ignition Gazebo Fortress 기반 3D / 2D SLAM + Nav2 통합 워크스페이스. RTAB-Map 3D SLAM, Mapper(wall_aligner / wall_detector), Nav2 smac hybrid 등 다중 자율주행 스택을 다룬다.

## 참조 자료 규칙

현재 외부 벤더 매뉴얼 의존 없음. 추가 시 [docs/claude_guideline/manual.md](docs/claude_guideline/manual.md) 절차를 따른다.

## 문서 작업 규칙 (먼저 읽기)

작업 영역에 따라 **시작 전 반드시** 해당 SSOT 를 먼저 읽고 규칙을 따른다. 규칙은 각 파일이 단일 근원(SSOT) 이며 본 CLAUDE.md 에 복제하지 않는다.

### Claude 작업 지침 (메타 규칙)

- 진입점 → [docs/claude_guideline/README.md](docs/claude_guideline/README.md)
  - CLAUDE.md 작성 가이드 → [claude_md.md](docs/claude_guideline/claude_md.md)
  - 사용자 지시사항 처리 SOP → [user_instruction_handling_sop.md](docs/claude_guideline/user_instruction_handling_sop.md)
  - ROS2 / 임베디드 작업 규칙 → [ros2.md](docs/claude_guideline/ros2.md)
  - 작업 절차 체크리스트 → [workflow.md](docs/claude_guideline/workflow.md)
  - 코드 작업 규칙 → [coding.md](docs/claude_guideline/coding.md)
  - GitHub 워크플로 → [github.md](docs/claude_guideline/github.md)
  - 기술 부채 방지 → [tech_debt.md](docs/claude_guideline/tech_debt.md)
  - Iteration 반복 수정 방지 → [iteration_anti_pattern.md](docs/claude_guideline/iteration_anti_pattern.md)
  - 매뉴얼 / 데이터시트 보관·인용 → [manual.md](docs/claude_guideline/manual.md)
  - 스킬 / 자동화 자산 SSOT 등록 → [skill_update.md](docs/claude_guideline/skill_update.md)
  - 문서 작성 방법 → [documentation.md](docs/claude_guideline/documentation.md)
  - 프로젝트별 비공개 override → [docs/claude_guideline/local/](docs/claude_guideline/local/)

### 도메인 문서

- 진입점 → [docs/README.md](docs/README.md)

## 모듈 CLAUDE.md (override 계층)

현재 모듈 CLAUDE.md 없음. 추가 시 본 절에 등록하며 충돌 시 모듈 규칙이 루트보다 우선한다.

## 도메인 문서 SSOT

| 영역 | SSOT |
| --- | --- |
| 프로젝트 운영 규칙 (직접 실행 원칙, 구조·환경 요약) | [docs/claude_guideline/local/project_rules.md](docs/claude_guideline/local/project_rules.md) |
| GPU 자동 선택 (`launch_utils.setup_gpu_offload`) | [docs/architecture/gpu_offload.md](docs/architecture/gpu_offload.md) |
| 실험 실행 절차 (`kill_all_ros2.sh`, pkill 가이드) | [docs/usage/experiment_execution.md](docs/usage/experiment_execution.md) |
| 캡처 · 테스트 증거 수집 | [docs/test/capture_test.md](docs/test/capture_test.md) |

규칙 변경이 필요하면 해당 SSOT 수정 여부를 먼저 사용자에게 문의한다.
