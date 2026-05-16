# CLAUDE.md 작성 가이드 (SSOT / Single Source of Truth / 단일 근원)

본 SSOT 는 CLAUDE.md 파일의 정의·구조·작성 절차·검증·변종 차단 룰을 정의한다.

본 가이드의 출처는 [docs/projects_analysis/claude_md_analysis.md](../../docs/projects_analysis/claude_md_analysis.md) (23개 CLAUDE.md 전수 분석) + [common_features_priority.md](../../docs/projects_analysis/common_features_priority.md) (공통 기능 우선순위 추출).

## 1. 정의

CLAUDE.md 는 Claude Code 가 프로젝트 작업 시 *자동 로드* 하는 메타 지침 파일.

- **위치**: 프로젝트 루트 (필수) + 모듈 단위 (선택, override 계층)
- **명명**: 대문자 `CLAUDE.md` 만 허용 (소문자 `claude.md` 는 Claude Code 가 자동 로드하지 않음)
- **인코딩**: UTF-8 마크다운
- **분량**: 50줄 내외 (얇은 진입점) — SSOT 위임형 패턴 권장

## 2. 핵심 원칙 (모든 CLAUDE.md 가 포함해야 하는 8대 원칙)

본 8대 원칙은 23개 CLAUDE.md 분석에서 16개가 채택한 5대 원칙 + 본 프로젝트의 의도 부채 분석 결과 추가 3개로 구성.

### 2.1 시간 흐름 순서

1. **모든 의도와 결정을 즉시 기록한다** (의도 부채 방지 — 가장 큰 부채)
2. **객관적 사실로 판단한다** (코드·로그·매뉴얼 인용, 추측·기억·일반 지식 금지)
3. **모르는 것은 "확인되지 않음" 으로 명시한다** (할루시네이션 금지)
4. **사용자가 지시한 사항만 수행한다** (범위를 임의로 확장하지 않는다)
5. **관련 이론을 철저히 조사한 후 시작한다** (공식 문서·매뉴얼·기존 코드 우선)
6. **코딩 전 구조를 제시하고 사용자 승인을 받는다**
7. **임의로 기능을 추가하거나 변경하지 않는다** (범위 이탈 사전 차단)
8. **검증·보고는 파일·줄·실제 출력을 인용한다** ("동일합니다" 금지)

### 2.2 추가 원칙 (선택)

- 같은 실수를 다른 형태로 반복하지 않는다 ([iteration_anti_pattern.md](iteration_anti_pattern.md))

## 3. 표준 구조 (Thin SSOT 위임형)

본 프로젝트(claude_code) 의 CLAUDE.md 가 표준 모범 (≤60줄, 얇은 SSOT 위임형). 본문은 SSOT 링크만, 룰 authoring SSOT 는 `kuks_claude_setup/claude_guideline/` 1 곳. downstream 프로젝트는 `docs/claude_guideline/` mirror 를 참조하되 직접 수정하지 않는다 ([documentation.md](documentation.md) §SSOT 경로 참조).

### 3.1 권장 템플릿 ([templates/CLAUDE.md.template](templates/CLAUDE.md.template) 참조)

```markdown
# Claude 작업 지침

## 핵심 원칙

(§2 8대 원칙 본문 또는 SSOT 링크)

## 프로젝트 성격

<프로젝트 1-2줄 설명 + SSOT 원격 링크>

## 참조 자료 규칙

(도메인별 참조 자료가 추가되면 본 절에 SSOT 링크 작성)

## 문서 작업 규칙 (먼저 읽기)

### Claude 작업 지침 (메타 규칙)

- 진입점 → docs/claude_guideline/README.md
  - CLAUDE.md 작성 가이드 → claude_md.md
  - 사용자 지시사항 처리 SOP (Standard Operating Procedure / 표준 운영 절차) → user_instruction_handling_sop.md
  - 지시 작성 가이드 → request_giving.md
  - 작업 절차 체크리스트 → workflow.md
  - 코드 작업 규칙 → coding.md
  - GitHub 워크플로 → github.md
  - 기술 부채 방지 → tech_debt.md
  - Iteration 반복 방지 → iteration_anti_pattern.md
  - 매뉴얼 인용 → manual.md
  - 스킬 / 자동화 자산 → skill_update.md
  - 문서 작성 → documentation.md
  - 프로젝트별 비공개 override → local/

### 도메인 문서

- 진입점 → docs/README.md

## 모듈 CLAUDE.md (override 계층)

(현재 모듈 CLAUDE.md 없음. 추가 시 본 절에 등록하며 충돌 시 모듈 규칙이 루트보다 우선한다.)

## 도메인 문서 SSOT

(도메인 추가 시 docs/README.md 진입점 표에 등록)

규칙 변경이 필요하면 해당 README 수정 여부를 먼저 사용자에게 문의한다.
```

## 4. 작성 절차

### 4.1 신규 프로젝트 시작 시

1. 템플릿 복사: `cp kuks_claude_setup/claude_guideline/templates/CLAUDE.md.template <project>/CLAUDE.md`
2. 프로젝트 성격 1-2줄 작성 (목적, 도메인, 외부 SSOT 링크)
3. 적용할 SSOT 도메인 문서 식별 (ROS2/임베디드/AI/Web 등)
4. 도메인 문서 SSOT 표 채움
5. install.sh 로 docs/claude_guideline/ 동기화
6. 사용자 검토 → 승인 → 첫 commit

### 4.2 모듈 CLAUDE.md 추가 시

1. 루트 CLAUDE.md 의 "모듈 CLAUDE.md" 절에 등록
2. 모듈별 *특수 규칙만* 작성 (루트 8대 원칙 자동 상속, 중복 작성 금지)
3. override 시 명시: "본 모듈은 X 룰을 다음과 같이 변경 적용함"

### 4.3 룰 변경 시

1. 사용자 승인 필수 (8대 원칙 #4 적용)
2. SSOT 파일 변경 → 영향받는 CLAUDE.md 동기화
3. CHANGELOG / VERSION 갱신
4. push 전 사용자 명시 확인 ([github.md](github.md) Push 전 확인 절)

## 5. 검증 체크리스트

신규/수정 후 확인:

- [ ] 8대 원칙 전부 포함 (또는 §2 SSOT 링크)
- [ ] 프로젝트 성격 1-2줄 명시
- [ ] 도메인 문서 SSOT 표 채움
- [ ] 모듈 CLAUDE.md 가 있다면 모두 등록
- [ ] 외부 SSOT 링크 작동 (404 없음, 상대경로 정확)
- [ ] 명명 규약: 대문자 `CLAUDE.md` (소문자 `claude.md` 금지)
- [ ] 50줄 내외 (얇은 진입점) — 두꺼우면 SSOT 로 분리
- [ ] 변경 사항 사용자 승인 받음
- [ ] [audit.sh](audit.sh) dry-run 통과 (린트·구조·SSOT 정합성)

## 6. Variant 차단 (anti-pattern 방지)

다음 변형 금지 — [audit.sh](audit.sh) 가 자동 감지:

| Variant | 위반 사유 | 자동 정정 |
|---|---|---|
| 소문자 `claude.md` | Claude Code 자동 로드 안 됨 | rename → `CLAUDE.md` |
| 같은 워크스페이스의 분기 사본 (`current/CLAUDE.md` vs `_old/CLAUDE.md`) | 정보 손실, 동기화 부담 | `_old` 정리, single source 통일 |
| 본문에 SSOT 룰 직접 복제 (예: 350줄+ 두꺼운 CLAUDE.md) | SSOT 위반, 변경 시 다중 동기화 | docs/claude_guideline/ 링크로 분리 |
| 8대 원칙 누락 | 거버넌스 표준 미충족 | 누락 원칙 추가 |
| 모듈 CLAUDE.md 미등록 | 루트가 모듈 존재 모름 | 루트 "모듈 CLAUDE.md" 절에 추가 |

본 프로젝트의 23개 CLAUDE.md 분석에서 발견된 실제 위반 사례:

- `Project/Charging_Robot/claude.md` (소문자) → 정정 대상
- `Project/FITO2026/MLCC_Index/claude.md` (소문자) → 정정 대상
- `T-Robotics/TM_Robot_ros2_ws/CLAUDE.md` ↔ `T-Robotics/TM_Robot_ros2_ws_old/CLAUDE.md` (분기 사본) → 정리 대상
- `parking_robot_ros2_ws/CLAUDE.md` (352줄) → SSOT 분리 대상

## 7. 모듈 override 패턴

상위 워크스페이스의 CLAUDE.md 와 모듈 CLAUDE.md 의 관계:

| 패턴 | 사례 | 권장 |
|---|---|---|
| **상속** (모듈이 상위 룰 그대로 따름) | TM_Robot_Task_Manager (124줄, 상위 지침 상속) | OK |
| **부분 override** (특정 룰만 모듈에서 변경) | parking_robot/.../ros2_calib (오프라인 작동, ROS1→ROS2 변환 룰 비적용) | 명시 권장 |
| **자체 완결** (모듈이 독립 도메인) | Hailo-Compiler-UI (WSL2 전용) | 별도 SSOT 권장 |

## 8. 본 가이드의 출처

본 SSOT 는 다음 분석에서 추출:

- [docs/projects_analysis/claude_md_analysis.md](../../docs/projects_analysis/claude_md_analysis.md) — 23개 CLAUDE.md 전수 분석 (도메인별 그룹화, 공통/고유 요소)
- [docs/projects_analysis/common_features_priority.md](../../docs/projects_analysis/common_features_priority.md) — 공통 기능 10종, P0-P3 우선순위, 4단계 마이그레이션 계획
- [docs/projects_analysis/instruction_classification_analysis.md](../../docs/projects_analysis/instruction_classification_analysis.md) — 분류 시스템 D1-D13, anti-pattern AP-1~AP-10

## 9. 변경 절차

본 룰은 SSOT 이므로 변경 시 사용자 승인 필수. 변경 후:

1. CLAUDE.md 의 "참조 자료 규칙" 절에 본 SSOT (`claude_md.md`) 가 등록되어 있는지 확인
2. README.md 표 / 진입점 표 갱신
3. CHANGELOG.md / VERSION (semver) 갱신
4. install.sh / update.sh 의 FILES 배열에 `claude_md.md` 추가 (외부 배포 대상)
5. 본 SSOT 가 적용된 다른 프로젝트 CLAUDE.md 동기화 안내 (CHANGELOG 통해)
