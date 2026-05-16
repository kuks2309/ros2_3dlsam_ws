# 문서 작성 방법

문서를 만들거나 수정할 때 지켜야 할 공통 메타 규칙의 단일 근원.

도메인별 형식(분석, 설계, 캘리브레이션, 실험 결과, 마이그레이션 등)은 각 도메인 README 가 결정하며, 본 문서는 그 위 공통층만 다룬다.

## 명명 규칙

- 파일명: 소문자, 단어 구분은 `_` 또는 `-` (도메인 일관성 우선)
- 확장자: `.md`
- 폴더 진입점: `README.md` 1 개
- 신규 도메인 추가 시: `docs/<domain>/README.md` 생성 → [README.md](README.md) "진입점" 표에 한 줄 링크 추가

## 헤딩 계층

- `# 제목` 1 개 → `##` → `###` 순서로 진행
- 단계 건너뛰기 금지 (예: `#` 다음 `###` 금지)

## 시각/날짜 형식

- 형식: `YYYY-MM-DD HH:MM (KST)` 또는 `YYYY-MM-DD` — KST = Korea Standard Time (한국 표준시)
- 시각 확인: 시스템 시간 기준 (`date "+%Y-%m-%d %H:%M (%Z)"`)
- 24 시간제 사용

## 누적 정렬

- 시간순 기록 문서(요청 기록, 이슈 기록, 캘리브레이션 결과 등) 는 최신이 위 (역순 누적)

## 링크

- 같은 저장소 내 문서: 상대 경로 (`../analysis/...`)
- IDE/뷰어에서 클릭 가능하도록 마크다운 링크 사용

## 코드 블록

- 언어 태그 명시: ` ```cpp `, ` ```python `, ` ```bash `, ` ```yaml `, ` ```markdown ` 등

## 표

- 헤더 행과 정렬 행 명시
- 필요 시 정렬 지정자(`:---`, `:---:`, `---:`) 사용

## 언어

- 본문: 한국어
- 도메인 식별자(토픽 / 노드 / 명령 / 레지스터 / 핀맵 / 명령어 / 경로 / 코드 블록): 원문 유지

## 약자 (Acronym) 병기

- 약자는 **각 문서 안 첫 등장 시 풀네임 병기 필수**.
  - 형식 예: `SOP (Standard Operating Procedure / 표준 운영 절차)` 또는 `SSOT (단일 근원)`
  - 영문 풀네임 + 한국어 의역 둘 다 권장. 한국어 단독도 허용.
- 두 번째 이후 등장은 약자만 사용 OK.
- 자동 검증: [audit.sh](audit.sh) `[acronym-no-expansion]` 룰 — `claude_guideline/` 자산 + 도입 프로젝트 `docs/` 의 `.md` 파일에서 알려진 약자 사전과 비교, 첫 등장 풀네임 미병기 시 경고.
- 약자 사전 (본 프로젝트 도메인): SOP (Standard Operating Procedure), SSOT (Single Source of Truth), MCP (Model Context Protocol), KST (Korea Standard Time), TBD (To Be Determined), PR (Pull Request), CI (Continuous Integration), API (Application Programming Interface), LLM (Large Language Model), ECC (Everything Claude Code), AP-N (Anti-Pattern), D-N (Dimension).

## 인용·근거

- 외부 매뉴얼 인용 시 "매뉴얼명, 페이지/섹션" 명시 ([manual.md](manual.md))
- 코드 또는 매뉴얼에서 확인되지 않은 내용은 "확인되지 않음" 명시
- 추측 기반 기술 금지

## SSOT 원칙

- 규칙은 한 곳에만 작성한다.
- 워크스페이스 루트 `CLAUDE.md` 에는 다른 README/지침 파일 내용을 복제하지 않고, 진입 링크만 둔다.
- 모듈 CLAUDE.md 는 워크스페이스 가이드라인을 복제하지 않고, **모듈 고유 규칙**(핀맵·상수·하드웨어 명령) 만 작성한다.
- 규칙 변경이 필요하면 해당 SSOT 파일을 먼저 수정하고 사용자 승인을 받는다.

## SSOT 경로 (Authoring vs Mirror)

본 가이드라인 자산의 **authoring SSOT** 는 `kuks_claude_setup/claude_guideline/` 1 곳뿐이다.

- **Authoring SSOT** (룰을 수정·신설하는 위치): `kuks_claude_setup/claude_guideline/`
- **Generated mirror** (다운스트림 설치본): `docs/claude_guideline/` — `install.sh` / `update.sh` 산출물. **직접 수정 금지** — 변경이 필요하면 authoring SSOT 를 수정 후 `update.sh` 재실행.
- 본 레포 내부 cross-reference 는 bare 상대경로 (`[claude_md.md](claude_md.md)`) 를, 다운스트림 사용자가 실행하는 명령은 `docs/claude_guideline/...` 절대 표기를 쓴다.
- mirror 직접 수정 시 다음 `update.sh` 에서 덮어쓰임. audit.sh `[tree-diff]` / `[mirror-only-file]` 룰이 drift 검출.

## 용어 표준 (Terminology SSOT)

본 워크스페이스의 모든 마크다운 자산이 따르는 표기 표준. 어휘 / 어투 / 디렉터리 네이밍 / 파일명을 한 곳에서 정의한다 (재발 방지: `audit.sh [terminology-drift]`).

### 표준 어휘

| 개념 | KO prose | EN / 제품명 | 코드 / CLI 토큰 | 파일명 |
|---|---|---|---|---|
| skill | **스킬** | Skill (Title) | `skill` | `SKILL.md` (번들) / `skills/<kebab>.md` (인라인) |
| agent | **에이전트** / 서브에이전트 | Agent (Title) | `agent`, `sub-agent` | — |
| workflow | **워크플로** | workflow | `workflow` | — |
| user instruction | **사용자 지시사항** (명사) / "사용자가 지시한 사항" (서술구) | (영문 사용 금지 — 한국어 SSOT) | — | — |
| guideline | **지침** (umbrella) / **규칙** (도메인) / **원칙** (불변량) | guideline | — | — |

- `워크플로우` (잘못된 표기) → `워크플로`. `사용자 요청` / `요청사항` / `request` → `사용자 지시사항`.
- prose 의 `SKILL` (all-caps) 금지 — 파일명 / CLI 토큰 한정.

### 어투 규칙

- `claude_guideline/**` (메타 규칙) = **평어 (`-한다`)**
- `skills/**`, `agents/**`, 패키지 README, 루트 `README.md` = **존댓말 (`-합니다`)**
- 한 파일 내 평어 / 존댓말 혼용 금지 (audit.sh 가 검출).

### 디렉터리 네이밍

- 표준: **kebab-case** (예: `claude-mistake`, `pptx-design-styles`, `web-to-pdf`).
- Legacy snake-case (`claude_guideline/`, `conversation_backup/`) 는 외부 cross-reference 비용으로 유예. 신규 디렉터리는 kebab 강제.
- 한글 디렉터리명 (`hwp스킬/`) 은 후속 PR 에서 ASCII rename.

### 파일명 ↔ H1 일관성

- 파일명과 H1 헤딩이 같은 어휘를 사용해야 한다 (`documentation.md` ↔ `# 문서 작성 규칙`).
- 다국어 README 는 H1 도 언어별로 분기 (`README.md` = 영문 H1, `README_ko.md` = 한국어 H1).

## 폴더 명명 규칙 (Canonical)

폴더명은 파일명보다 강한 일관성을 요구한다. 변종이 누적되면 install.sh / audit.sh 가 SSOT 룰을 자동 적용할 수 없게 되기 때문이다.

- **구분자**: 언더바 `_` 만 허용. 하이픈 `-` / 공백 / 한글 폴더명 금지.
  - 위반 예: `code-review/`, `sw-architecture/`, `GLIM-slam only/`, `함수리스트/`
- **단·복수 일관**:
  - 컬렉션(여러 entry 누적): 복수형 — `issues_and_fixes/`, `assets/`, `tests/`
  - 단일 카테고리(주제 한 가지): 단수형 — `architecture/`, `usage/`, `code_review/`, `refactoring/`
- **숫자 prefix**: 사용 시 두 자리 — `01_`, `02_`. 그룹화 순서가 의미를 가질 때만(예: `manual/01_TMflow/`).
- **acronym 예외**: `README`, `LICENSE`, `CHANGELOG`, `VERSION`, `API` 는 대문자 유지(파일명에 한함, 폴더명에는 사용 금지).
- **SSOT 배포 자산 예외**: `kuks_claude_setup` SSOT 가 정의한 폴더명은 SSOT 표기를 그대로 유지한다. 위 언더바-only 룰을 따르지 않더라도 변종이 아니라 **canonical**. 현재 해당 자산:
  - `claude_guideline/` (언더바)
  - `claude-mistake/` (하이픈) — SSOT 가 하이픈으로 정의했으므로 그대로 사용
  - `superpowers/` (단어 하나)
  - SSOT 가 신규 자산을 추가하면 본 목록도 갱신한다.

## 표준 레이아웃 (Canonical Tree)

### Repo-root 직속

| 항목 | 역할 |
|---|---|
| `README.md` | 프로젝트 진입점 |
| `LICENSE` | 라이선스 (MIT/Apache-2.0 권장) |
| `CHANGELOG.md` | 변경 이력 |
| `VERSION` | semver |
| `manual/` | **외부 벤더 매뉴얼 PDF**(우리가 저작하지 않은 자료). `manual/SOURCES.md` 필수 — [manual.md](manual.md) 형식 참조 |
| `api/` | **자동생성 API 참조**(Doxygen / Sphinx 등). 빌드 아티팩트 성격이면 root 위치. 수동작성이면 `docs/api/` 와 택일 (한 repo 안에서 한 쪽만 선택) |
| `docs/` | **프로젝트 자체 문서**(우리가 작성·관리하는 모든 산출물) |

### docs/ 표준 폴더

**install.sh / update.sh 가 아래 표준 폴더를 모두 자동 생성**한다 (v1.8.6 정정). 각 폴더에는 역할·SSOT 링크를 담은 README.md 가 자동 배치된다. 활동이 발생하지 않은 폴더는 README 만 남고 비어 있어도 정상 (이전 v1.8.5 까지 "활동 발생 시 추가, 빈 폴더 금지" 정책은 SOP 가 의존하는 폴더의 부재로 인한 마찰을 양산하여 폐기).

| 폴더 | 역할 | 흡수하는 변종 |
|---|---|---|
| `architecture/` | 구조 · 설계 | `sw_structure/`, `sw_structures/`, `sw-architecture/`, `PROJECT_STRUCTURE.md` |
| `usage/` | 설치 · 실행 · 튜토리얼 | `INSTALLATION.md`, `USER_GUIDE.md`, `run_guide.md`, `usage.md` |
| `issues_and_fixes/` | 이슈 · 수정 기록 (시간 누적 역순) | `issues-fixes/`, `issues_fixes/` |
| `user_instructions/` | **사용자가 터미널에 입력한 지시사항의 시간 누적 기록** — 형식은 [user_instruction_handling_sop.md](user_instruction_handling_sop.md) §3 (`> "<원문 인용>"` + `### 처리` + `### 결론/산출물`). 결과 요약·리뷰 보고서·분석 결과는 여기 두지 말 것 → `code_review/` 또는 `analysis/` 로 분리 | — |
| `worklog/` | 작업 기록 | — |
| `assets/` | 이미지 · 동영상 · 스크린샷 · 다이어그램 | `rviz_screenshots/`, 흩어진 `*.png` / `*.mp4` |
| `claude-mistake/` | **Claude(LLM) 실수 누적 기록, 재발 방지** — SSOT 자산. 형식은 [claude-mistake/README.md](../claude-mistake/README.md) — 날짜별 `YYYY-MM-DD.md` + 5섹션(무엇을 했는가 / 잘못 / 사용자 지적 / 사유 / 재발 방지). **하이픈 표기 유지(SSOT 정의)** | `mistake/`, `claude_mistake/`, `claude_mistakes/` |
| `code_review/` (옵션) | 코드 리뷰 결과 | `code-review/`, `code_reivew/`, `CODE_REVIEW_REPORT.md` |
| `refactoring/` (옵션) | 리팩토링 계획·결과 | `IMPLEMENTATION_PLAN.md` (리팩토링 한정) |
| `analysis/` (옵션) | 분석 · 리서치 | `*_research.md`, `*_summary.md`, `phase*_report.md` |
| `test/` (옵션) | 테스트 시나리오 · 리포트 | `test_scenarios/`, `test_reports/` |
| `troubleshooting/` (옵션) | 문제 해결 | `TROUBLESHOOTING.md` (단일 파일은 폴더로 승격) |
| `api/` (옵션) | 수동작성 API 참조 | `API.md` — repo root `api/` 와 택일 |
| `code_updates/` (옵션) | 코드 변경 로그 — 위치 룰은 아래 ROS2 특칙 참조 | `*_code_updates.md` 평탄 5개+ |
| `claude_guideline/` | SSOT install 결과 — **직접 수정 금지**, `update.sh` 로만 갱신 | — |
| `superpowers/{plans,specs}/` (옵션) | Claude superpowers 워킹스페이스 | — |

### ROS2 워크스페이스 특칙

워크스페이스 레벨 docs/ 와 패키지 docs/ 의 책임을 분리한다.

```text
<workspace>/
├── docs/
│   ├── README.md
│   ├── architecture/        # 워크스페이스 전반 설계 (여러 패키지 묶음 책임)
│   ├── usage/
│   ├── issues_and_fixes/
│   └── code_updates/        # ★ 워크스페이스 횡단 변경만 (인터페이스/메시지/QoS 등)
└── src/
    └── <pkg>/
        ├── package.xml
        ├── CMakeLists.txt
        └── docs/             # ★ 패키지 자급자족 문서
            ├── README.md
            ├── architecture/  # 패키지 내부 설계
            └── code_updates/  # ★ 패키지별 변경 로그 (디폴트 위치)
```

- 단일 패키지 안 코드 변경 → `src/<pkg>/docs/code_updates/`
- 여러 패키지 동시 영향(공유 메시지 / 액션 인터페이스 변경, QoS 정책 변경 등) → 워크스페이스 `docs/code_updates/`
- 비-ROS2 단일 repo (라이브러리·도구·문서 워크스페이스) → 그냥 `docs/code_updates/`

## Variant → Canonical 매핑 (audit.sh 자동 적용 대상)

| 발견된 변종 | Canonical | 근거 |
|---|---|---|
| `code-review/`, `code_reivew/` | `code_review/` | 단수형 + 언더바 + 오타 차단 |
| `issues-fixes/`, `issues_fixes/` | `issues_and_fixes/` | 복수형 표준 |
| `sw_structure/`, `sw_structures/`, `sw-architecture/` | `architecture/` | 단수형 표준 |
| `stratedgy/` | `strategy/` | 오타 정정 |
| `mistake/`, `claude_mistake/`, `claude_mistakes/` | `claude-mistake/` | SSOT 자산명 유지 (하이픈) |
| `docs/user_instructions/` 안에 `*review*`, `*report*`, `*summary*`, `*analysis*` 파일명 | `docs/code_review/` 또는 `docs/analysis/` 로 이동 | `user_instructions/` 는 사용자 지시 기록 전용 |
| `INSTALLATION.md`, `USER_GUIDE.md`, `run_guide.md`, `usage.md` (직속 단일 파일) | `usage/` 폴더로 승격 + 내부 `.md` 분리 | 단일 파일 → 폴더 |
| `TROUBLESHOOTING.md` (직속 단일 파일) | `troubleshooting/` 폴더로 승격 | 동일 |
| `API.md` | `docs/api/` (수동작성) 또는 root `api/` (자동생성) | 위치 룰 |
| `*_code_updates.md` 평탄 5개 이상 | `code_updates/` 또는 `src/<pkg>/docs/code_updates/` 로 묶음 | ROS2 워크스페이스이면 패키지별 우선 |
| docs/ 안 외부 PDF | root `manual/` 으로 이전 + `manual/SOURCES.md` 추가 | 외부 vs 자체 책임 분리 |
| 한글 폴더명 / 공백 포함 폴더명 | 영문 ASCII 언더바로 재명명 | 폴더 명명 규칙 |

## 변종 차단 룰 (audit.sh)

`bash docs/claude_guideline/audit.sh [path...]` 실행 시 다음을 검사하고 권고만 출력한다(파일 이동 없음, dry-run 전용).

1. **variant 폴더 검출** — 위 매핑표 기준
2. **평탄 .md 검출** — docs/ 직속 .md 가 5개 이상이면 카테고리 폴더 권고
3. **외부 PDF 위치 위반** — docs/ 안 PDF 발견 시 root `manual/` 이전 권고
4. **ROS2 패키지 docs 누락** — `src/<pkg>/package.xml` 존재 + `src/<pkg>/docs/` 부재 시 생성 권고
5. **오탈자 폴더명** — 사전 정의된 오탈자 사전과 매칭하여 정정 제안
6. **빈 폴더** — `docs/<x>/` 가 비어있으면 폴더 자체 삭제 또는 `.gitkeep` + README 권고

`audit.sh --fix` (향후 옵션, 디폴트 OFF) — 위 검출 결과를 실제 `git mv` 로 적용. 기본은 항상 dry-run.

## 책임 분리 한 줄 요약

- **외부 자료(우리가 저작 X)** → repo root `manual/`, `api/`(자동생성)
- **우리가 저작·관리** → `docs/`
- **패키지 내부 자급자족 문서(ROS2)** → `src/<pkg>/docs/`
- **워크스페이스 횡단 / 인터페이스** → 워크스페이스 `docs/`
