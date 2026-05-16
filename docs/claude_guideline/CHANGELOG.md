# Changelog

## 1.8.6 — 2026-05-11

docs/ 표준 폴더 전체를 install/update 자동 생성 대상으로 확장 + "빈 폴더 금지" 정책 폐기. 이전까지 v1.8.5 의 SOP 의존 3개 폴더만 자동 생성되었으나, SOP 가 의존하지 않는 일반 카테고리 폴더 (architecture / usage / issues_and_fixes / assets / code_review / refactoring / analysis / test / troubleshooting / api) 도 신규 프로젝트에서 사용자가 수동 mkdir 해야 했던 마찰 해소.

### 변경

- `claude_guideline/documentation.md` §docs/ 표준 폴더:
  - **수정**: "처음 생성 시에는 `docs/README.md` 하나만 두고 … 활동 발생 시 추가 (빈 폴더 금지)" 정책 폐기. install/update 가 모든 표준 폴더를 자동 생성하며, 활동 미발생 폴더는 README 만 남고 비어 있어도 정상.
- `claude_guideline/audit.sh`:
  - §7 `[empty]` 룰을 `[no-readme]` 로 정정. 빈 폴더 자체는 OK, README.md 부재만 정보성 권고. `N_ISSUE+=1` 카운트도 제거 (강제 X).
- `claude_guideline/install.sh`, `update.sh`:
  - 자동 생성 폴더 4종 → 13종 확장: architecture, usage, issues_and_fixes, assets, user_instructions, worklog, claude-mistake, code_review, refactoring, analysis, test, troubleshooting, api.
  - 각 폴더에 역할·SSOT 링크를 담은 stub README 자동 배치 (`write_stub` 헬퍼 함수). claude-mistake/README.md 만 SSOT 원격 다운로드.
  - 기존 README 가 있으면 덮어쓰지 않음 (모든 stub 작성에 `if [ -f ... ]` 가드).

### 트리거

사용자 지적 (2026-05-11):

> "폴더 /home/amap/Project/claude_code/docs 에 있는 폴더는 다 설치하게 만들어야 하지 않을까요?"
> "'처음 생성 시에는 docs/README.md 하나만 두고, 아래 폴더는 해당 활동이 실제로 발생하면 그 시점에 추가한다(빈 폴더 금지).' ← 수정해야 합니다. 잘못 만들어진 정책임"

근본 원인: v1.8.5 가 SOP 의존 3개 폴더만 자동화하고 나머지 표준 폴더는 기존 정책("활동 발생 시 추가") 을 따랐으나, 신규 프로젝트가 SSOT 표준 구조를 한 번에 받지 못해 사용자가 폴더마다 수동 mkdir 해야 하는 마찰 누적. 정책 자체가 install 자동화와 충돌.

### 호환성

patch bump (1.8.5 → 1.8.6). 기존 다운스트림은 `bash docs/claude_guideline/update.sh` 실행 시 누락 폴더 자동 보충. 기존 README 는 보존. audit.sh 의 `[empty]` 경고를 받던 다운스트림은 본 룰 정정으로 경고 사라짐.

## 1.8.5 — 2026-05-11

SOP 가 의존하는 표준 폴더(`docs/claude-mistake/`, `docs/user_instructions/`, `docs/worklog/`)를 install/update 가 자동 생성하도록 정정. 이전까지는 `claude_guideline/` 만 자동 설치되고 나머지 3개 폴더는 사용자가 수동 mkdir 해야 했음 — SOP 흐름에 폴더 부재 시 어떻게 만드는지 절차 누락.

### 변경

- `claude_guideline/install.sh`:
  - **NEW**: `mkdir -p docs/claude-mistake docs/user_instructions docs/worklog` 자동 실행.
  - **NEW**: 각 폴더에 README.md 생성 — claude-mistake 는 SSOT 원격에서 다운로드, user_instructions / worklog 는 install.sh heredoc 으로 stub 생성 (역할·형식·SSOT 링크 명시).
  - 기존 파일이 있으면 덮어쓰지 않음 (`if [ ! -f ... ]` 가드).
- `claude_guideline/update.sh`:
  - 동일 mkdir + README 보강 — 기존 다운스트림이 update.sh 실행 시 누락된 폴더 자동 보충.
  - 기존 README 는 덮어쓰지 않음.

### 트리거

사용자 지적 (2026-05-11):

> "현재 mistake 부분에서 프로젝트 처음 설치할때 폴더를 만들게 되어 있나요?"
> "폴더 /home/amap/Project/claude_code/docs/user_instructions 도 처음 프로젝트에 적용할때 폴더 설치하라는 지시가 없는것 같은데요."

근본 원인: v1.8.2 / v1.8.3 의 SOP 정정이 `worklog/` 책임을 새로 부여하고 `user_instructions.md` 기록 절차를 강화했으나, install.sh 가 `claude_guideline/` 만 다루도록 설계되어 신규 프로젝트가 SOP 를 따르려면 사용자가 별도로 폴더를 만들어야 하는 마찰 존재. SOP 가 의존하는 모든 폴더는 install 단계에서 자동 보장되어야 함.

### 호환성

patch bump (1.8.4 → 1.8.5). 기존 다운스트림은 `bash docs/claude_guideline/update.sh` 실행 시 누락된 폴더 자동 보충. 이미 폴더와 README 가 있다면 그대로 유지.

## 1.8.4 — 2026-05-11

`kuks_claude_setup/claude-mistake/` 책임 경계 정정 — v1.8.2 / v1.8.3 에서 SSOT 본판에 잘못 push 한 사건 entry 제거 + README 명시화.

### 변경

- `claude-mistake/2026-05-07.md` 제거 (`git rm`) — 본 파일은 본 SSOT 폴더가 아닌 사건 발생 프로젝트의 `docs/claude-mistake/` 에 있어야 함.
- `claude-mistake/2026-05-11.md` 제거 (`git rm`) — 본 워크스페이스 (claude_code) 사건이므로 `claude_code/docs/claude-mistake/2026-05-11.md` (이미 존재) 가 정상 위치.
- `claude-mistake/INDEX.md` 제거 (`git rm`) — 카테고리 인덱스 운용은 다운스트림 책임. 본 SSOT 에는 형식 정의만.
- `claude-mistake/README.md`:
  - **NEW §책임 경계** — 본 SSOT 폴더에는 README 만, 사건 entry / INDEX 는 다운스트림 `docs/claude-mistake/` 에만 작성. 본 SSOT 에 사건을 두면 다른 프로젝트에 무관한 사건이 노출되는 책임 경계 침범.
  - §Closure 규칙 / §검토 시점: INDEX.md 참조를 다운스트림 `docs/claude-mistake/INDEX.md` 로 명시.
  - §설치 방법 마지막 줄에 "SSOT 에는 본 README 만" 강조.

### 트리거

사용자 지적 (2026-05-11 06:10 KST):

> "일반화 폴더에 이 프로젝트의 내용을 기록하는 특별한 이유가 있는지?"
> "폴더 /home/amap/Project/claude_code/kuks_claude_setup/claude-mistake 가 일반화 폴더라고 1000번 더 이야기 할까"

근본 원인: v1.8.2 / v1.8.3 patch 시 본 워크스페이스 사건을 SSOT 본판에 commit (`1c34741`, `be47c6c`). 또한 v1.8.2 이전부터 있던 `2026-05-07.md` (`73fc92b`) 도 같은 책임 경계 위반인데 정정 누락. README 가 묵시적으로만 다운스트림 위치를 가리켰을 뿐 명시적이지 않아, 운영자(Claude/이전 작업자) 가 SSOT 에 사건을 push 하는 패턴이 반복됨.

### 호환성

patch bump (1.8.3 → 1.8.4). 다운스트림 영향:
- 다운스트림이 SSOT 의 `claude-mistake/2026-05-*.md` 를 참조하지 않았다면 영향 없음.
- 참조했다면 자체 `docs/claude-mistake/` 의 사건 기록을 사용하도록 수정 필요 (애초에 다른 프로젝트의 사건이라 참조해서는 안 되는 데이터).

## 1.8.3 — 2026-05-11

`claude-mistake/` 학습 closure 인프라 신설 — INDEX.md 카테고리 색인 + closure 규칙 + SessionStart hook + audit.sh 검출 룰. 1.8.2 의 SOP 정정이 다운스트림 패턴을 차단했다면 본 1.8.3 은 같은 세션 내 재발 차단을 시스템화.

### 변경

- `claude-mistake/INDEX.md`: NEW
  - §메타 패턴 (사전 스캔 없이 생성 모드 진입 / SSOT 무비판 채용 / 카테고리 학습 미적용)
  - §카테고리 × 사건 매트릭스 (현재 7 카테고리, 3 entry)
  - §미해결 항목 (open) 추적 표
  - §운용 규칙 (신규 entry / closure / 카테고리 신설 / 세션 시작 검토)
  - LLM-HINT 주석 마커 — SessionStart 시 자동 주입 대상 영역 표시
- `claude-mistake/README.md`:
  - **NEW §Closure 규칙** — 반영 자산 1개 이상 / TBD 금지 / 카테고리 부착 / owner 명시
  - §기존 실수 검토 시점: SessionStart hook (자동) + 작업 시작 전 (수동) + 사용자 정정 직후 3 시점으로 확장
- `claude-mistake/2026-05-07.md`, `2026-05-11.md`: 헤더에 카테고리 뱃지 + 상태 + INDEX 링크 추가 (`> **카테고리**: ... · **상태**: closed (vN.N.N)`).
- `claude_guideline/audit.sh`:
  - **NEW [user-instructions-headings]**: `docs/user_instructions/*.md` 안 `### 처리|### 결론|### 산출물` 헤딩 검출 (v1.8.2 SOP 정정의 audit 강제).
  - **NEW [legacy-request]**: legacy `docs/request/` 폴더 검출 시 `user_instructions/` 로 rename 권고.
  - **NEW [mistake-order]**: `docs/claude-mistake/YYYY-MM-DD.md` 안 `## ... HH:MM` 헤딩이 시간 역순 (최신 위) 위반 시 검출.
  - **NEW [mistake-tbd]**: `docs/claude-mistake/*.md` 안 TBD/후보/추후/미정 키워드 검출 (closure 의무 환기).
  - **NEW [hint]** `docs/claude-mistake/INDEX.md` 부재 권고.
- `claude_guideline/hooks/`: NEW
  - `session_start_claude_mistake.sh`: SessionStart 훅 — `claude-mistake/INDEX.md` §메타 패턴 + §미해결 항목 자동 주입.
  - `README.md`: hook 등록 가이드 (`~/.claude/settings.json` 또는 `<project>/.claude/settings.json`).
- `claude_guideline/install.sh`, `update.sh`: hooks/ 디렉토리 생성 + 다운로드 + chmod 추가. FILES 배열에 `claude_md.md` (CLAUDE.md 작성 가이드 SSOT, 172줄) 등록.
- `claude_guideline/claude_md.md`: SSOT 자산 정식 등록 (이전 untracked 상태에서 install/update 대상으로 승격).

### 트리거

10명 sub-agent + Codex/Gemini 검토에서 도출된 closure 결함:
- "권고 없이 자동 주입 메커니즘 부재" (Methodology, Analyst, UX, Architect 4개 합의)
- "카테고리/인덱스 부재로 100건 누적 시 grep 의존" (Analyst, UX, Architect 3개 합의)
- 근본 원인 (A) "트리거 부재" 가설 채택 (Codex, Gemini, Tracer 모두 합의)

본 1.8.3 은 (A) 트리거 부재 가설에 대한 직접 처방: SessionStart hook 으로 INDEX top-N 자동 주입 + audit.sh 로 closure 의무 자동 검출.

### 호환성

patch bump (1.8.2 → 1.8.3). 기존 다운스트림은 `bash docs/claude_guideline/update.sh` 로 hooks/ 폴더 + audit.sh 갱신본 자동 다운로드. SessionStart hook 활성화는 사용자가 `~/.claude/settings.json` 에 entry 1줄 추가 (수동).

## 1.8.2 — 2026-05-11

`user_instruction_handling_sop.md` ↔ `documentation.md` 충돌 정정 — 1.8.1 의 `docs/user_instructions/` 정의 강화("사용자 지시사항 전용") 가 SOP §3 의 형식(`### 처리` / `### 결론·산출물`)과 충돌하여, 다운스트림에서 결과 요약이 `docs/user_instructions/` 에 누적되는 패턴을 양산. 본 패치는 SOP 측 형식 정정으로 충돌 해소.

### 변경

- `claude_guideline/user_instruction_handling_sop.md`:
  - **§3 (Step 2)**: `docs/user_instructions/user_instructions.md` 기록 형식에서 `### 처리` / `### 결론 / 산출물` 섹션 제거. 사용자 원문 인용만 남김.
  - **§9 (Step 8)**: 결과·산출물 기록 책임을 `docs/worklog/` (또는 `code_review/` / `analysis/` / `refactoring/` / `troubleshooting/`) 로 이동. user_instructions.md 와 worklog entry 는 시각·제목으로 매핑.
  - **§1 흐름도**: Step 8 라벨을 "user_instructions.md 결론 갱신" → "worklog 결과 기록" 으로 갱신.
  - 신규 ✓ 체크: `grep -E "^### (처리|결론|산출물)" docs/user_instructions/user_instructions.md` → 출력 없어야 함.

### 트리거

`claude-mistake/2026-05-11.md` 05:10 entry — Claude 가 같은 세션 안에서 동일 카테고리 ("결과 요약을 `user_instructions/` 에 넣음") 를 두 번째로 재현. 첫 번째는 `ccg-review-2026-05-10.md` 위치 오류, 두 번째는 `user_instructions.md` 안 `### 처리` / `### 결론` 섹션 작성.

10명 sub-agent 검토 + Codex/Gemini 외부 검토에서 "1.8.1 의 `user_instructions/` 정의와 SOP §3 형식이 충돌" 로 확정.

### 호환성

patch bump (1.8.1 → 1.8.2). 기존 `user_instructions.md` 에 이미 작성된 `### 처리` / `### 결론` 섹션은 본 패치로 자동 제거되지 않음 — `audit.sh` 의 차기 룰(P1)이 검출 후 사용자가 worklog 로 이전.

## 1.8.1 — 2026-05-11

1.8.0 누락 수정 — `claude-mistake/` (SSOT 자산) canonical 등록 + `docs/user_instructions/` 정의 명확화.

### 변경

- `claude_guideline/documentation.md`:
  - **NEW**: 폴더 명명 규칙에 **SSOT 배포 자산 예외** 추가. `claude_guideline/` (언더바), `claude-mistake/` (하이픈), `superpowers/` 는 SSOT 표기를 그대로 유지(언더바-only 룰의 예외).
  - **NEW**: docs/ 필수 폴더 표에 `claude-mistake/` 추가. 형식 / 목적(Claude 실수 재발 방지) 명시.
  - **수정**: `user_instructions/` 정의 명확화 — "**사용자가 터미널에 입력한 지시사항의 시간 누적 기록**"(`user_instruction_handling_sop.md` §3 형식). 결과 요약·리뷰·분석은 여기 금지.
  - Variant 매핑 표에 `mistake/`, `claude_mistake/`, `claude_mistakes/` → `claude-mistake/` 추가.
  - Variant 매핑 표에 `user_instructions/` 안 `*review*`/`*report*`/`*summary*`/`*analysis*` 파일 → `code_review/` 또는 `analysis/` 이전 권고 추가.
- `claude_guideline/audit.sh`:
  - `FOLDER_VARIANTS` 에 mistake 류 3 종 추가.
  - **NEW [request-misclass]** 검출: `docs/user_instructions/` 안 비-요청 파일(review/report/summary/analysis 키워드 매칭) 자동 권고.
  - **NEW [hint]** `docs/claude-mistake/` 부재 시 정보성 권고 (강제 X — 활동 발생 시 생성 권장).

### 트리거

사용자 지적:

> "`docs/request` 는 사용자의 지시사항을 기록하는 것입니다. 사용자가 터미널에 입력하는 지시사항을 정리해서 기록하는 것입니다. … 현재 `ccg-review-2026-05-10.md` 는 결과 요약을 했는데 결과 요약의 목적이 아닙니다."

> "제일 중요한 `claude_mistake` 가 빠졌네요. 이 부분은 매우 중요합니다. 지적사항에 대해서 claude 의 실수를 기록해서 반복 실수를 하지 않도록 하는 것이 목적입니다."

근본 원인:

1. 1.8.0 의 canonical 트리가 `claude_guideline/` 만 인식하고 `claude-mistake/` 등 SSOT 의 다른 자산을 누락. 즉 SSOT 의 책임 분리 분석이 불완전.
2. `documentation.md` 의 매핑 표가 `user_instructions/` 를 "요구사항 · 요청 사항"으로 모호하게 정의해, 결과물 분류 폴더로 오인되기 쉬웠음. SSOT 결함.

본 1.8.1 은 두 결함을 직접 정정. **이 사건 자체는 `claude-mistake/2026-05-11.md` 항목으로 별도 기록 권장** (Claude 가 SSOT 자산 인벤토리를 사전 점검하지 않은 실수).

## 1.8.0 — 2026-05-11

`documentation.md` 에 docs/ canonical 구조 정의 + `audit.sh` 신규 도입. 46 개 docs/ 폴더 일괄 감사 결과를 SSOT 룰로 코드화.

### 변경

- `claude_guideline/documentation.md`:
  - **NEW §폴더 명명 규칙(Canonical)**: 폴더는 언더바 `_` 만 허용, 하이픈/공백/한글 폴더명 금지. 단·복수 일관 룰.
  - **NEW §표준 레이아웃(Canonical Tree)**: repo-root 직속(`manual/`, `api/`) vs `docs/` 책임 분리. `docs/` 필수·옵션 폴더 표.
  - **NEW §ROS2 워크스페이스 특칙**: 패키지별 `src/<pkg>/docs/code_updates/` 가 디폴트, 워크스페이스 `docs/code_updates/` 는 횡단 변경 한정.
  - **NEW §Variant → Canonical 매핑**: `code-review`/`code_reivew` → `code_review`, `issues-fixes`/`issues_fixes` → `issues_and_fixes`, `sw_structure`/`sw-architecture` → `architecture`, `stratedgy` → `strategy` 등 audit.sh 자동 적용 대상.
  - **NEW §변종 차단 룰**: audit.sh 가 검사하는 항목 6 종 명시.
- `claude_guideline/audit.sh`: NEW
  - `bash audit.sh [path...]` 또는 `--batch <file>` 로 일괄 점검.
  - dry-run 전용(파일 이동 없음). variant 폴더 / 단일파일→폴더 승격 / 평탄 .md / 외부 PDF / 한글·공백 폴더명 / 오탈자 / 빈 폴더 / ROS2 패키지 docs 누락 / repo root 필수 파일 누락 검출.
  - exit 0 항상(audit-only).
- `claude_guideline/install.sh`:
  - FILES 배열에 `manual.md`, `ros2.md`, `tech_debt.md`, `iteration_anti_pattern.md`, `skill_update.md`, `user_instruction_handling_sop.md`, `audit.sh` 추가(기존 누락 해소).
  - 설치 후 안내에 `audit.sh` 사용법 추가.
- `claude_guideline/update.sh`:
  - FILES 배열 단일화(install.sh 와 동일). manual.md/ros2.md/iteration_anti_pattern.md/skill_update.md/user_instruction_handling_sop.md/audit.sh 백업 + 다운로드 포함.

### 트리거

`/team 20` 으로 이 컴퓨터의 46 개 docs/ 폴더(직속 하위에 docs/ 가 있는 모든 사용자 프로젝트) 일괄 감사 결과:

- 공통 폴더명 변종 다수(`code-review`/`code_reivew`/`issues-fixes`/`issues_fixes`/`sw-architecture`/`stratedgy`)
- 평탄 `*_code_updates.md` 20+ 개(T-Robot_nav_ros2_ws)
- docs/ 안 외부 벤더 PDF(TM_Robot/Hailo/parking_robot) — root `manual/` 분리 필요
- 한글 폴더명·공백 포함 폴더명(`GLIM-slam only/`, `학습_및_DFC_가이드` 류)
- ROS2 패키지에 docs/ 누락 다수

세부 감사 결과: `Project/claude_code/docs/projects_analysis/README.md` 및 개별 46 보고서.

### 호환성

minor bump (1.7.0 → 1.8.0). 기존 설치 repo 는 `bash docs/claude_guideline/update.sh` 로 새 파일(`audit.sh`, `manual.md` 등) 다운로드 가능. 기존 폴더 구조 강제 마이그레이션 없음(`audit.sh` 는 dry-run 권고만).

## 1.7.0 — 2026-05-09

`user_instruction_handling_sop.md` 신규 — 사용자 지시사항 처리 9 단계 SOP 도입.

### 변경

- `claude_guideline/user_instruction_handling_sop.md`: NEW
  - 9 단계 (지시 명확화 → requirements 기록 → 기존 자료 검색 → SSOT 룰 식별 → 사전 승인 → 실행 → 검증 → 결론 갱신 → 결과 보고)
  - 각 단계 ✓ 체크 강제 (skip 시 거짓 단정 / 중복 작업 / 미승인 변경 위험)
  - SOP 위반 시 영향 사례 (Step 2/3/4/5/7 skip 패턴, 익명화)
  - Step 5 사전 승인 트리거 표 = 신 `github.md` ("기록" 4단계 자동) 와 정합 (외부 SSOT 는 push 직전 사용자 명시 확인)
- `claude_guideline/README.md`: SOP 진입 항목 ★★ (표 최상단) 추가

### 트리거

임베디드 분석 세션에서 발견된 5 패턴:
- requirements 기록 skip → 추후 이력 추적 불가
- 기존 자료 검색 skip → 외부 spec 이 `manual/` 에 있는데 다시 다운로드 시도
- 적용 룰 식별 skip → 거짓 단정 발생 (manual.md v1.6.0 트리거 사례 재현)
- 사전 승인 판단 skip → 코드 / 큰 변경 미승인 진행
- 검증 skip → 일괄 정정 (`replace_all`) 후 verbose 결과

본 SOP 는 위 5 패턴 모두 차단. `manual.md` / `iteration_anti_pattern.md` / `github.md` 와 결합 적용.

## 1.6.2 — 2026-05-08

`manual.md` §3 강화 — 역방향 비약 경고 + DataSheet vs User Manual 분리 추가.

### 변경

- `claude_guideline/manual.md`:
  - **NEW §3.1 역방향 비약 경고**: datasheet → 운영점 해석 시 다음 비약 금지
    - "TYP = 권장값" 비약 (datasheet 명시 없으면 단정 X)
    - "Min/Max 안 = 무조건 OK" 비약 (footnote 측정 조건 충족 필요)
    - "SDK 매크로 수정값 = TYP 일치 = 합리적" 비약 (commit 증거 필요)
  - **NEW §3.2 DataSheet vs User Manual / Family Manual 분리**: pinout/전기 spec vs register-level/SR/DMA 토폴로지. 분석 대상에 따라 추가 PDF 필요. `manual/` 에 두 종류 모두 보관 권장.

### 트리거

20명 매뉴얼 근거 감사팀 (HFPDC ADC 프로젝트, 10 문서 × 2 워커) 의 발견:
- "iLLD 매크로 수정 = datasheet TYP 와 일치 = 합리적" 단정에서 "TYP=권장" 역방향 비약 식별 (m-a, m-b)
- DMA channelId / EVADC SR / GTM ADCTRIG mux 검증이 DataSheet 만으로 불가, User Manual/Family Manual 추가 필요 (w03-a, w03-b, w04-a, w04-b, w05-a, w05-b)
- 측정 조건 (postcalibration, ENRMS, ripple, footnote 2/3) 미인용 시 INL/DNL/TUE 단정 ⓦ 격하 권고 (m-a, m-b, w06-b)

본 패치로 manual.md 가 위 패턴 모두 명시적으로 경고. 다른 프로젝트도 동일 비약 회피 가능.

## 1.6.1 — 2026-05-08

`manual.md` + `skills/manual-first.md` 일반화 — 1.6.0 에서 특정 프로젝트 (HFPDC) 의 칩 / SDK 고유명 (예: AURIX TC38x, fADCI, IfxEvadc 등) 이 SSOT 본문에 노출되어 있던 문제를 정정. SSOT 는 프로젝트 중립이어야 한다.

### 변경

- `claude_guideline/manual.md`:
  - §3 source 분리 예시: 벤더 고유명 → "벤더 SDK / 드라이버 매크로", "`<PERIPHERAL>_<PARAM>_MAX`" 형태 일반 표현
  - §11 다운로드 절차 grep 예시: 칩 고유 파라미터명 → `<parameter_name>` 형태
  - §12 위반 사례: HFPDC 고유명 (AURIX, TC38x, fADCI, IfxEvadc) 모두 제거 → "어느 벤더 SDK 의 `<PARAMETER>_MAX` 매크로" 형태로 패턴만 보존. 상세 사례는 "프로젝트별 case study 파일" 로 outsource
- `skills/manual-first.md`:
  - 키워드 list 에서 칩 고유 파라미터 (`fADCI`, `fSPB`) 제거, 일반 spec 카테고리 (`INL`, `DNL`, `ENRMS`, 외부 표준 ISO/IEEE/UL/UN, 단위 등) 로 대체
  - 도입 사례 섹션 추상화 (HFPDC 명 제거)
- 기존 SSOT 5 섹션 (보관 위치 / 인용 / 추정금지 / 라이선스 / 누락처리) + 신규 6 섹션 (체크리스트 / 등급 / 다운로드 절차 / 변경 절차) 모두 보존, 본 패치는 표현 일반화만

### 트리거

1.6.0 push 직후 사용자 정정: "일반화를 해야지 특정 프로젝트용이면 안됨". SSOT 의 재사용성 보장을 위해 즉시 일반화 패치.

## 1.6.0 — 2026-05-08

`manual.md` 강화 + `skills/manual-first.md` 신규 등록 — datasheet 미참조 거짓 단정 누적 방지.

### `claude_guideline/manual.md` 강화 (42 줄 → 117 줄)

- **NEW §3 source 분리**: `iLLD/SDK 매크로 ≠ silicon datasheet spec` — 본 룰의 핵심 원칙
- **NEW §7 작업 전 체크리스트** (Pre-Work)
- **NEW §8 작업 중 체크리스트** (In-Progress)
- **NEW §9 작업 후 체크리스트** (Post-Work)
- **NEW §10 검증 등급** (✓ 직접 / ⓦ 보고만 / ⚠ 추론) 강제 표기
- **NEW §11 Datasheet 다운로드 표준 절차** (curl + pdftotext + grep)
- **NEW §12 본 룰 위반 시 영향 (실제 사례)** — HFPDC 2026-05-08 ADC 분석 사례
- 기존 SSOT 5 섹션 (보관 위치 / 인용 규칙 / 추정 금지·실측 검증 / 라이선스 / 누락 처리) 모두 보존 + 인용 형식 강제 (`[문서명 v버전, Table N, page P](경로)`)

### `skills/manual-first.md` 신규

- 키워드 (datasheet, 데이터시트, spec, 사양, INL, DNL, AEC-Q100, Operation Conditions, Electrical Characteristics, MHz, fADCI, fSPB, 위반, 초과, non-compliance, violation) 자동 트리거
- 핵심 3 줄 요약 + 작업 전/중/후 체크리스트 + 다운로드 절차 + 도입 사례
- SSOT 룰 ([`manual.md`](manual.md)) 진입점 역할

### 트리거 사건

HFPDC ADC 흐름도 분석 세션 (2026-05-08) — lead 가 iLLD `IFXEVADC_ANALOG_FREQUENCY_MAX = 20MHz` 매크로를 silicon datasheet spec 으로 비약 → "fADCI 33MHz가 datasheet 위반" 거짓 단정 → AEC-Q100 위반 / INL·DNL 무보증 등 downstream 거짓 결론 누적 → 검증팀 8명 launch 의 부분 전제 오염 → 다중 정정 라운드 (v2 → v2.2 → v2.3) 발생, 토큰 / 시간 낭비. 사용자가 datasheet PDF 직접 다운로드 후 검증: fADCI = 16/40/53.33 MHz Min/Typ/Max @ 5V VDDM (TC38x DataSheet v1.2, Table 3-21, page 316) → 33MHz = TYP 안쪽 정상. 본 룰 강화 + 스킬 등록으로 재발 방지.

## 1.5.0 — 2026-05-07

- `iteration_anti_pattern.md` 신규 추가 — Iteration 반복 수정 방지 원칙
  - 핵심 원칙: 1 회 정공법 우선 (2 회 이상 부분 수정 시 전체 재작성 전환)
  - 4 규칙: 기존 자료 우선 조사 / 모호한 단어 추측 금지 / 정정 시 임의 추가분 전체 감사 / 단일 파일도 구조 사전 승인
  - Iteration Loop 탈출 규칙 (2 회 반복 시 작성 중지 + 단어 재정의 질문 + 전체 재작성)
  - 본 규칙은 `coding.md` 및 `workflow.md` 를 강화하며 충돌 시 본 규칙 우선
- `README.md`: 진입점 표에 `iteration_anti_pattern.md` 행 추가
- 트리거 사건: ONE_LINERS.md 5 회 반복 수정 — `claude-mistake/2026-05-07.md` 참조

## 1.4.1 — 2026-05-07

표현 수정 — `skills.md` 를 `skill_update.md` 로 rename. "스킬 목록" 으로 오해할 여지를 줄이고 "스킬 갱신·등록 절차" 의미를 명시화. 내용은 동일.

- `skills.md` → `skill_update.md`
- `README.md`, `templates/CLAUDE.md.template` 의 링크 갱신

## 1.4.0 — 2026-05-07

신규 워크스페이스 자산(스킬 / hook / 가이드라인 / 템플릿)이 SSOT 스킬 레포에 누락되지 않도록 등록 절차를 메타 규칙으로 표준화.

### 추가된 파일

- `skills.md` 신규 추가 — 스킬 / 자동화 자산 SSOT 등록 규칙 (적용 대상 5 종, 등록 절차 6 단계, 워크스페이스↔SSOT 우선순위, 비공개 자산 처리, deprecate 정책)

### 보강된 파일

- `README.md`: 진입점 표에 `skills.md` 행 추가
- `templates/CLAUDE.md.template`: 메타 규칙 진입 링크에 `skills.md` 행 추가

### 신규 정책 요약

- 워크스페이스에서 만든 신규 스킬 / hook / 가이드라인 보강 / sub-agent / 도메인 템플릿은 SSOT 레포(`kuks_claude_setup`)에 등록한다.
- 등록 전 워크스페이스 검증 필수, 비공개 / 환경 의존 부분은 placeholder 또는 `local/` 처리.
- SSOT 가 단일 근원이며 워크스페이스 측 직접 수정 후 SSOT 갱신을 잊는 패턴 금지.

## 1.3.0 — 2026-05-05

ROS2 + 임베디드 + 모듈 CLAUDE.md override 계층을 가진 워크스페이스에서 운영하면서 발견된 규칙 보강. FITO AMR ROS2 워크스페이스 배포에서 검증된 변경분 contributions.

### 추가된 파일

- `ros2.md` 신규 추가 — ROS2 + 임베디드 결합 환경의 도메인 SSOT (빌드 / src 원본 / COLCON_IGNORE / vendored read-only / 시리얼 함정 / 패키지 종류별 주의)
- `manual.md` 신규 추가 — 외부 벤더 매뉴얼·데이터시트 보관·인용·검증 규칙 (추정 금지, 실측 검증, NDA / 라이선스 처리)
- `local/README.md` 신규 추가 — 프로젝트별 비공개 override 폴더 패턴 (하드웨어 IP / 사설 네트워크 / read-only 경로)

### 보강된 파일

- `workflow.md`:
  - 시작 7 항목 / 종료 8 항목 체크리스트로 확장
  - 임베디드 도메인의 펌웨어 다운로드(플래시) 절차 추가 — 포트 점유 확인 → 부트모드 → 플래시 → 실패 시 진단 우선순위
  - 보고 형식을 "매 답변 강제" 에서 "분기 시점에만" 으로 완화 (사전 승인 트리거 / workaround / "기록" / 범위 외 변경)
- `tech_debt.md`:
  - 실시간 / 임베디드 시스템에서 정공법이 특히 중요한 이유 명시
  - 하드웨어 quirk 우회 강화 조항 (벤더 펌웨어 errata / 외부 SDK / 외부 의존성 알려진 버그)
  - TODO 형식 표준화: `// TODO(YYYY-MM-DD): <할 일> [참조]` + 30 일 정리 룰
- `coding.md`:
  - **상수 분리 원칙** 신규 — 의미가 다른 두 값이 우연히 같을 때 한 상수로 합치지 않기 (silent bug 방지)
  - 사전 승인 트리거 5 항목 명시 (패키지 신규 / 외부 인터페이스 변경 / 빌드 시스템 / 하드웨어 인터페이스 / 데이터 스키마)
  - 사전 승인 없이 진행 가능한 변경 명시 (단일 파일 버그 / 파라미터 추가 / 내부 리팩터)
  - 코딩 스타일 섹션 추가 — `.clang-format` 등 저장소 설정 우선, 모듈 CLAUDE.md 가 워크스페이스 가이드보다 우선
  - 보고 양식을 "매 답변" 에서 "변경 분기에서만" 으로 완화
- `github.md`:
  - "기록" 명령에서 push 자동 실행 분리 — push 는 별도 명시 확인 후 수행
  - Read-only 외부 vendored 저장소 가드 섹션 신규 (외부 SDK / 시스템 라이브러리 / COLCON_IGNORE)
  - scope 별 commit 분할 예시 + 자동 staging(`git add -A`/`.`) 금지 명시
  - Push 전 확인에 사설 IP/MAC/endpoint, vendored 원본 저장소 오염 방지 추가
- `documentation.md`:
  - 모듈 CLAUDE.md 가 워크스페이스 가이드를 복제하지 않는다는 SSOT 규칙 명시
  - 도메인 식별자(토픽 / 노드 / 명령 / 레지스터 / 핀맵) 원문 유지 명시
- `README.md`: 진입점 표에 `ros2.md`, `manual.md`, `local/`, `CHANGELOG.md` 행 추가

### 신규 정책 요약

- 모듈 CLAUDE.md override 계층 — 모듈 CLAUDE.md 의 모듈 고유 규칙(핀맵·상수·하드웨어 명령)이 워크스페이스 가이드라인보다 우선
- "기록" → 자동 push 분리
- 보고 형식을 "매 답변 강제" → "분기 시점에만" 으로 완화

## 1.2.0 — 2026-05-01

- `tech_debt.md` 신규 추가 — 기술 부채 방지 원칙 (정공법 우선)
  - 핵심 원칙: 시간이 더 걸리더라도 근본 원인 해결, workaround 금지
  - 우회 사용 시 3가지 조건 (비용/리스크 제시 + 사용자 승인 + 정리 일정 기록) 모두 만족 필수
  - 시간 트레이드오프 보고 의무
  - 임시·진단 코드 정리 / TODO 코멘트 정책 / ADR Open Question 30일 재평가 정책
  - `coding.md` "회피 대안 절대 금지" 와 충돌 시 본 규칙 우선
- `README.md`: 진입점 표에 `tech_debt.md` 행 추가

## 1.1.0 — 2026-04-30

- `github.md`: "커밋·푸쉬는 작업 단위로 분리" 섹션 추가
  - 작업 단위 = 커밋 단위 = push 단위 원칙
  - staged 범위 검증 절차 (`git status --short`, `git diff --cached --name-only`)

## 1.0.0 — 2026-04-30

- 초기 릴리스
- `claude_guideline/` 5 개 파일 추가: `README`, `github`, `coding`, `workflow`, `documentation`
- `install.sh`, `update.sh`, `VERSION`, `CHANGELOG.md` 추가
- `templates/CLAUDE.md.template` 추가 (프로젝트별 CLAUDE.md 골격)

## 정책

- **major** (X.0.0): 기존 규칙과 호환 안 됨, 수동 마이그레이션 필요
- **minor** (X.Y.0): 규칙 추가, 호환됨
- **patch** (X.Y.Z): 오탈자, 표현 수정
