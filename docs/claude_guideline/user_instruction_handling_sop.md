# 사용자 지시사항 처리 표준 운영 절차 (SOP / Standard Operating Procedure)

매번 사용자 지시가 도착하면 다음 9 단계를 **순차 실행** 하고 각 단계마다 **✓ 체크** 후 다음 단계로 진행한다. 단계 skip 또는 ✓ 체크 누락은 거짓 단정 / 중복 작업 / 미승인 변경 등의 위험을 초래한다 (실제 사례 §11 참조).

본 SOP 는 SSOT (Single Source of Truth / 단일 근원) 룰. 변경 시 사용자 승인 필수.

---

## 0. 본 SOP 의 목적 — 의도 부채(Intent Debt) 방지

소프트웨어 부채는 세 층위로 나뉘며, 회복 가능성이 다르다:

| 부채 | 회복 가능성 | 근거 |
|------|-----------|------|
| 기술 부채 | 가능 | 코드가 남아있어 리팩토링 가능 |
| 인지 부채 | 가능 | 지식은 외부 소스로 재학습 가능 |
| **의도 부채** | **불가능** | 사용자 의도가 망실되면 추측 외 복원 수단 없음 |

**의도 부채가 세 부채 중 가장 심각**하며, **사용자 지시의 기록 부재에서 발생**한다. 본 SOP 의 핵심 목적은 사용자 지시사항을 `docs/user_instructions/user_instructions.md` 에 원문 기록하여 **의도 부채를 0 으로 유지**하는 것.

**Step 2 (지시 도착 즉시 기록) + Step 8 (결론·산출물 갱신)** 이 본 SOP 의 중심축이며, 나머지 step (명확화·자료 검색·룰 식별·승인·실행·검증·보고) 은 이 기록이 정확·완결되도록 보조하는 절차다.

기록이 없을 때 발생하는 의도 부채의 증식 경로:
- 사용자 의도가 Claude 의 추측으로 대체되어 사실로 굳어짐
- 후속 작업이 잘못된 의도 위에 누적 → 회수 비용 기하급수 증가
- context window 압축 / 세션 종료 후 원문 망실 → 정정 불가능한 왜곡
- 다음 세션의 Claude 가 무엇을 이어받았는지 모름 → 위임 불가

→ **결과·산출물·분석은 본 파일이 아니라 [worklog/](../../docs/worklog/), [code_review/](../../docs/code_review/) 등 별도 도메인이 책임진다.** user_instructions.md 는 **지시 원문 보존 전용**.

---

## 1. 흐름도 (한눈에)

```
[사용자 지시 도착]
   ↓
[Step 1] 지시 명확화                ────→  ✓ 체크: 작업 범위/의도 명확
   ↓
[Step 2] user_instructions.md 기록        ────→  ✓ 체크: 시간 역순 추가, grep 확인
   ↓
[Step 3] 기존 자료 검색 (5종)        ────→  ✓ 체크: 중복/과거 실수 인지
   ↓
[Step 4] 적용 SSOT 룰 식별           ────→  ✓ 체크: 룰 명시, 충돌 없음
   ↓
[Step 5] 사전 승인 판단              ────→  ✓ 체크: 트리거 시 STOP, ask user
   ↓
[Step 6] 실행 (TodoWrite 추적)       ────→  ✓ 체크: 단계별 완료
   ↓
[Step 7] 검증 (manual.md §9)         ────→  ✓ 체크: 거짓 단정 / 미인용 / 정정 잔존 0
   ↓
[Step 8] worklog 결과 기록         ────→  ✓ 체크: user_instructions.md 외 worklog/분야별 폴더에 기록
   ↓
[Step 9] 1-2줄 결과 보고             ────→  ✓ 체크: 보고 완료
   ↓
[완료]
```

---

## 2. Step 1 — 지시 명확화

- 모호하면 user 에 즉시 1줄 질문 (긴 추측 금지)
- 작업 유형 분류:
  - 분석 (read-only 조사)
  - 수정 (코드 / 문서 변경)
  - 신규 작성 (파일 생성)
  - 검증 (기존 결과 cross-check)
  - 외부 연동 (push / 다운로드 / 외부 도구)
- 작업 범위 1줄 요약 자체 작성 → user 와 일치 여부 즉시 확인 가능

**✓ 체크**: 작업 범위 / 의도 명확. 모호하면 STOP 후 질문.

---

## 3. Step 2 — `docs/user_instructions/user_instructions.md` 기록 (사용자 원문만)

본 프로젝트 루트의 `docs/user_instructions/user_instructions.md` (또는 동등 경로 — 프로젝트별 [`local/`](local/) override 가능).

`docs/user_instructions/` 의 정의([documentation.md](documentation.md) §docs/ 표준 폴더): **사용자가 터미널에 입력한 지시사항의 시간 누적 기록 전용**. 처리 계획·결과 요약·산출물 목록은 본 폴더 금지 — `docs/worklog/` 또는 분야별 폴더(`code_review/`, `analysis/` 등) 책임.

기록 형식 (v1.8.2 정정 — 처리/결론 섹션 제거, 시각 KST = Korea Standard Time / 한국 표준시):
```markdown
## YYYY-MM-DD HH:MM (KST) — <짧은 제목>

### 요청
> "<사용자 원문 인용>"

---
```

**룰**:
- KST 시각, 시간 역순 (최신 위)
- 사용자 원문만 인용 (요약·해석 금지)
- 항목 추가 즉시 (작업 완료 후 일괄 X)
- 동일 요구사항 단순 재확인은 생략 가능
- 비밀번호 / NDA 정보는 마스킹

**✓ 체크**: 항목 추가 완료. `grep "^## " docs/user_instructions/user_instructions.md | head -3` 로 시간 역순 확인. 본 entry 안 `### 처리` / `### 결론` / `### 산출물` 헤딩 부재 확인 (`grep -E "^### (처리|결론|산출물)" docs/user_instructions/user_instructions.md` → 출력 없어야 함).

---

## 4. Step 3 — 기존 자료 검색 (5종)

다음 5개 위치를 **모두 확인** (skip 시 중복 작업 / 과거 실수 재현 위험):

| # | 위치 | 확인 내용 |
|---|------|----------|
| 1 | `docs/<관련 분석명>/` | 기존 분석 결과 / MASTER 문서 |
| 2 | `.omc/research/<관련>/` | 워커 보고 / 임시 분석 |
| 3 | `manual/` | datasheet PDF / 외부 1차 source |
| 4 | `docs/claude_guideline/` | 적용할 SSOT 룰 (manual.md, github.md 등) |
| 5 | `~/.claude/projects/<프로젝트>/memory/` | feedback / 과거 실수 기록 |

**✓ 체크**: 5종 모두 검색 완료. 중복 작업 / 기존 정정 / 과거 실수 인지.

---

## 5. Step 4 — 적용 SSOT 룰 식별

본 프로젝트의 `docs/claude_guideline/` 에서 적용 룰 식별:

| 룰 파일 | 적용 트리거 |
|---------|-----------|
| [manual.md](manual.md) | 외부 spec / datasheet / INL/DNL/AEC-Q100 등 키워드 — auto skill `manual-first` |
| [github.md](github.md) | commit / push / 기록 명령 |
| [workflow.md](workflow.md) | 펌웨어 플래시 등 절차 |
| [documentation.md](documentation.md) | 문서 양식 / 작성 메타 룰 |
| [iteration_anti_pattern.md](iteration_anti_pattern.md) | 정정 라운드 의심 (2회 이상 부분 수정) |
| [skill_update.md](skill_update.md) | 신규 스킬 / 자산 등록 |

**✓ 체크**: 적용 룰 명시 (1개 이상). 룰 간 충돌 없음.

---

## 6. Step 5 — 사전 승인 판단

다음 트리거 해당 시 **STOP, ask user**:

| 트리거 | 사유 |
|--------|------|
| 코드 (`Source/` 또는 동등 코드 디렉토리) 수정 | 사용자 명시 승인 필수 |
| sub-agent 스폰 (3명 이상 또는 큰 비용) | 사용자 승인 |
| 외부 SSOT 원격 push (예: 벤더 SSOT 클론 → 원격) | "기록" 단축어 적용 대상이지만, 외부 SSOT 는 push 직전 사용자 명시 확인 (github.md "Push 전 확인" 절 + 일반 룰 "단 push 는 별도 확인") |
| `.gitignore` 우회 / 빌드 산출물 commit | github.md vendored 보호 |
| 비밀번호 / NDA / 자격증명 노출 가능 | 즉시 STOP |
| 새 폴더 / 모듈 / 패키지 생성 | coding.md 사전 승인 |

**✓ 체크**: 트리거 해당 → STOP. 미해당 → 다음 step.

---

## 7. Step 6 — 실행

- `TodoWrite` 로 sub-step 추적 (3 step 이상 작업 시 강제)
- 각 sub-step 완료마다 todo 갱신 (in_progress → completed)
- 병렬 가능한 작업은 single message multi tool call

**✓ 체크**: 모든 todo completed, 미완 항목 없음.

---

## 8. Step 7 — 검증 ([manual.md §9](manual.md) 사후 체크리스트)

- ✓ 표시 항목 = file:line 또는 datasheet:page 인용 있는가?
- 벤더 SDK / 드라이버 매크로 → silicon datasheet spec 비약 있는가? (있으면 ⚠ 격하)
- 강한 단정어 ("위반"/"초과"/"non-compliance"/"fail") = primary source 첨부?
- 미검증 추론을 ✓ 로 표시한 곳 없는가?
- 정정 잔존 (오류 흔적 본문 embedded) 없는가?
- 정정 이력 (vN → vN+1) 명시?

**✓ 체크**: 검증 통과. 미해결 ⚠ 항목은 사용자에게 명시.

---

## 9. Step 8 — `docs/worklog/` 결과·산출물 기록 (v1.8.2 정정)

`docs/user_instructions/user_instructions.md` 는 사용자 원문 전용([Step 2](#3-step-2--docsrequestrequestsmd-기록-사용자-원문만)). 처리 결과·결론·산출물은 본 단계에서 **별도 폴더** 에 기록한다.

기록 위치 (작업 유형에 따라 택일):

| 작업 유형 | 기록 위치 |
|----------|----------|
| 일반 작업 로그 | `docs/worklog/YYYY-MM-DD.md` (시간 누적, 최신 위) |
| 코드 리뷰 / 평가 | `docs/code_review/<주제>.md` |
| 분석 / 리서치 | `docs/analysis/<주제>.md` |
| 리팩토링 계획·결과 | `docs/refactoring/<주제>.md` |
| 트러블슈팅 | `docs/troubleshooting/<주제>.md` |

기록 형식 (worklog 예시):
```markdown
## YYYY-MM-DD HH:MM (KST) — <짧은 제목>

### 트리거 요청
[user_instructions.md](../user_instructions/user_instructions.md) `YYYY-MM-DD HH:MM` entry 참조.

### 처리
- (수행한 작업 단계 요약)

### 결론 / 산출물
- (변경 파일 / commit hash / push 결과 / 후속 TODO)

---
```

**룰**:
- `docs/user_instructions/user_instructions.md` 의 동일 항목에 `### 처리` / `### 결론` 섹션을 **추가하지 않는다** (v1.8.2 변경 — 이전 버전에서 동일 항목 안 갱신을 명시했으나 `documentation.md` 의 `user_instructions/` 정의와 충돌하여 정정).
- worklog 파일이 처음 만들어진다면 `docs/worklog/README.md` 도 함께 만든다 ([documentation.md](documentation.md) §명명 규칙).
- user_instructions.md 의 entry 와 worklog 의 entry 는 시각·제목으로 매핑되어야 한다 (역방향 grep 가능).

**✓ 체크**: worklog 또는 분야별 폴더에 결과 entry 추가 완료. user_instructions.md 안 `### 처리` / `### 결론` 신규 작성 없음 (`grep -E "^### (처리|결론|산출물)" docs/user_instructions/user_instructions.md` → 출력 없음).

---

## 10. Step 9 — 1-2줄 결과 보고

- 일상 작업: 1-2줄 (workflow.md §보고 형식)
- 다음 시점 추가 명시:
  - 사전 승인 트리거 변경 착수 전
  - 우회 / workaround 사용 결정 전
  - "기록" 명령 처리 시 (commit 범위 보고)
  - 작업 범위 외 변경이 발견된 경우
  - 정정 라운드 / 큰 변경

**✓ 체크**: 보고 완료. 사용자 추가 지시 대기.

---

## 11. SOP 위반 시 영향 (실제 사례)

본 SOP 의 step 을 skip 하거나 ✓ 체크 누락 시 다음 패턴 발생 (SOP 도입 트리거 사례: 임베디드 분석 세션):

| skip 한 step | 발생한 문제 |
|--------------|-------------|
| Step 2 (요구사항 기록) | 추후 이력 추적 불가, 동일 요구 반복 |
| Step 3 (기존 자료 검색) | 외부 spec 이 `manual/` 에 있는데 다시 다운로드 시도 / 워커가 같은 분석 반복 |
| Step 4 (manual-first 룰) | "TYP=권장" 역방향 비약 같은 거짓 단정 (벤더 SDK 매크로 → silicon spec 비약) |
| Step 5 (사전 승인) | 코드 수정 / 큰 변경 사용자 동의 없이 진행 |
| Step 7 (검증) | 일괄 정정 적용 (`replace_all`) 후 verbose 결과 — 함수명 / 도메인 용어 일괄 치환 후 가독성 손상 |

**핵심 원칙**: 각 step 사이의 ✓ 체크는 다음 step 진행 전 명시적 확인. "한 번에 다 처리" 보다 "1 step 1 verify".

상세 case study (정정 라운드 5회 사례) 는 본 SSOT push 대상 외 — 도입 프로젝트 로컬의 정정 이력 문서에 보존.

---

## 12. 큰 결정 / 연구성 작업 보조 절차

일상 지시 처리는 본 SOP §1-§9 9 단계를 따른다. 단, 다음 분류에 해당하는 **큰 결정 / 연구성 작업** 은 Step 3 (5종 자료 검색) 직후 본 §12 절을 적용한다 (구 `user_instruction_analysis.md` v1.8.7 통합):

- 5-stage / 6-차원 / 4-way 결정 절차가 필요한 트레이드오프 분석
- 다중 vendor / 라이브러리 비교
- 외부 표준·인증 (AEC-Q100 / ISO / IEEE / UL) 적용 결정

**잠정 절차 (`research_subroutine.md` SSOT 분리 전 임시):**

1. Preflight — 사용자 결정 범위·시점·산출물 형식 1줄 확인.
2. Need Analysis — 결정에 필요한 자료 카테고리 (manual / spec / 외부 비교 / 실측) 식별.
3. Parallel Search — 카테고리별 자료 동시 수집 (`grep` / `manual/` / 외부 URL).
4. Evaluate — 6-차원 (비용 / 리스크 / 성능 / 유지보수 / 호환성 / 일정) 비교 표.
5. 4-way 결정 보고 — 후보 4 개 또는 그 이하 + 권고 + 사용자 선택.

본 절차는 v0.4 에서 `research_subroutine.md` 로 분리 예정. 분리 시 본 §12 는 외부 링크 1 줄로 축약.

## 13. 변경 절차

본 룰은 SSOT 이므로 변경 시 **사용자 승인 필수**. 변경 후:
1. CLAUDE.md §0 표 갱신 확인
2. README.md 표 / 원본 미반영 목록 동기화 확인
3. Skill 파일 (필요 시) 동기화
4. CHANGELOG / VERSION (semver) 갱신
