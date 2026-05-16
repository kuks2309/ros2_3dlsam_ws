# 스킬 등록 규칙

본 워크스페이스에서 신규 스킬 / 자동화 자산을 만들거나 기존 자산을 보강하면, SSOT (Single Source of Truth / 단일 근원) 스킬 레포 [`kuks_claude_setup`](https://github.com/kuks2309/kuks_claude_setup) 에 등록해 다른 프로젝트가 재사용할 수 있게 한다. 본 규칙은 그 절차의 단일 근원.

## 적용 대상 ("스킬" 의 범위)

다음 자산이 본 규칙의 대상이다.

| 종류 | 워크스페이스 위치 (예) | SSOT 레포 위치 |
| --- | --- | --- |
| Slash command / skill 정의 | `.claude/commands/`, `.claude/skills/` | `skills/<name>.md` |
| Hook 스크립트 | `.claude/hooks/<name>.sh` | `tools/hooks/<name>.sh` (또는 `skills/<name>/`) |
| 가이드라인 보강 | `kuks_claude_setup/claude_guideline/<file>.md` (authoring SSOT) | 동일 — mirror `docs/claude_guideline/` 는 `update.sh` 산출 |
| 커스텀 sub-agent | `.claude/agents/<name>.md` | `agents/<name>.md` |
| 도메인 템플릿 (이슈 / 실수 / 사용자 지시 README) | `docs/<domain>/README.md` | `<domain>/README.md` |
| Claude Skills bundle (multi-file) | `.claude/skills/<name>/` | `<name>/SKILL.md + scripts/ + references/ + ...` |

위 카테고리에 명확히 속하지 않는 신규 자산은 SSOT 레포의 README 에서 적합한 폴더를 결정하거나, 새 폴더를 사용자 승인 후 추가한다 ([coding.md](coding.md) "사전 승인 트리거 — 새 패키지 / 모듈 신규 생성").

## 자동화 정책 (자동 활성 vs 명시 호출)

본 워크스페이스의 모든 스킬·에이전트·가이드라인 자산은 다음 4 표현 중 하나의 트리거 동작만 가질 수 있다. 루트 `CLAUDE.md` 핵심 원칙 #1 ("사용자가 지시한 사항만 수행") / #2 ("임의로 기능을 추가하거나 변경하지 않는다") 와의 합치 의무.

| 표현 | 의미 | 핵심 원칙 충돌 |
|------|------|----------------|
| **명시 호출** | 사용자가 `/<skill>` 또는 명시 자연어 ("이 스킬 적용해줘") 로 호출. 호출 전 자산이 작동하지 않음 | 합치 |
| **권고 트리거 (advisory trigger)** | 키워드 / 컨텍스트 매칭 시 Claude 가 "이 스킬을 참조할까요?" 1줄 제안. 사용자 승인 후 작동 | 합치 |
| **자동 인용 (passive cite)** | 키워드 등장 시 본 SSOT 룰 *문서* 만 인용 (어떤 작업도 트리거하지 않음) | 합치 |
| **자동 활성 (auto-activate)** ← **금지 표현** | 키워드 / 이벤트 감지 즉시 *작업 수행* (다운로드 / 파일 생성 / `git add` / memory 기록 포함) | 위반 |

### 금지 표현

자산 본문에서 다음 표현은 모두 핵심 원칙 #1·#2 위반이므로 사용 금지:

- "키워드 감지 시 자동 활성", "키워드 (...) 등장 시 자동 활성화"
- "MANDATORY", "반드시 ... 업데이트", "사용자 개입 없이 완전 자동"
- "다음 순서" (단정형), "절대 사용 금지" (일률 적용 헤더)

→ "사용자가 `기록` 또는 `기록해` 라고 지시하면" / "본 SSOT 가 권장하는 순서 (사용자 합의 후 적용)" / "자동 인용" 으로 약화 표기.

### 자동 트리거 허용 화이트리스트 (3 종)

본 정책의 좁은 예외:

1. **자동 인용 (passive cite)** — 작업을 트리거하지 않고 문서만 참조.
2. **PostToolUse 로컬 jsonl 관찰** — 머신 로컬 한정 도구 사용 로그 누적 (단, 다음 세션의 Claude 행동을 변경하는 영속 memory 갱신은 별도 사용자 opt-in 필요).
3. **idempotent 디렉토리 생성** — `mkdir -p` 같이 기존 데이터를 덮어쓰지 않고 사용자가 곧 이어서 직접 파일을 쓸 디렉토리에만 한정. 신규 `.md` 파일 자동 생성은 본 예외 밖.

### 등록 안 해도 되는 경우 — 머신 로컬 학습 캐시

다음 자산은 SSOT 레포 추적 대상이 아니며 본 §등록 절차를 적용하지 않는다:

- `project-autolearn` 의 `.omc/patterns.md`, `.omc/observations/`
- `~/.claude/projects/*/memory/learned_patterns.md` (auto memory)
- 기타 머신 로컬 학습 캐시 (`.gitignore` 강제)

audit.sh `[auto-trigger-vocab]` 룰이 본 §의 금지 표현을 검출.

## 등록 절차

```text
[워크스페이스 검증]
    ↓
[SSOT 레포로 복사 / 갱신]
    ↓
[메타 갱신 (CHANGELOG / VERSION / README 진입점)]
    ↓
[SSOT 레포 commit + push]
    ↓
[워크스페이스 측 sync (필요 시)]
```

1. **워크스페이스에서 검증.** 자산이 의도대로 동작함을 실제로 확인한 뒤에만 등록한다 (검증 안 된 자산을 SSOT 에 올리지 않는다). 단, 가이드라인 자산은 authoring SSOT (`kuks_claude_setup/claude_guideline/`) 에서 직접 검증한다 — `docs/claude_guideline/` mirror 를 임시 수정해서 검증하면 documentation.md §SSOT 경로 규칙 위반 ([documentation.md](documentation.md) §SSOT 경로 참조).
2. **SSOT 레포 clone / pull.** 로컬 사본을 최신화한다.
3. **자산 복사.** 위 표의 "SSOT 레포 위치" 로 옮긴다. 워크스페이스 고유 경로(IP, 사설 endpoint 등) 가 포함돼 있으면 placeholder / 환경변수로 추상화한다 ([local/README.md](local/README.md) 참조).
4. **메타 갱신.**
   - `claude_guideline/` 변경이면: `CHANGELOG.md`, `VERSION` (semver), `README.md` 진입점, `templates/CLAUDE.md.template` 갱신.
   - `skills/`, `agents/`, `tools/hooks/` 등 그 외 폴더면: 해당 폴더의 README 또는 INDEX 가 있으면 갱신.
5. **SSOT 레포 commit + push.** 작업 단위 = 커밋 단위 ([github.md](github.md)). 자동 push 금지 — 사용자 명시 확인 후.
6. **워크스페이스 sync.** 필요 시 워크스페이스의 `docs/claude_guideline/update.sh` 또는 수동 복사로 SSOT → 워크스페이스 방향으로 동기화. SSOT 가 단일 근원이므로 워크스페이스 측 직접 수정 후 SSOT 갱신을 잊는 패턴은 금지.

## 워크스페이스 → SSOT 우선순위

자산이 양쪽에 모두 있을 때 충돌이 발생하면 다음 순서로 처리한다.

1. **SSOT 레포가 단일 근원** — 일반 규칙은 SSOT 가 우선.
2. **워크스페이스 override 가 명시적**일 때만 워크스페이스 우선 — 모듈 CLAUDE.md, [local/](local/) 가 그 예 ([documentation.md](documentation.md) "SSOT 원칙").
3. **SSOT 갱신 누락이 의심되면** SSOT 부터 갱신하고 워크스페이스로 sync.

## 비공개 / 프로젝트 고유 자산 처리

- 사설 IP / NDA 매뉴얼 / 운영 endpoint / 하드웨어 핀맵 등 외부 공개 부적합 자산은 SSOT 레포에 올리지 않는다 ([manual.md](manual.md) "라이선스 / 외부 공개").
- 일반화 가능한 부분만 SSOT 에 올리고, 비공개 부분은 워크스페이스 [local/](local/) 또는 비공개 저장소에 둔다.
- 일반화가 어려우면 SSOT 등록을 보류하고 본 규칙의 적용 대상에서 제외한다.

## 등록 후 책임

- 등록자는 다음 분기까지 SSOT 측 자산이 정상 동작하는지 확인한다 (외부 사용자가 install / update 시 깨지지 않는지).
- 자산이 의존하는 환경 (커맨드, 라이브러리, OS 종류) 을 SSOT 의 README 또는 자산 헤더 주석에 명시한다.
- 30 일 이상 미사용 / 사용처 부재 자산은 deprecate 표시 또는 정리 ([tech_debt.md](tech_debt.md) ADR Open Question 30 일 룰과 동일 정신).

## 등록 안 해도 되는 경우

- 워크스페이스 1 회성 패치 (재사용 가치 없음)
- 디버그용 임시 스크립트
- 사용자 1 인 / 1 머신 한정 설정 ([local/](local/) 에만 둠)

판단이 모호하면 사용자에게 한 줄로 확인 ("이 자산을 SSOT 에 등록할까요?").
