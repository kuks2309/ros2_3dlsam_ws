# Claude 실수 기록

Claude(LLM 어시스턴트) 가 본 프로젝트 작업 중 일으킨 실수와 그 사유, 재발 방지책을 누적 기록한다. 동일한 실수의 반복을 막고, 후속 세션이 학습 자료로 활용한다.

## 책임 경계 (필독)

본 폴더 (`kuks_claude_setup/claude-mistake/`) 는 **일반화 SSOT** — 형식·규칙·운용 가이드만 둔다.

- **여기에 두는 것**: `README.md` (본 파일), 형식 변경 이력
- **여기에 두지 말 것**: 실제 사건 entry (`YYYY-MM-DD.md`), 카테고리 인덱스 (`INDEX.md`), 프로젝트별 closure 데이터
- **사건 entry 작성 위치**: 각 다운스트림 프로젝트의 `docs/claude-mistake/YYYY-MM-DD.md`. 본 SSOT 폴더에 사건을 작성하면 다른 프로젝트에 무관한 사건이 노출되는 책임 경계 침범.

## 파일 구조

- 한 사건 = 한 항목, 시간순 누적 (최신 위)
- 권장: 날짜별 파일 `YYYY-MM-DD.md` (같은 날 여러 건은 한 파일에 모음)
- 사건이 큰 경우: 별도 파일 `YYYY-MM-DD_<짧은제목>.md`

## 항목 형식

```markdown
## YYYY-MM-DD HH:MM (KST) — <짧은 제목>

### 무엇을 했는가
Claude 가 시도한 행동 / 작성한 내용을 객관적으로 기술.

### 무엇이 잘못이었나
어떤 결과가 발생했는지, 어떤 규칙·의도와 어긋났는지.

### 사용자 지적
사용자가 어떻게 교정했는지 (인용 또는 요약).

### 사유 / 가설
Claude 가 왜 그 실수를 했는지 (정보 부족, 잘못된 가정, 컨텍스트 누락, 매뉴얼 오독 등).

### 재발 방지
같은 실수를 막기 위해 무엇을 바꿔야 하는지. 가능하면 가이드라인·메모리·체크리스트의 어느 항목에 반영했는지 링크.
```

## 사용 규칙

- 사용자가 직접 지적한 실수는 그 즉시 본 폴더에 기록한다.
- 사용자가 지적하지 않았더라도 Claude 가 스스로 인지한 실수도 기록할 수 있다.
- 기록 후, 가이드라인 / 메모리 / 체크리스트에 재발 방지 규칙을 반영한다 (단순 기록만으로는 학습이 닫히지 않음).
- 민감한 컨텍스트 (개인 정보, 운영 비밀) 가 포함되는 실수는 기록 위치를 `docs/claude_guideline/local/` (gitignore) 로 옮긴다.

## Closure 규칙 (학습 루프 닫기)

각 entry 는 다음 조건을 모두 만족할 때만 "closed" 로 간주한다. 미만족 entry 는 다운스트림 `docs/claude-mistake/INDEX.md` §미해결 항목 표에 등재 (INDEX.md 운용은 다운스트림 선택 사항 — 사건 누적 시 권장).

- **반영 자산 명시 1개 이상** — `### 재발 방지` 절의 "본 가이드라인 적용" 목록에 실제로 갱신된 파일 (가이드라인·audit 룰·hook·메모리 등) 의 링크가 1개 이상 있어야 한다.
- **TBD 금지** — 반영 자산이 "TBD", "추후", "후보" 로 끝나면 open 상태. 동일 세션 또는 7일 이내 closure 의무.
- **카테고리 부착** — entry 생성 시 다운스트림 `docs/claude-mistake/INDEX.md` §카테고리 × 사건 매트릭스 에 한 행 추가 (INDEX 운용 시).
- **미해결 책임 owner 명시** — open 상태 entry 는 누가 (Claude / 사용자 / 자동화 룰) closure 책임을 지는지 한 줄로 명시.

## 기존 실수 검토 시점

- **세션 시작 (자동)** — SessionStart hook 이 다운스트림 `docs/claude-mistake/INDEX.md` §메타 패턴 + §미해결 항목 두 절을 자동 주입한다 (hook 미설치 다운스트림은 수동 `head -50 docs/claude-mistake/INDEX.md`). 본 SSOT 에 INDEX 가 없는 것이 정상.
- **작업 시작 전 (수동)** — 동일 영역 / 모듈에서 기존 실수가 있었는지 다운스트림 `docs/claude-mistake/` 폴더를 빠르게 훑는다.
- **사용자 정정 직후** — 같은 세션의 다음 entry / 작업 전 정정한 카테고리의 `### 재발 방지` 절을 재독한다 (카테고리 학습 미적용 차단).

## 설치 방법

본 컨벤션은 단일 README 만으로 운용된다. 프로젝트 루트에서 다음을 실행하면 `docs/claude-mistake/` 가 생성되고 본 README 가 배치된다.

```bash
mkdir -p docs/claude-mistake
curl -fsSL https://raw.githubusercontent.com/kuks2309/kuks_claude_setup/master/claude-mistake/README.md \
  -o docs/claude-mistake/README.md
```

설치 후, 프로젝트 루트의 `CLAUDE.md` "도메인 문서 SSOT" 표에 다음 한 줄을 추가한다.

```markdown
| Claude 실수 기록 (재발 방지) | [docs/claude-mistake/README.md](docs/claude-mistake/README.md) |
```

**SSOT 에는 본 README 만**. 사건 entry (`YYYY-MM-DD.md`) 와 카테고리 인덱스 (`INDEX.md`) 는 다운스트림 `docs/claude-mistake/` 에서만 운용. 본 SSOT 폴더에 사건이나 INDEX 가 들어가는 것은 책임 경계 위반.
