# GitHub 워크플로

Git / GitHub 작업 규칙의 단일 근원.

## "기록" 명령 처리

자연어 트리거 "기록" / "기록해" 는 본 절의 4단계 단축어 워크플로다. 슬래시 명령 `/worklog` 와는 **별개 트리거** 이며, 자연어 "기록" 이 `docs/worklog/` 를 자동 갱신하지 않는다 ([../skills/worklog.md](../skills/worklog.md) 참조).

(본 단축어는 CLAUDE.md "사용자 지시만 수행" 룰의 예외가 아니라, 사용자 위임에 의한 표준 워크플로 — line 14 근거.)

사용자가 "기록" 또는 "기록해" 등의 지시를 내리면 다음 4단계를 모두 수행한다.

1. 관련 docs (`docs/user_instructions/user_instructions.md`, `docs/issues_and_fixes/issues_and_fixes.md`) 에 SSOT (Single Source of Truth / 단일 근원) 형식대로 entry 추가 (단, `docs/worklog/` 는 본 단축어가 갱신하지 않으며 `/worklog` 별도 호출 필요)
2. 변경 파일 staging (`git add <명시 파일>`)
3. 커밋 생성 (`git commit`, 작업 단위별 1 커밋). 본문에 동일 세션 worklog 가 있으면 `Worklog: docs/worklog/YYYY/MM/YYYY-MM-DD.md` 한 줄 링크로 본문 prose 를 대체 — worklog 가 본문 SSOT, 커밋 메시지는 요약 (§커밋 메시지 절 참조)
4. push (`git push` 현재 추적 분기로). 본 워크스페이스 origin 직접 push 모델이며 PR 워크플로 미사용 — §브랜치 / PR 정책 절 참조

**근거**: 사용자 명시 지시 (2026-05-08, 2026-05-09 재확인) — "기록" 은 단축어 워크플로우이며 커밋 + 푸쉬 모두 포함.

예외:

- "커밋만" / "푸쉬만" 처럼 단계 한정 시 그 범위만 수행 (단축어 적용 X).
- 파괴적 명령 (`git push --force`, `git reset --hard` 등) 은 본 단축어 적용 외, 사용자 명시 승인 후에만 실행.
- 외부 vendored / SSOT 클론의 원격 push 는 본 단축어 적용 대상이나, push 직전 사용자 명시 확인 (§Push 전 확인 절 + [user_instruction_handling_sop.md](user_instruction_handling_sop.md) Step 5 외부 SSOT 트리거).

## 작업 단위 = 커밋 단위 = push 단위

서로 다른 작업의 변경을 한 커밋/푸쉬에 섞지 않는다.

- `git add` 직전·직후 `git status --short` 로 사전 staged 된 무관 파일 확인.
- 무관한 파일이 staged 면 `git restore --staged <file>` 로 분리.
- commit 직전 `git diff --cached --name-only` 로 staged 범위 마지막 검증.

다중 scope 가 누적된 dirty tree 는 scope 별 commit 으로 분할한다.

```bash
# scope 별 분할 예시
git add <scope-A 경로>
git commit -m "<type>(<scope-A>): ..."
git add <scope-B 경로>
git commit -m "<type>(<scope-B>): ..."
```

자동 staging(`git add -A`, `git add .`) 사용 금지. 항상 명시 경로로 add 한다.

## Read-only 외부 vendored 저장소 가드

다음 유형의 경로는 외부 공식 저장소에서 가져온 vendored 코드. **수정 금지, config / launch overlay 만 변경 허용**.

- 외부 SDK / 드라이버 (예: 센서 공식 드라이버, 모터 SDK)
- 외부 ROS 패키지 fork
- 시스템 설치 라이브러리 (`/usr/local/lib` 등)
- 의도적으로 빌드 제외된 레거시 패키지(`COLCON_IGNORE` 등)

수정이 불가피한 경우 wrapper 패키지 또는 launch / config overlay 로 처리하고, 변경 사유를 commit 메시지 본문에 명시한다.

프로젝트별 구체 경로 목록은 워크스페이스 루트 `CLAUDE.md` 또는 [`local/`](local/) 의 override 문서에서 정의한다.

## 일반 Git 작업 규칙

- 사용자 명시 요청 없이 commit/push 수행 금지. "기록" / "커밋" / "푸쉬" 자연어 단축어는 예외 (§"기록" 명령 처리 참조). 단 외부 vendored / SSOT 클론 원격 push 는 단축어 적용 대상이라도 push 직전 사용자 명시 확인.
- 코드 수정 후에는 staging 까지만 수행하고 commit 은 사용자 지시("기록" 또는 "커밋")로 진행.
- 파괴적 명령(`git push --force`, `git reset --hard`, 브랜치 삭제, `git clean -f` 등)은 사용자 명시 승인 후에만 실행.
- 외부 vendored 저장소(read-only 가드 목록)에는 절대 push 하지 않는다.

## 브랜치 / PR 정책

본 워크스페이스 (kuks_claude_setup 자산 큐레이션) 는 **현재 추적 분기 직접 push 모델** 이며, PR / Pull Request / feature branch / merge 워크플로를 사용하지 않는다.

- 모든 변경은 `master` (또는 `git status` 가 보여주는 현재 추적 분기) 에 직접 commit + push.
- 브랜치 네이밍·feature branch 분기·PR 생성·리뷰어 지정·merge 정책 규칙 없음 — 본 워크스페이스 범위 외.
- PR 워크플로 도입은 별도 사용자 명시 승인 후 본 절 갱신.

**근거**: 본 SSOT 의 단축어 (§"기록" 명령 처리) 가 `git push <추적 분기>` 직접 push 를 명시하며 (line 12), 본 저장소의 git 히스토리에 PR / merge commit 패턴 부재 → 직접 push 모델이 현행 합의.

## 커밋 메시지

- 권장 형식: `<type>(<scope>): <subject>`
- 타입 예: `feat`, `fix`, `docs`, `refactor`, `test`, `chore`, `build`
- 본문에 변경 사유와 관련 이슈 번호(있다면) 명시 권장.
- 형식 강제는 아니나 자동 changelog 생성에 유용.
- **콘텐츠 SSOT**: 동일 세션 worklog 가 존재하면 `Worklog: docs/worklog/YYYY/MM/YYYY-MM-DD.md` 한 줄 링크로 본문 prose 를 대체 가능. SSOT 는 worklog 파일 (§"배경 / 변경 / 결정" 본문 보유), 커밋 메시지는 요약 ([../skills/worklog.md](../skills/worklog.md) §일별 템플릿 참조).
- worklog 만 별도 정리하는 커밋은 `docs(worklog): YYYY-MM-DD 정리` 로 분리. 일반 task 커밋의 staging 에 같은 세션 worklog 변경분이 포함되어도 무방하다 ("작업 단위 = 커밋 단위" 의 동일 task 범위로 간주).

## Push 전 확인

- 변경 파일이 사용자 요청 범위 내인지 재확인
- 비밀 정보(`.env`, 키, 토큰, 인증서, 사설 IP/MAC, 운영 환경 endpoint)가 포함되지 않았는지 확인
- 대상 저장소가 의도한 저장소인지 (외부 vendored 원본 저장소가 아닌 본 워크스페이스 origin 인지) 확인
- read-only 가드 목록의 파일이 staged 되어 있지 않은지 확인
