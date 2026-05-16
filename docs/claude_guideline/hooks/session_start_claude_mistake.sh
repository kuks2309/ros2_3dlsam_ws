#!/usr/bin/env bash
# session_start_claude_mistake.sh — SessionStart hook
#
# 목적: 매 세션 시작 시 docs/claude-mistake/INDEX.md 의
#   §메타 패턴 + §미해결 항목 두 절을 stdout 에 출력하여
#   Claude (LLM) 가 자동으로 학습 컨텍스트로 받도록 한다.
#
# 설치: ~/.claude/settings.json 의 hooks.SessionStart 에 등록
#   {
#     "hooks": {
#       "SessionStart": [
#         { "type": "command",
#           "command": "bash docs/claude_guideline/hooks/session_start_claude_mistake.sh" }
#       ]
#     }
#   }
#
# 출력 형식: claude-mistake INDEX 의 핵심 절을 system-reminder 형태로 wrap.
# stdout 에 쓴 내용이 SessionStart 컨텍스트로 주입된다.

set -u

# 프로젝트 루트 결정 — git 루트 우선, 없으면 PWD
if git_root=$(git rev-parse --show-toplevel 2>/dev/null); then
  ROOT="$git_root"
else
  ROOT="$PWD"
fi

# 후보 위치 우선순위:
#  1. 다운스트림 설치 위치: $ROOT/docs/claude-mistake/INDEX.md
#  2. SSOT 본판 (워크스페이스 자체가 SSOT 인 경우): $ROOT/claude-mistake/INDEX.md
#  3. SSOT 가 워크스페이스 안 sub-repo 인 경우: $ROOT/kuks_claude_setup/claude-mistake/INDEX.md
#  4. hook 스크립트 기준 상대 경로 (hook 이 docs/claude_guideline/hooks/ 안에 있는 경우)
INDEX=""
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
for candidate in \
  "$ROOT/docs/claude-mistake/INDEX.md" \
  "$ROOT/claude-mistake/INDEX.md" \
  "$ROOT/kuks_claude_setup/claude-mistake/INDEX.md" \
  "$SCRIPT_DIR/../../claude-mistake/INDEX.md" \
  "$SCRIPT_DIR/../../../claude-mistake/INDEX.md" \
; do
  if [ -f "$candidate" ]; then
    INDEX="$candidate"
    break
  fi
done

if [ -z "$INDEX" ]; then
  exit 0  # INDEX 없으면 조용히 종료 (다운스트림에 미설치 가능)
fi

cat <<EOF
[claude-mistake] 매 세션 시작 컨텍스트 — 동일 카테고리 재발 차단용. 작업 시작 전 본 절 1회 검토 의무.

EOF

# §메타 패턴 + §미해결 항목 두 절만 추출 (start: ## 메타 패턴, end: 다음 ## 헤딩 또는 §운용 규칙)
awk '
  /^## 메타 패턴/        { capture=1 }
  /^## 카테고리/         { capture=0 }
  /^## 미해결 항목/      { capture=1 }
  /^## 운용 규칙/        { capture=0 }
  capture { print }
' "$INDEX"

cat <<EOF

---
참조: $INDEX
EOF

exit 0
