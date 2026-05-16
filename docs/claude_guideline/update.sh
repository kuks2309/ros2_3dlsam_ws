#!/usr/bin/env bash
set -euo pipefail

# Claude 작업 지침 업데이트 스크립트
#
# 사용법:
#   bash docs/claude_guideline/update.sh           # 대화식
#   bash docs/claude_guideline/update.sh --auto    # 자동 (patch/minor)

RAW_URL="https://raw.githubusercontent.com/kuks2309/kuks_claude_setup/master/claude_guideline"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

CURRENT_VERSION="0.0.0"
if [ -f "$SCRIPT_DIR/VERSION" ]; then
  CURRENT_VERSION=$(tr -d '\n[:space:]' < "$SCRIPT_DIR/VERSION")
fi

UPSTREAM_VERSION=$(curl -fsSL "$RAW_URL/VERSION" | tr -d '\n[:space:]')

echo "현재 버전:    $CURRENT_VERSION"
echo "Upstream 버전: $UPSTREAM_VERSION"

if [ "$CURRENT_VERSION" = "$UPSTREAM_VERSION" ]; then
  echo "[OK] 이미 최신 버전입니다."
  exit 0
fi

# major 버전 변경 감지 (수동 승인 필수)
CURRENT_MAJOR="${CURRENT_VERSION%%.*}"
UPSTREAM_MAJOR="${UPSTREAM_VERSION%%.*}"

AUTO=false
if [ "${1:-}" = "--auto" ]; then
  AUTO=true
fi

if [ "$CURRENT_MAJOR" != "$UPSTREAM_MAJOR" ]; then
  echo ""
  echo "[!] MAJOR 버전 변경 감지: $CURRENT_VERSION -> $UPSTREAM_VERSION"
  echo "[!] 호환되지 않는 변경일 수 있으니 CHANGELOG 를 먼저 확인하세요:"
  echo "    $RAW_URL/CHANGELOG.md"
  AUTO=false
fi

if [ "$AUTO" = false ]; then
  read -r -p "업데이트하시겠습니까? (y/N) " ans
  if [ "$ans" != "y" ] && [ "$ans" != "Y" ]; then
    echo "취소되었습니다."
    exit 0
  fi
fi

# 백업 + 다운로드 대상 파일 목록 (단일 source of truth)
FILES=(
  "README.md"
  "github.md"
  "coding.md"
  "workflow.md"
  "documentation.md"
  "manual.md"
  "ros2.md"
  "tech_debt.md"
  "iteration_anti_pattern.md"
  "skill_update.md"
  "user_instruction_handling_sop.md"
  "claude_md.md"
  "VERSION"
  "CHANGELOG.md"
  "update.sh"
  "audit.sh"
)

HOOK_FILES=(
  "README.md"
  "session_start_claude_mistake.sh"
)

# 백업 (hooks/ 포함)
BACKUP_DIR="$SCRIPT_DIR/.backup-$(date +%Y%m%d-%H%M%S)"
mkdir -p "$BACKUP_DIR/hooks"
for f in "${FILES[@]}"; do
  if [ -f "$SCRIPT_DIR/$f" ]; then
    cp "$SCRIPT_DIR/$f" "$BACKUP_DIR/$f"
  fi
done
for f in "${HOOK_FILES[@]}"; do
  if [ -f "$SCRIPT_DIR/hooks/$f" ]; then
    cp "$SCRIPT_DIR/hooks/$f" "$BACKUP_DIR/hooks/$f"
  fi
done
# legacy 파일 백업 (request_handling_sop.md → user_instruction_handling_sop.md rename)
if [ -f "$SCRIPT_DIR/request_handling_sop.md" ]; then
  cp "$SCRIPT_DIR/request_handling_sop.md" "$BACKUP_DIR/request_handling_sop.md.legacy"
  rm "$SCRIPT_DIR/request_handling_sop.md"
  echo "[i] legacy request_handling_sop.md 제거 (백업 보존). v1.8.2 부터 user_instruction_handling_sop.md."
fi
echo "[+] 백업 완료: $BACKUP_DIR"

# 새 파일 다운로드 (local/ 은 건드리지 않음)
mkdir -p "$SCRIPT_DIR/hooks"
for f in "${FILES[@]}"; do
  echo "[+] Downloading $f"
  curl -fsSL "$RAW_URL/$f" -o "$SCRIPT_DIR/$f"
done
for f in "${HOOK_FILES[@]}"; do
  echo "[+] Downloading hooks/$f"
  curl -fsSL "$RAW_URL/hooks/$f" -o "$SCRIPT_DIR/hooks/$f"
done

chmod +x "$SCRIPT_DIR/update.sh" "$SCRIPT_DIR/audit.sh" "$SCRIPT_DIR/hooks/session_start_claude_mistake.sh"

# docs/ 표준 폴더 전체 보강 (v1.8.6: install.sh 와 동일 정책, 기존 README 는 덮어쓰지 않음)
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
mkdir -p \
  "$PROJECT_ROOT/docs/architecture" \
  "$PROJECT_ROOT/docs/usage" \
  "$PROJECT_ROOT/docs/issues_and_fixes" \
  "$PROJECT_ROOT/docs/assets" \
  "$PROJECT_ROOT/docs/user_instructions" \
  "$PROJECT_ROOT/docs/worklog" \
  "$PROJECT_ROOT/docs/claude-mistake" \
  "$PROJECT_ROOT/docs/code_review" \
  "$PROJECT_ROOT/docs/refactoring" \
  "$PROJECT_ROOT/docs/analysis" \
  "$PROJECT_ROOT/docs/test" \
  "$PROJECT_ROOT/docs/troubleshooting" \
  "$PROJECT_ROOT/docs/api"

if [ ! -f "$PROJECT_ROOT/docs/claude-mistake/README.md" ]; then
  echo "[+] Downloading claude-mistake/README.md (신규)"
  curl -fsSL "https://raw.githubusercontent.com/kuks2309/kuks_claude_setup/master/claude-mistake/README.md" \
    -o "$PROJECT_ROOT/docs/claude-mistake/README.md"
fi

write_stub() {
  local path="$1"; local title="$2"; local body="$3"
  [ -f "$path" ] && return 0
  printf '# %s\n\n%s\n' "$title" "$body" > "$path"
}

write_stub "$PROJECT_ROOT/docs/architecture/README.md" "구조 · 설계 (architecture)" \
  "프로젝트 / 워크스페이스의 구조·설계 문서. ROS2 워크스페이스는 [documentation.md](../claude_guideline/documentation.md) §ROS2 특칙 참조."

write_stub "$PROJECT_ROOT/docs/usage/README.md" "설치 · 실행 · 튜토리얼 (usage)" \
  "설치 / 실행 / 사용 가이드. 단일 \`.md\` 파일 (\`INSTALLATION.md\`, \`USER_GUIDE.md\` 등) 흡수 대상."

write_stub "$PROJECT_ROOT/docs/issues_and_fixes/README.md" "이슈 · 수정 기록 (issues_and_fixes)" \
  "이슈 발생 · 분석 · 수정 결과를 시간 누적 (최신 위). 형식: \`YYYY-MM-DD.md\`."

write_stub "$PROJECT_ROOT/docs/assets/README.md" "이미지 · 동영상 · 다이어그램 (assets)" \
  "스크린샷, RViz 캡처, 다이어그램 등 바이너리 리소스."

write_stub "$PROJECT_ROOT/docs/user_instructions/README.md" "사용자 지시 기록 (user_instructions)" \
  "사용자가 터미널에 입력한 지시사항의 시간 누적 기록 전용. 형식 SSOT: [user_instruction_handling_sop.md](../claude_guideline/user_instruction_handling_sop.md) §3. 사용자 원문 인용만. 처리 결과는 [worklog/](../worklog/) 책임. 실제 지시는 \`user_instructions.md\` 에 누적."

write_stub "$PROJECT_ROOT/docs/worklog/README.md" "작업 기록 (worklog)" \
  "사용자 지시에 대한 처리 결과·결론·산출물 시간 누적 기록. 형식 SSOT: [user_instruction_handling_sop.md](../claude_guideline/user_instruction_handling_sop.md) §9. 파일: \`YYYY-MM-DD.md\`."

write_stub "$PROJECT_ROOT/docs/code_review/README.md" "코드 리뷰 결과 (code_review)" \
  "코드 리뷰·외부 advisor (Codex/Gemini ccg) 결과 리포트."

write_stub "$PROJECT_ROOT/docs/refactoring/README.md" "리팩토링 계획·결과 (refactoring)" \
  "리팩토링 계획·진행·결과 기록."

write_stub "$PROJECT_ROOT/docs/analysis/README.md" "분석 · 리서치 (analysis)" \
  "분석 보고서, 리서치 결과, phase 리포트."

write_stub "$PROJECT_ROOT/docs/test/README.md" "테스트 시나리오 · 리포트 (test)" \
  "테스트 계획·시나리오·실행 리포트."

write_stub "$PROJECT_ROOT/docs/troubleshooting/README.md" "문제 해결 (troubleshooting)" \
  "운영 / 디버깅 트러블슈팅 가이드."

write_stub "$PROJECT_ROOT/docs/api/README.md" "수동작성 API 참조 (api)" \
  "수동 작성 API 문서. 자동생성 API 는 repo root \`api/\` 와 택일."

echo ""
echo "[OK] 업데이트 완료: $CURRENT_VERSION -> $UPSTREAM_VERSION"
echo "[i] local/ 폴더는 그대로 유지됩니다."
