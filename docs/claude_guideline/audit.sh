#!/usr/bin/env bash
# claude_guideline/audit.sh — docs/ canonical 구조 점검 (dry-run, 파일 이동 없음)
#
# 사용법:
#   audit.sh                      # 현재 디렉터리 점검
#   audit.sh <path> [<path>...]   # 명시한 경로(들) 점검
#   audit.sh --batch <file>       # <file> 의 줄마다 경로 하나, 일괄 점검
#
# 출력: stdout 에 프로젝트별 finding 목록. 항상 exit 0 (audit-only).
#
# SSOT: kuks_claude_setup/claude_guideline/documentation.md
#       (Variant → Canonical 매핑 / 변종 차단 룰 섹션)

set -u

# 색상 (TTY 일 때만)
if [ -t 1 ]; then
  C_RED=$'\033[0;31m'; C_YEL=$'\033[0;33m'; C_GRN=$'\033[0;32m'
  C_BLU=$'\033[0;34m'; C_DIM=$'\033[0;90m'; C_RST=$'\033[0m'
else
  C_RED=""; C_YEL=""; C_GRN=""; C_BLU=""; C_DIM=""; C_RST=""
fi

# Variant → Canonical 매핑 (folder)
# 형식: "variant_name:canonical_name"
# SSOT 자산명 예외: claude-mistake (하이픈), claude_guideline (언더바), superpowers — 그대로 유지
FOLDER_VARIANTS=(
  "code-review:code_review"
  "code_reivew:code_review"
  "issues-fixes:issues_and_fixes"
  "issues_fixes:issues_and_fixes"
  "issue_and_fix:issues_and_fixes"
  "sw_structure:architecture"
  "sw_structures:architecture"
  "sw-architecture:architecture"
  "stratedgy:strategy"
  "mistake:claude-mistake"
  "claude_mistake:claude-mistake"
  "claude_mistakes:claude-mistake"
)

# 단일 파일 → 폴더 승격
FILE_TO_FOLDER=(
  "INSTALLATION.md:usage"
  "USER_GUIDE.md:usage"
  "run_guide.md:usage"
  "usage.md:usage"
  "TROUBLESHOOTING.md:troubleshooting"
  "CODE_REVIEW_REPORT.md:code_review"
  "PROJECT_STRUCTURE.md:architecture"
)

# 알려진 오탈자 사전 (간이)
TYPO_DICT=(
  "stratedgy:strategy"
  "reivew:review"
  "calibation:calibration"
  "performace:performance"
)

# 알려진 약자 사전 (Acronym → 풀네임) — documentation.md §약자 병기
# 형식: "ACRONYM:풀네임"
ACRONYM_DICT=(
  "SOP:Standard Operating Procedure / 표준 운영 절차"
  "SSOT:Single Source of Truth / 단일 근원"
  "MCP:Model Context Protocol"
  "KST:Korea Standard Time / 한국 표준시"
  "TBD:To Be Determined / 미정"
  "API:Application Programming Interface"
  "LLM:Large Language Model"
  "PR:Pull Request"
  "CI:Continuous Integration"
  "ECC:Everything Claude Code"
)

# Counters
declare -i N_PROJ=0 N_ISSUE=0

# ---- 함수 ----

# 한 프로젝트 점검: $1 = 절대경로
audit_project() {
  local proj="$1"
  local docs="$proj/docs"
  local rel="${proj/#$HOME\//~/}"

  N_PROJ+=1
  echo ""
  echo "${C_BLU}========================================${C_RST}"
  echo "${C_BLU}[#$N_PROJ] $rel${C_RST}"
  echo "${C_BLU}========================================${C_RST}"

  if [ ! -d "$docs" ]; then
    echo "${C_DIM}  docs/ 없음 — skip${C_RST}"
    return 0
  fi

  # 1) variant 폴더 검출
  local found_variant=0
  for entry in "${FOLDER_VARIANTS[@]}"; do
    local v="${entry%:*}"
    local c="${entry#*:}"
    if [ -d "$docs/$v" ]; then
      echo "${C_YEL}  [variant]${C_RST} $docs/$v/ → ${C_GRN}$c/${C_RST} 로 rename 권고"
      found_variant=1
      N_ISSUE+=1
    fi
  done

  # 2) 단일 파일 → 폴더 승격 (docs/ 직속만)
  for entry in "${FILE_TO_FOLDER[@]}"; do
    local f="${entry%:*}"
    local fdir="${entry#*:}"
    if [ -f "$docs/$f" ]; then
      echo "${C_YEL}  [promote]${C_RST} $docs/$f → ${C_GRN}$fdir/$f${C_RST} (단일 파일은 폴더로 승격)"
      N_ISSUE+=1
    fi
  done

  # 3) docs/ 직속 평탄 .md 5개 이상 + 공통 prefix
  local flat_md_count
  flat_md_count=$(find "$docs" -maxdepth 1 -name '*.md' -type f 2>/dev/null | grep -v -E '/(README|CHANGELOG)\.md$' | wc -l)
  if [ "$flat_md_count" -ge 5 ]; then
    echo "${C_YEL}  [flat]${C_RST} docs/ 직속 .md $flat_md_count 개 — 카테고리 폴더 권고"
    # 공통 prefix 검출 (예: *_code_updates.md 5개+ → code_updates/)
    if find "$docs" -maxdepth 1 -name '*_code_updates.md' -type f 2>/dev/null | head -5 | wc -l | grep -q '^5$'; then
      echo "${C_DIM}    └ *_code_updates.md prefix 발견 → ${C_GRN}code_updates/${C_RST} 로 묶음 권고"
    fi
    N_ISSUE+=1
  fi

  # 4) docs/ 안 외부 PDF 검출
  local pdf_count
  pdf_count=$(find "$docs" -type f -iname '*.pdf' 2>/dev/null | wc -l)
  if [ "$pdf_count" -gt 0 ]; then
    echo "${C_YEL}  [external-doc]${C_RST} docs/ 안 PDF $pdf_count 개 — repo root ${C_GRN}manual/${C_RST} 로 이전 + ${C_GRN}manual/SOURCES.md${C_RST} 추가 권고"
    N_ISSUE+=1
  fi

  # 5) 한글/공백 폴더명
  local bad_name
  bad_name=$(find "$docs" -mindepth 1 -maxdepth 2 -type d 2>/dev/null | grep -P '[가-힣]|[ ]' || true)
  if [ -n "$bad_name" ]; then
    echo "${C_YEL}  [naming]${C_RST} 한글/공백 폴더명 발견:"
    echo "$bad_name" | sed "s|^|    ${C_RED}|; s|$|${C_RST}|"
    N_ISSUE+=1
  fi

  # 6) 오탈자 폴더명
  while IFS= read -r d; do
    [ -z "$d" ] && continue
    local base; base="$(basename "$d")"
    for entry in "${TYPO_DICT[@]}"; do
      local typo="${entry%:*}"
      local fix="${entry#*:}"
      if [[ "$base" == *"$typo"* ]]; then
        local suggest="${base/$typo/$fix}"
        echo "${C_YEL}  [typo]${C_RST} $d → ${C_GRN}$(dirname "$d")/$suggest${C_RST}"
        N_ISSUE+=1
      fi
    done
  done < <(find "$docs" -mindepth 1 -maxdepth 3 -type d 2>/dev/null)

  # 6.5) docs/user_instructions/ (또는 legacy docs/request/) 안 오분류 파일 검출
  for ui_dir in "$docs/user_instructions" "$docs/request"; do
    [ ! -d "$ui_dir" ] && continue
    local ui_base; ui_base="$(basename "$ui_dir")"
    while IFS= read -r f; do
      [ -z "$f" ] && continue
      local base; base="$(basename "$f")"
      # 허용 파일
      [[ "$base" == "user_instructions.md" ]] && continue
      [[ "$base" == "requests.md" ]] && continue
      [[ "$base" == "README.md" ]] && continue
      # 오분류 패턴 (파일명)
      if [[ "$base" =~ (review|report|summary|analysis) ]]; then
        echo "${C_YEL}  [user-instructions-misclass]${C_RST} $f — $ui_base/ 는 사용자 지시 기록 전용 (형식: user_instruction_handling_sop.md §3). ${C_GRN}code_review/${C_RST} 또는 ${C_GRN}analysis/${C_RST} 로 이전 권고"
        N_ISSUE+=1
      fi
    done < <(find "$ui_dir" -maxdepth 1 -type f -name '*.md' 2>/dev/null)
    # 결과 요약성 헤딩 검출 (v1.8.2 신규 — 같은 카테고리 재발 차단)
    while IFS= read -r f; do
      [ -z "$f" ] && continue
      if grep -qE "^### (처리|결론|산출물)" "$f" 2>/dev/null; then
        echo "${C_YEL}  [user-instructions-headings]${C_RST} $f — ### 처리/### 결론/### 산출물 헤딩 검출. v1.8.2 SOP §3 정정으로 본 헤딩은 ${C_GRN}docs/worklog/${C_RST} 책임. requests.md → user_instructions.md 마이그레이션 시 worklog 로 이전 권고"
        N_ISSUE+=1
      fi
    done < <(find "$ui_dir" -maxdepth 1 -type f -name '*.md' 2>/dev/null)
    # legacy 폴더명 검출
    if [ "$ui_base" = "request" ]; then
      echo "${C_YEL}  [legacy-request]${C_RST} $ui_dir/ — v1.8.2 부터 ${C_GRN}user_instructions/${C_RST} 로 rename. \`git mv $ui_dir $docs/user_instructions\` 권고"
      N_ISSUE+=1
    fi
  done

  # 6.6) docs/claude-mistake/ 권장 (SSOT 핵심 자산이지만 미설치 → 정보성 권고만, 강제 아님)
  if [ ! -d "$docs/claude-mistake" ]; then
    echo "${C_DIM}  [hint]${C_RST} docs/claude-mistake/ 없음 — Claude 실수 재발 방지 기록 영역. SSOT 자산: kuks_claude_setup/claude-mistake/README.md 참조"
  else
    # 6.6.1) claude-mistake YYYY-MM-DD.md 안 HH:MM 시간 역순 검사 (v1.8.2 신규)
    while IFS= read -r f; do
      [ -z "$f" ] && continue
      local base; base="$(basename "$f")"
      # 파일명 형식 검증
      [[ ! "$base" =~ ^20[0-9]{2}-[0-9]{2}-[0-9]{2}\.md$ ]] && continue
      # 파일 내 ## YYYY-MM-DD HH:MM 헤딩 추출 후 시간 비교
      local times; times=$(grep -oE "^## 20[0-9]{2}-[0-9]{2}-[0-9]{2} [0-9]{2}:[0-9]{2}" "$f" 2>/dev/null | grep -oE "[0-9]{2}:[0-9]{2}$")
      [ -z "$times" ] && continue
      local prev=""; local violated=0
      while IFS= read -r t; do
        [ -z "$t" ] && continue
        if [ -n "$prev" ] && [[ "$t" > "$prev" ]]; then
          violated=1
          break
        fi
        prev="$t"
      done <<< "$times"
      if [ "$violated" = "1" ]; then
        echo "${C_YEL}  [mistake-order]${C_RST} $f — entry 시각이 시간 역순 (최신 위) 규칙 위반. README.md §파일 구조 참조"
        N_ISSUE+=1
      fi
    done < <(find "$docs/claude-mistake" -maxdepth 1 -type f -name '*.md' 2>/dev/null)
    # 6.6.2) claude-mistake entry 안 TBD 미해결 항목 검출 (v1.8.2 신규)
    while IFS= read -r f; do
      [ -z "$f" ] && continue
      local base; base="$(basename "$f")"
      [[ "$base" == "README.md" ]] && continue
      [[ "$base" == "INDEX.md" ]] && continue
      if grep -qE "(TBD|후보|추후|미정)" "$f" 2>/dev/null; then
        echo "${C_DIM}  [mistake-tbd]${C_RST} $f — TBD/후보/추후/미정 키워드 검출. README.md §Closure 규칙: closure 의무 (7일 이내), INDEX.md §미해결 항목 표 등재 확인"
      fi
    done < <(find "$docs/claude-mistake" -maxdepth 1 -type f -name '*.md' 2>/dev/null)
    # 6.6.3) INDEX.md 부재 권고
    if [ ! -f "$docs/claude-mistake/INDEX.md" ]; then
      echo "${C_DIM}  [hint]${C_RST} docs/claude-mistake/INDEX.md 부재 — 카테고리·메타 패턴 색인 누락. SSOT: kuks_claude_setup/claude-mistake/INDEX.md 참조"
    fi
  fi

  # 7) docs/ 하위 폴더 README 부재 (v1.8.6: 빈 폴더 자체는 정상, README 부재만 경고)
  while IFS= read -r d; do
    [ -z "$d" ] && continue
    if [ ! -f "$d/README.md" ]; then
      echo "${C_DIM}  [no-readme]${C_RST} $d/ — README.md 부재. install.sh / update.sh 재실행 또는 stub README 작성 권고"
    fi
  done < <(find "$docs" -mindepth 1 -maxdepth 1 -type d 2>/dev/null)

  # 7.5) Authoring SSOT vs Mirror 가드 (F01: documentation.md §SSOT 경로)
  # authoring SSOT 는 본 audit.sh 가 있는 디렉토리 (= kuks_claude_setup/claude_guideline/).
  # mirror 는 다운스트림 프로젝트의 docs/claude_guideline/.
  # WHITELIST: 시차 허용 자산 (VERSION, CHANGELOG.md).
  local script_dir; script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
  if [ -d "$docs/claude_guideline" ] && [ "$script_dir" != "$docs/claude_guideline" ]; then
    # [tree-diff]: authoring vs mirror 파일 차이 검출
    while IFS= read -r mirror_file; do
      [ -z "$mirror_file" ] && continue
      local rel="${mirror_file#$docs/claude_guideline/}"
      case "$rel" in
        VERSION|CHANGELOG.md) continue ;;
      esac
      local auth_file="$script_dir/$rel"
      if [ -f "$auth_file" ] && ! diff -q "$auth_file" "$mirror_file" >/dev/null 2>&1; then
        echo "${C_YEL}  [tree-diff]${C_RST} docs/claude_guideline/$rel ↔ authoring SSOT 와 다름. update.sh 재실행 또는 authoring 측에서 변경 후 sync 권고"
        N_ISSUE+=1
      fi
    done < <(find "$docs/claude_guideline" -maxdepth 2 -type f -name '*.md' 2>/dev/null)
    # [mirror-only-file]: mirror 에만 있는 파일 검출
    while IFS= read -r mirror_file; do
      [ -z "$mirror_file" ] && continue
      local rel="${mirror_file#$docs/claude_guideline/}"
      local auth_file="$script_dir/$rel"
      if [ ! -f "$auth_file" ]; then
        echo "${C_RED}  [mirror-only-file]${C_RST} docs/claude_guideline/$rel — authoring SSOT 에 부재. (a) authoring 추가, (b) mirror 삭제, (c) 참조 제거 중 택일"
        N_ISSUE+=1
      fi
    done < <(find "$docs/claude_guideline" -maxdepth 2 -type f -name '*.md' 2>/dev/null)
    # [authoring-mtime]: mirror 가 authoring 보다 새로움 (직접 수정 의심)
    while IFS= read -r mirror_file; do
      [ -z "$mirror_file" ] && continue
      local rel="${mirror_file#$docs/claude_guideline/}"
      local auth_file="$script_dir/$rel"
      if [ -f "$auth_file" ] && [ "$mirror_file" -nt "$auth_file" ]; then
        echo "${C_DIM}  [authoring-mtime]${C_RST} docs/claude_guideline/$rel 가 authoring 보다 새로움. mirror 직접 수정 의심"
      fi
    done < <(find "$docs/claude_guideline" -maxdepth 2 -type f -name '*.md' 2>/dev/null)
  fi

  # 7.6) Template parity 가드 (F02: templates/CLAUDE.md.template ↔ README.md Tier 표)
  local readme_md="$script_dir/README.md"
  local tpl_md="$script_dir/templates/CLAUDE.md.template"
  if [ -f "$readme_md" ] && [ -f "$tpl_md" ]; then
    # README Tier 표의 각 .md 파일이 template 에 등장하는지 검사
    while IFS= read -r ref_file; do
      [ -z "$ref_file" ] && continue
      if ! grep -q "$ref_file" "$tpl_md"; then
        echo "${C_DIM}  [template-vs-readme]${C_RST} $ref_file — README Tier 표에는 있으나 template 에 미반영. templates/CLAUDE.md.template 동기화 권고"
      fi
    done < <(grep -oE '\[[a-z_]+\.md\]\(([a-z_]+\.md)\)' "$readme_md" | sed -E 's/.*\(([^)]+)\)/\1/' | sort -u)
  fi

  # 8) ROS2 워크스페이스 패키지 docs/ 누락 (src/<pkg>/package.xml 있고 src/<pkg>/docs/ 없음)
  if [ -d "$proj/src" ]; then
    while IFS= read -r pkg_xml; do
      [ -z "$pkg_xml" ] && continue
      local pkg_dir; pkg_dir="$(dirname "$pkg_xml")"
      local pkg_rel="${pkg_dir/#$proj\//}"
      if [ ! -d "$pkg_dir/docs" ]; then
        echo "${C_YEL}  [ros2-pkg]${C_RST} $pkg_rel — docs/ 부재. ${C_GRN}$pkg_rel/docs/README.md${C_RST} 생성 권고"
        N_ISSUE+=1
      fi
    done < <(find "$proj/src" -maxdepth 4 -name 'package.xml' -type f 2>/dev/null)
  fi

  # 9) repo root 필수 파일
  for must in README.md LICENSE CHANGELOG.md VERSION; do
    if [ ! -f "$proj/$must" ]; then
      # LICENSE 는 git 추적 repo 일 때만 강제, 그 외는 권고
      echo "${C_DIM}  [root]${C_RST} $must 부재"
      N_ISSUE+=1
    fi
  done

  # 10) repo root manual/ 있으면 SOURCES.md 필수
  if [ -d "$proj/manual" ] && [ ! -f "$proj/manual/SOURCES.md" ]; then
    echo "${C_YEL}  [manual]${C_RST} manual/ 있으나 ${C_GRN}manual/SOURCES.md${C_RST} 부재"
    N_ISSUE+=1
  fi

  # 11) 약자 미병기 검출 (documentation.md §약자 병기) — hint-level
  # 검증: 첫 등장 줄에 ACR + 풀네임 키워드 (영문/한국어) 동시 존재 여부.
  # 단순 괄호 매칭으로는 false-positive (예: "SSOT (repo_name)") 발생 — 풀네임 키워드 검증 필수.
  # 화이트리스트:
  #   - CHANGELOG.md: 역사적 entry 보존
  #   - user_instructions.md: 사용자 원문 quote 보존
  #   - user_instruction_analysis.md: 폐기/통합 결정 진행 중
  if [ -d "$docs" ]; then
    while IFS= read -r f; do
      [ -z "$f" ] && continue
      local base; base="$(basename "$f")"
      [[ "$base" == "CHANGELOG.md" ]] && continue
      [[ "$base" == "user_instructions.md" ]] && continue
      [[ "$base" == "user_instruction_analysis.md" ]] && continue
      for entry in "${ACRONYM_DICT[@]}"; do
        local acr="${entry%%:*}"
        local full="${entry#*:}"
        local first_ln
        first_ln=$(grep -nE "\\b${acr}\\b" "$f" 2>/dev/null | head -1 | cut -d: -f1)
        [ -z "$first_ln" ] && continue
        local first_line
        first_line=$(sed -n "${first_ln}p" "$f")
        # 풀네임 키워드 split (" / " 구분자) + 트림
        local matched=0
        local saved_ifs="$IFS"
        IFS='/' read -ra parts <<< "$full"
        IFS="$saved_ifs"
        for p in "${parts[@]}"; do
          # 앞뒤 공백 제거
          p="${p# }"; p="${p% }"; p="${p# }"; p="${p% }"
          [ -z "$p" ] && continue
          if echo "$first_line" | grep -qF "$p"; then
            matched=1
            break
          fi
        done
        if [ "$matched" -eq 0 ]; then
          echo "${C_DIM}  [acronym-no-expansion]${C_RST} $f:${first_ln} — '${acr}' 첫 등장 풀네임 키워드 미발견 (예: ${C_GRN}${acr} (${full})${C_RST})"
        fi
      done
    done < <(find "$docs" -type f -name '*.md' 2>/dev/null)
  fi

  if [ "$found_variant" -eq 0 ] && [ "$N_ISSUE" -eq 0 ]; then
    echo "${C_GRN}  [OK]${C_RST} 위반 없음"
  fi
}

# ---- 인자 파싱 ----

PATHS=()
if [ $# -eq 0 ]; then
  PATHS=("$PWD")
elif [ "${1:-}" = "--batch" ]; then
  [ -z "${2:-}" ] && { echo "Usage: audit.sh --batch <file>"; exit 1; }
  while IFS= read -r line; do
    [ -z "$line" ] && continue
    [[ "$line" =~ ^# ]] && continue
    PATHS+=("$line")
  done < "$2"
else
  PATHS=("$@")
fi

# ---- 헤더 ----
echo "${C_BLU}claude_guideline audit.sh${C_RST} — dry-run 점검 (파일 이동 없음)"
echo "${C_DIM}SSOT: kuks_claude_setup/claude_guideline/documentation.md${C_RST}"

# ---- 실행 ----
for p in "${PATHS[@]}"; do
  audit_project "$p"
done

# ---- 요약 ----
echo ""
echo "${C_BLU}========================================${C_RST}"
echo "${C_BLU}요약${C_RST}: 프로젝트 $N_PROJ 개, 발견 항목 $N_ISSUE 건"
echo "${C_BLU}========================================${C_RST}"
echo ""
echo "권고 적용은 사용자가 직접 수행. audit.sh 는 변경하지 않음 (dry-run 전용)."

exit 0
