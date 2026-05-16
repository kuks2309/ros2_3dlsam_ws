# 매뉴얼 / 데이터시트 보관·인용·검증

외부 벤더의 매뉴얼·데이터시트·프로토콜 명세를 어디에 보관하고, 어떻게 인용하며, 어떻게 검증할지 정하는 단일 근원. 본 룰 위반 시 거짓 단정 누적으로 다중 정정 라운드 / 토큰 낭비 / 신뢰 손상이 발생한다 (실제 사례 §12 참조).

본 룰은 OMC Skill `manual-first` 로도 등록되어 spec 관련 키워드 (datasheet, 데이터시트, manual, 매뉴얼, spec, 사양, Operation Conditions, Electrical Characteristics, 위반, 초과, non-compliance, violation, 무보증 등) 등장 시 본 룰을 **자동 인용 (passive cite)** 한다 — 사용자 명시 지시 없이 추가 작업 수행 금지 ([skill_update.md](skill_update.md) §자동화 정책).

---

## 1. 보관 위치

- 외부 벤더 매뉴얼은 `docs/manual/<vendor>/<product>/` 하위에 보관한다.
- 모듈에 강하게 결합된 매뉴얼은 모듈 내부(예: `<모듈경로>/docs/manual/`)에 둘 수도 있으며, 위치는 모듈 CLAUDE.md 가 결정한다.
- **간단 단일 프로젝트** (예: 단일 칩 임베디드) 에서는 루트 `manual/` 폴더 단순 사용도 허용 — 모듈 CLAUDE.md 에 명시.
- 원본 파일명을 가급적 유지하되, 경로 / 검색이 불편하면 `<vendor>_<product>_<version>.pdf` 형식으로 정규화한다.
- PDF 가 우선이며, 변환본(텍스트 추출 등)은 원본과 함께 보관한다 (예: `pdftotext -layout` 결과를 `.txt` 로 같이 저장).

## 2. 인용 규칙

- 코드 주석 / 문서에서 인용 시: **매뉴얼 이름·섹션·페이지**를 명시한다.
- 인용한 매뉴얼의 상대 경로를 함께 적어 추적 가능하게 한다.
- **강제 인용 형식**: `[문서명 v버전, Table N, page P](경로/파일명.pdf)`
  - 예: `[<Vendor> <Product> DataSheet vX.Y, Table N, page P](docs/manual/<vendor>/<product>/datasheet.pdf)`
- 외부 매뉴얼에 의존하는 상수·환산식·시퀀스 코드는 매뉴얼 인용을 해당 상수/함수 바로 위에 둔다 ([coding.md](coding.md) "상수 분리 원칙" 와 결합).
- 매뉴얼 버전 차이로 동작이 달라질 수 있는 부분은 **참조한 매뉴얼 버전**을 함께 명시한다.

## 3. source 분리 (가장 중요)

- **벤더 SDK / 드라이버 매크로 ≠ silicon (또는 device) datasheet spec** ← 본 룰의 핵심
- SDK docstring (권장 사용 범위) ≠ datasheet (silicon spec)
- 둘은 **별도 검증 항목** 으로 다룬다.
- SDK 매크로 (예: 어느 벤더 SDK 의 `<PERIPHERAL>_<PARAM>_MAX` 형태 매크로) 에서 datasheet spec 추론 **금지**.
- SDK 권장 범위 (예: docstring 의 "Range = [N, M]") 도 datasheet 와 별개. stale 가능성 항상 의심.

### 3.1 역방향 비약 경고 (datasheet → 운영점 해석)

datasheet 를 읽고 운영점을 해석할 때 다음 비약을 **금지**:

- ❌ "TYP = 권장값" 비약: datasheet 의 TYP 컬럼은 **typical 측정 기준점** (대표 silicon, 25°C, 표준 조건). datasheet 가 명시적으로 "recommended operating point" 라 표기하지 않은 한 "TYP = 권장 운영값" 으로 단정 금지.
- ❌ "Min/Max 안에 들어오면 무조건 OK" 비약: Min~Max 는 spec 보장 범위. 단 측정 조건 (postcalibration, ENRMS, ripple, 온도, 전압, 부하 등 footnote) 이 충족돼야 함. footnote 미인용 시 ⓦ 격하.
- ❌ SDK 매크로 수정값 = datasheet TYP 와 일치 = "합리적 설정" 비약: 매크로 수정의 의도와 datasheet TYP 일치는 **우연** 일 수 있음. "매크로 수정자가 datasheet 보고 의도적으로 TYP 정렬" 단정은 commit 메시지 / PR (Pull Request) 설명 등 별도 증거 필요.

### 3.2 datasheet vs User Manual / Family Manual 분리

벤더 문서 체계에서:

- **DataSheet**: pinout, package, 전기 특성 (Operation Conditions, Electrical Characteristics), 일반 사양
- **User Manual / Family Manual**: 모듈 (peripheral) 의 register-level 동작, IP 챕터, SR (service request) 라우팅, DMA/IRQ 토폴로지

→ 한 PDF 에 모든 정보가 있지 않다. 분석 대상이 다음에 해당하면 추가 다운로드 필요:
- Module register / SR / IRQ vector 매핑 → User Manual / Family Manual
- DMA 채널 ↔ peripheral SR 매핑 → User Manual / Family Manual
- 외부 trigger 라우팅 mux (예: GPT/PWM ↔ ADC trig) → User Manual / Family Manual

`manual/` 폴더에 두 종류 모두 보관 권장: `<vendor>_<product>_DataSheet_vX.Y.pdf` + `<vendor>_<product>_UserManual_vX.Y.pdf`.

## 4. 추정 금지 · 실측 검증

- 데이터시트의 모호한 표현(예: "정밀도 1/N", "최대 N pulse", 단위 미표기 수치)을 **추정으로 단정 적용하지 않는다**.
- 의미가 불분명하면 다음 순서로 처리:
  1. 벤더의 추가 자료(예: 응용 노트, 펌웨어 매뉴얼, FAQ) 확인
  2. 벤더 기술지원 문의
  3. 실측 / 실험으로 검증
- 실측이 데이터시트 표현과 다를 때: **실측을 신뢰**하고, 차이의 원인 가설을 주석에 남긴다.
- 모호한 수치를 단정 적용했다가 silent bug 가 발견되면 ([coding.md](coding.md) "상수 분리 원칙"), 수정 이력(v1/v2/v3 가설과 사유)을 코드 또는 모듈 CLAUDE.md 에 보존한다.

## 5. 라이선스 / 외부 공개

- 매뉴얼 PDF 를 GitHub 공개 저장소에 commit 하기 전 라이선스 / 저작권을 확인한다.
- 벤더 NDA / 재배포 금지 매뉴얼은 다음 중 하나로 처리:
  - 별도 비공개 저장소
  - `docs/manual/local/` (gitignore 대상)
  - 공식 URL 링크로 대체 (저장소에는 위치 메타데이터만)
- 공개 가능한 데이터시트는 가능하면 **공식 URL 링크**를 우선하고, 로컬 사본은 보조로 둔다 (벤더가 매뉴얼을 갱신할 때 stale 사본 의존 위험 감소).

## 6. 매뉴얼 누락 / 모호 처리

- 매뉴얼이 없거나 핵심 항목이 모호한 부분은 모듈 CLAUDE.md 의 "Open Question" 또는 이슈 트래커에 기록한다.
- 모호한 항목에 임시 추정값을 사용해야 한다면 [tech_debt.md](tech_debt.md) "우회 사용 3 조건"을 따른다 (사유·승인·정리 일정 기록).
- 추정값을 코드에 둘 때는 `// TODO(YYYY-MM-DD): 매뉴얼 확인 또는 실측 검증 필요` 형식의 주석을 함께 남긴다.

---

## 7. 작업 전 체크리스트 (Pre-Work Checklist)

분석 작업 시작 전 반드시 확인. 미충족 항목은 **진행 전 해결**:

- [ ] 본 작업이 외부 spec / datasheet / 표준 / 인증에 의존하는가?
- [ ] 의존한다면 1차 source (datasheet PDF) 가 정해진 위치 (`manual/` 또는 `docs/manual/<vendor>/<product>/`) 에 있는가?
- [ ] 없으면 사용자에게 source 제공 요청 → **받기 전 spec 관련 결론 보류**
- [ ] 기존 문서 / AI 보고서에 검증 안 된 spec 주장이 있는가? (있으면 ⓦ/⚠ 격하 표시)
- [ ] 분석 범위 결정 — datasheet 의존 부분 vs 코드 분석 부분 **명확히 분리**

## 8. 작업 중 체크리스트 (In-Progress Checklist)

각 주장 작성 시:

- [ ] 검증 등급 표시: **✓** (1차 source 직접) / **ⓦ** (다른 보고만) / **⚠** (UNVERIFIED)
- [ ] ✓ 표시 = file:line 인용 또는 datasheet:page 인용 필수
- [ ] SDK / 드라이버 매크로 인용 시 → "**silicon (또는 device) spec 아님**" 명시
- [ ] 강한 단정어 사용 룰 (강제):
  - **금지 단어** (primary source 없이): `위반`, `초과`, `무보증`, `non-compliance`, `violation`, `fail`, `규격 위반`, `인증 위반`
  - **사용 조건**: primary source 직접 인용 + page/table 번호 첨부 시에만
  - primary source 없을 시: `추정`, `의심`, `미확인`, `확인 필요` 등 약한 표현

## 9. 작업 후 체크리스트 (Post-Work Checklist)

문서 완성 / 정정 라운드 종료 전:

- [ ] 모든 **✓** 항목 = 인용 (file:line 또는 datasheet:page) 있는가?
- [ ] SDK / 드라이버 매크로 → silicon spec 비약 있는가? 있으면 **⚠** 로 격하
- [ ] "위반 / fail / non-compliance" 단정어 사용 항목 = primary source 첨부?
- [ ] 미검증 추론을 "✓" 로 표시한 곳 없는가?
- [ ] 정정 이력 (vN → vN+1) 명시 — 무엇을 왜 정정?
- [ ] 다음 라운드 필요한 **⚠** 항목을 사용자에게 명시 (datasheet 추가 다운로드 요청 등)

## 10. 검증 등급 (강제 표기)

| 표기 | 의미 | 허용 단정어 |
|------|------|---------------|
| **✓** | 1차 source 직접 확인 (코드 file:line 또는 datasheet 페이지) | 강한 단정 OK |
| **ⓦ** | 다른 워커 / AI 보고만, lead 직접 미확인 | 약한 표현 ("보고됨", "주장됨") |
| **⚠** | 추론 / 추측, 1차 source 없음 | "추정", "의심", "확인 필요" 만 |

## 11. Datasheet 다운로드 표준 절차

1. **`manual/` 먼저 확인** (이미 있으면 재다운로드 X)
2. WebSearch 로 공식 PDF URL 확인
3. 다운로드 시도:
   - `curl -sSL -A "Mozilla/5.0" -o manual/<file>.pdf <URL>`
   - 벤더 사이트 직접 차단 시 미러 시도 (mouser, farnell, alldatasheet, digikey)
4. **차단 시 사용자에게 수동 다운로드 요청** → 사용자가 `manual/` 에 배치
5. **검증**: `file manual/<file>.pdf` 로 PDF 매직 바이트 확인 (HTML 차단 페이지 받지 않았는지)
6. 텍스트 추출: `pdftotext -layout manual/<file>.pdf manual/<file>.txt`
7. spec parameter 검색: `grep -in "<parameter_name>\|Operation Conditions\|Electrical Characteristics" manual/<file>.txt`
8. spec 표 컨텍스트 (전후 50~100줄) `sed -n` 으로 발췌 후 분석
9. 인용 시 §2 형식 강제

---

## 12. 본 룰 위반 시 일반 패턴 (사례 추상화)

대표 위반 시퀀스 (실제 발생):

1. AI 가 벤더 SDK 의 `<PARAMETER>_MAX` 매크로를 보고
2. **silicon datasheet spec 으로 비약** → "현재 동작 값이 datasheet 위반" 거짓 단정
3. downstream 거짓 결론 누적: "외부 표준(예: AEC-Q100, 안전인증) 위반", "정밀도(INL/DNL 등) 무보증", "제품 위험"
4. 후속 검증/협업 워크플로 (병렬 워커, 외부 도구) 의 **부분 전제 오염** — 모두 거짓 단정을 사실로 받아들임
5. 다중 정정 라운드 발생, 토큰 / 시간 낭비, 사용자 신뢰 손상
6. 사용자가 datasheet PDF 직접 다운로드 후 검증 → spec 안쪽 정상 (TYP 값 근방), SDK 매크로는 SW 보수 한계였을 뿐

**본 룰의 핵심**: "벤더 SDK / 드라이버 매크로 ≠ silicon (또는 device) datasheet spec" 을 항상 의식하고 1차 source 직접 확인을 강제한다. 사용자가 source 제공 가능성 확인 → 받기 전 단정 보류.

프로젝트별 상세 사례는 해당 프로젝트의 case study 파일에 보존 (예: `<프로젝트>/docs/<분석명>/MASTER.md` 부록).

---

## 13. 변경 절차

본 룰은 SSOT (Single Source of Truth / 단일 근원) 이므로 변경 시 **사용자 승인 필수**. 변경 후:
1. CLAUDE.md §0 표 갱신 확인
2. README.md 표 / 원본 미반영 목록 동기화 확인
3. Skill 파일 (`skills/manual-first.md`) 동기화
4. CHANGELOG / VERSION (semver) 갱신
