# WDAC(Device Guard) 예외 요청 — Emscripten(emsdk) WASM 빌드용

## 배경
- 펌웨어(`main/oled_ui.c`)의 OLED 렌더링을 **웹 시뮬레이터**에서 그대로 실행해 테스트하기 위해
  Emscripten(emsdk)으로 WASM 빌드가 필요합니다.
- 이 PC는 **Device Guard / WDAC(Windows Defender Application Control)** 가 **Enforced(=2)** 모드입니다.
  - 확인: `Get-CimInstance Win32_DeviceGuard -Namespace root\Microsoft\Windows\DeviceGuard`
    → `CodeIntegrityPolicyEnforcementStatus = 2`
- emsdk의 LLVM 도구(미서명)가 정책에 차단됩니다:
  - 실행 시도 → `blocked by your organization's Device Guard policy` (오류 4551)
- 참고: ESP-IDF(`~/.espressif`)의 컴파일러도 **미서명**이지만 **managed-installer 신뢰**로 통과합니다.
  emsdk는 git clone + python 설치라 그 신뢰 태그가 없어 차단됩니다.

## 요청
아래 LLVM 실행파일(또는 `D:\emsdk\upstream\bin\` 디렉토리)을 코드 무결성 정책에 **신뢰 추가**해 주세요.
빌드에 실제로 필요한 것은 **clang.exe** 와 **wasm-ld.exe** 두 개이며, 보조 도구까지 포함하면 아래 목록입니다.

| 파일 | 경로 | 역할 |
|---|---|---|
| clang.exe   | `D:\emsdk\upstream\bin\clang.exe`   | C → WASM 컴파일 |
| wasm-ld.exe | `D:\emsdk\upstream\bin\wasm-ld.exe` | WASM 링크 |
| clang++.exe | `D:\emsdk\upstream\bin\clang++.exe` | (보조) |
| lld.exe / llvm-ar.exe / llvm-nm.exe / llvm-objcopy.exe | `D:\emsdk\upstream\bin\` | (보조) |

### SHA256 (해시 규칙용)
```
clang.exe    2DA762F1C18603E35ABC6DE2A8081B5FA5652AE71A93A802DFFA029AED68DD89
wasm-ld.exe  02D242B99FCCFC60E5A8C04F8A489497F848793315E9B06839C4FF74BE463D40
```
> 나머지 파일 해시가 필요하면 `Get-FileHash <path> -Algorithm SHA256` 로 추출 가능합니다.

## 등록 방법 (관리자, 택1)
**A. 경로 규칙 (가장 간단)** — `D:\emsdk\` 하위를 신뢰. WDAC 보충 정책에 FilePath 규칙:
```powershell
# 예시 — 실제 정책 파일/배포는 환경에 맞게
$rules  = New-CIPolicyRule -FilePathRule "D:\emsdk\*"
New-CIPolicy -FilePath .\emsdk_allow.xml -Rules $rules -UserPEs
ConvertFrom-CIPolicy .\emsdk_allow.xml .\emsdk_allow.cip
# .cip 를 코드무결성 보충정책으로 배포(그룹정책/Intune/CITool)
# CITool -up .\emsdk_allow.cip   (Win11 22H2+)
```
**B. 해시 규칙** — 위 SHA256 으로 `New-CIPolicyRule -Hash` (업데이트 시 갱신 필요).

**C. Managed Installer** — emsdk 를 신뢰된 설치자로 재설치(환경에 따라).

## 검증 (예외 등록 후)
```powershell
cmd /c "D:\emsdk\upstream\bin\clang.exe --version"   # 버전이 출력되면 통과
```
그 다음 빌드:
```powershell
D:\dev\workspaces\somfy-blinds-things-by-claude\sim\wasm\build.ps1
# → sim\oled_sim.js (+ .wasm) 생성. web_sim.html 새로고침하면 자동 WASM 모드.
```

## 보안 메모
- emsdk 는 공개 오픈소스(Emscripten/LLVM) 입니다. 자체 서명이 없을 뿐 출처는 명확합니다.
- 경로 규칙(A)은 `D:\emsdk` 전체를 신뢰하므로, 보안상 해시 규칙(B)을 선호하면 위 2개 해시만으로도 빌드가 됩니다(보조 도구가 추가로 막히면 그 해시도 동일 방식으로 추가).
