#!/usr/bin/env pwsh
# Somfy RTS 블라인드 컨트롤러 — 빌드/플래시 스크립트.
# esp-idf · esp-matter 는 외부 SDK(환경변수 참조). 프로젝트 루트를 빌드한다.
#
# 사용:
#   ./build.ps1 [-Action build|set-target|restore-sdkconfig|flash|erase|menuconfig|clean|ota-image] [-Port COMx] [-Board <name>]
#   ota-image : 빌드된 .bin → 보드별 Matter OTA 이미지
#               (dist/somfy_blinds_<board>_<pcf>_<enc>_<oled>_v<NNNN>.ota, oled=<W>x<H>r<0|180>)
#               VID/PID/버전은 보드 sdkconfig, HW 변형(PCF8574/8575·EC11/EC05·OLED해상도/회전)은
#               boards/<board>.h 에서 읽어 파일명·SoftwareVersionString 에 반영(VERSION 4자리 zero-pad).
#
# 보드별 빌드 (브랜드-SoC 키):
#   -Board gnpe-c6  (기본) — GNPE ESP32-C6-0.42 (검증됨, PID 0x8000)
#   -Board xiao-c6         — Seeed XIAO ESP32-C6 (템플릿, PID 0x8003 — 핀맵 조정 필요)
#   -Board esp32-h2        — ESP32-H2 (Thread, 템플릿 — 핀맵 조정 필요)
#   (-Board esp32-c6 = gnpe-c6 역호환 별칭)
#
# 각 보드는 별도 빌드 디렉토리(build-<board>/) + 별도 sdkconfig 사용 →
# 보드 전환 시 전체 재컴파일 없이 보존된 산출물 재사용.
#
# 보드별 매핑 테이블:
#   - idf target  : esp32c6 / esp32h2
#   - sdkconfig.defaults 조합 (Thread vs WiFi 등)
#   - board.h    : boards/<board>.h  (BOARD_PIN_* 매크로)
param(
    [string]$Action = "build",
    [string]$Port   = "COM3",
    [string]$Board  = "gnpe-c6",
    # ── OTA 처럼 변형 선택 빌드 (빈값 = 보드 기본) ──
    [string]$Pcf    = "",   # "" | 8574 | 8575        (좌/우 버튼)
    [string]$Rotary = "",   # "" | ec11 | ec05        (로터리 디텐트)
    [string]$Oled   = "",   # "" | 72x40 | 128x64 | 64x128  (OLED 해상도; 64x128=세로 패널)
    [string]$Rotate = "",   # "" | 0 | 180 | m0 | m180  (OLED 회전; m 접두=좌우반전)
    [string]$Freq   = "",   # "" | 447.70 | 447.72  (기본 송신 주파수 register; 미지정=보드 기본 447.70)
    [string]$Lp     = "",   # "" | on | off   (LP 코어 PCF 폴링; off = HP 직접 폴링)
    [string]$Bat    = "",   # "" | on | off   (배터리 ADC; off = 측정회로 없는 테스트보드)
    [string]$Rot    = "",   # "" | ab | ba   (로터리 A/B 배선; ba = 뒤바뀐 기판)
    # ★2026-09-02 XIAO ESP32C6 온보드 RF 스위치(FM8625H) 안테나 선택.
    #  int = 내장 세라믹(기본·기존 동작) / ext = 외장 u.FL 커넥터.
    #  ※외장 안테나를 **실제로 꽂은 개체만** ext 로 빌드할 것. 안 꽂고 ext 를 고르면
    #    급전점이 열려 내장보다 크게 나빠진다.
    [string]$Ant    = "",   # "" | int | ext
    [switch]$OledTest,      # OLED 단독 부팅 테스트 (Matter/RF/버튼/배터리 전부 비활성, OLED만 구동)
    # ★★2026-08-31 병렬 컴파일 개수 제한 (0 = ninja 기본 = 논리코어 수).
    #  왜: 20코어 기계에서 ninja 가 기본 20+ 개 cc1plus 를 띄우는데, OpenThread 의
    #  C++ 는 하나당 수백MB~1GB 를 먹는다. 다른 앱이 메모리를 쥐고 있으면
    #  `cc1plus.exe: out of memory allocating 1052671 bytes` 로 **빌드가 죽는다**
    #  (실제로 COM8 플래시 중 70/332 에서 터짐, 여유 5.5GB / 페이지파일 27.9GB 사용중).
    #  코어 수가 아니라 **메모리**가 병목이므로 -Jobs 6 정도면 안전하고 속도 손해도 작다.
    [int]$Jobs      = 0
)

# ─── 보드별 매핑 (브랜드-SoC 단위) ──────────────────────────
#  같은 SoC 라도 브랜드(GNPE/XIAO)마다 핀맵·Product ID 가 다르다.
$BoardMap = @{
    "gnpe-c6" = @{                 # GNPE ESP32-C6-0.42 (현 검증 보드, PID 0x8000)
        IdfTarget        = "esp32c6"
        SdkconfigDef     = "sdkconfig.defaults;sdkconfig.defaults.c6_thread"
        EspMatterDevice  = "device_hal/device/esp32c6_devkit_c"
    }
    "xiao-c6" = @{                 # Seeed XIAO ESP32-C6 (PID 0x8003 오버레이)
        IdfTarget        = "esp32c6"
        SdkconfigDef     = "sdkconfig.defaults;sdkconfig.defaults.c6_thread;sdkconfig.defaults.xiao_c6"
        EspMatterDevice  = "device_hal/device/esp32c6_devkit_c"
    }
    "xiao-c6-test" = @{            # ★2026-08-16 XIAO C6 테스트 보드(COM6)
        #  PCF8574 / LP 미적용(공유 I2C) / 447.72MHz / 배터리·충전회로 없음.
        #  실기(xiao-c6, COM7)와 **빌드 디렉터리·sdkconfig 를 분리**해 오갈 때
        #  전체 재빌드가 나지 않게 한다. 핀맵은 boards/xiao-c6-test.h 가
        #  xiao-c6.h 를 include 해 상속(복사본을 두지 않는다).
        IdfTarget        = "esp32c6"
        SdkconfigDef     = "sdkconfig.defaults;sdkconfig.defaults.c6_thread;sdkconfig.defaults.xiao_c6"
        EspMatterDevice  = "device_hal/device/esp32c6_devkit_c"
    }
    "esp32-c6" = @{                # DEPRECATED 별칭 = gnpe-c6 (역호환)
        IdfTarget        = "esp32c6"
        SdkconfigDef     = "sdkconfig.defaults;sdkconfig.defaults.c6_thread"
        EspMatterDevice  = "device_hal/device/esp32c6_devkit_c"
    }
    "esp32-h2" = @{
        IdfTarget        = "esp32h2"
        # H2 는 802.15.4(Thread)+BLE, WiFi 없음 → C6 와 동일 Thread 트랜스포트
        SdkconfigDef     = "sdkconfig.defaults;sdkconfig.defaults.h2_thread"
        EspMatterDevice  = "device_hal/device/esp32h2_devkit_c"
    }
}

if (-not $BoardMap.ContainsKey($Board)) {
    Write-Host "알 수 없는 -Board '$Board'. 지원: $($BoardMap.Keys -join ', ')" -ForegroundColor Red
    exit 1
}
$BoardCfg = $BoardMap[$Board]
$IdfTarget = $BoardCfg.IdfTarget
$SdkconfigDef = $BoardCfg.SdkconfigDef

# ─── 경로는 시스템 환경 변수에서만 읽는다 (개인 경로 비노출) ──────────────
#   RTS_BLINDS_THREAD_PATH / IDF_PATH / ESP_MATTER_PATH / ESP_SSD1306_PATH 를
#   시스템 환경 변수에 등록해 둘 것. 시스템 등록 직후의 기존 셸이면 프로세스에
#   값이 없을 수 있어 Machine 스코프에서 보충한다.
foreach ($v in 'RTS_BLINDS_THREAD_PATH','IDF_PATH','ESP_MATTER_PATH','ESP_SSD1306_PATH') {
    if (-not [Environment]::GetEnvironmentVariable($v)) {
        $m = [Environment]::GetEnvironmentVariable($v, 'Machine')
        if ($m) { Set-Item "Env:$v" $m }
        else {
            Write-Host "[env] 필수 시스템 환경변수 '$v' 미설정 — 등록 후 새 셸에서 실행하세요." -ForegroundColor Red
            exit 1
        }
    }
}
# CMake 는 백슬래시를 이스케이프로 오해 → 환경변수 경로를 forward slash 로 정규화.
function ConvertTo-FSlash([string]$p) { $p -replace '\\','/' }

# ─── 보드 자산 무결성 검증 (없는 파일 → 명확한 에러로 조기 차단) ───
$PROJ_PRECHECK = ConvertTo-FSlash $env:RTS_BLINDS_THREAD_PATH
$missing = @()
# 1) board 헤더 (boards/<board>.h)
$boardHeader = "$PROJ_PRECHECK/main/boards/$Board.h"
if (-not (Test-Path $boardHeader)) { $missing += "main/boards/$Board.h (BOARD_PIN_* 핀맵)" }
# 2) sdkconfig.defaults 조합의 각 파일
foreach ($d in ($SdkconfigDef -split ';')) {
    $d = $d.Trim()
    if ($d -and -not (Test-Path "$PROJ_PRECHECK/$d")) { $missing += $d }
}
if ($missing.Count -gt 0) {
    Write-Host "[board=$Board] 필요한 보드 자산이 없습니다:" -ForegroundColor Red
    foreach ($m in $missing) { Write-Host "  - $m" -ForegroundColor Red }
    Write-Host "  → 위 파일을 생성한 뒤 다시 빌드하세요 (esp32-c6 를 템플릿으로 참고)." -ForegroundColor Yellow
    exit 1
}
# 기본 검증 보드(gnpe-c6, 구 esp32-c6 별칭)는 기존 build/ + sdkconfig 유지(역호환).
# 다른 보드는 build-<board>/, sdkconfig.<board> 별도 디렉토리/파일.
if ($Board -eq "gnpe-c6" -or $Board -eq "esp32-c6") {
    $BuildDir = "build"
    $SdkconfigFile = "sdkconfig"
} else {
    $BuildDir = "build-$Board"
    $SdkconfigFile = "sdkconfig.$Board"
}

# 경로는 환경변수에서 읽고 CMake 용으로 forward slash 정규화 (esp-matter 하위는 파생).
$IDF_PATH    = ConvertTo-FSlash $env:IDF_PATH            # 시스템 환경변수 (esp-idf 루트)
$MATTER_PATH = ConvertTo-FSlash $env:ESP_MATTER_PATH     # 시스템 환경변수 (esp-matter 루트)
$RISCV_GCC   = "$env:USERPROFILE\.espressif\tools\riscv32-esp-elf\esp-14.2.0_20260121\riscv32-esp-elf\bin"  # IDF v5.5.5
$CMAKE_BIN   = "$env:USERPROFILE\.espressif\tools\cmake\3.30.2\bin"
$NINJA_BIN   = "$env:USERPROFILE\.espressif\tools\ninja\1.12.1"
$PIGWEED_BIN = "$MATTER_PATH/connectedhomeip/connectedhomeip/third_party/pigweed/repo/environment/cipd/packages/pigweed"
$VENV        = "$env:USERPROFILE\.espressif\python_env\idf5.5_py3.14_env\Scripts"
$ROM_ELF_DIR = "$env:USERPROFILE\.espressif\tools\esp-rom-elfs\20241011"
$PW_ENV_ROOT = "$MATTER_PATH/connectedhomeip/connectedhomeip/third_party/pigweed/repo/environment"
$PROJ        = ConvertTo-FSlash $env:RTS_BLINDS_THREAD_PATH   # 시스템 환경변수 (이 프로젝트 루트)
# Matter OTA 이미지 생성 도구 (CHIP)
$OTA_TOOL    = "$MATTER_PATH/connectedhomeip/connectedhomeip/src/app/ota_image_tool.py"

Remove-Item Env:MSYSTEM -ErrorAction SilentlyContinue
# toolchain PATH — 지원 보드 모두 RISC-V (esp32c6 / esp32h2)
$env:PATH = "$VENV;$RISCV_GCC;$CMAKE_BIN;$NINJA_BIN;$PIGWEED_BIN;$env:PATH"
$env:IDF_PATH               = $IDF_PATH
$env:ESP_MATTER_PATH        = $MATTER_PATH
$env:ESP_MATTER_DEVICE_PATH = "$MATTER_PATH/$($BoardCfg.EspMatterDevice)"
$env:IDF_TOOLS_PATH         = "$env:USERPROFILE\.espressif"
$env:IDF_PYTHON_ENV_PATH    = "$env:USERPROFILE\.espressif\python_env\idf5.5_py3.14_env"
$env:ESP_ROM_ELF_DIR        = $ROM_ELF_DIR
$env:_PW_ACTUAL_ENVIRONMENT_ROOT = $PW_ENV_ROOT
$env:PYTHONIOENCODING       = "utf-8"
$env:SDKCONFIG_DEFAULTS     = $SdkconfigDef

Set-Location $PROJ

# ─── 로그 디렉토리 ──────────────────────────────────────────
#   모든 빌드/플래시 산출 로그는 ./logs/ 아래에 보드+액션별 파일로 남긴다.
#   (deterministic 이름 → 매 실행 덮어쓰기, logs/ 무한 증식 방지.)
$LogDir = "$PROJ/logs"
New-Item -ItemType Directory -Force -Path $LogDir | Out-Null
$LogPath = "$LogDir/$Board-$($Action.ToLower()).log"

Write-Host "[board=$Board target=$IdfTarget build=$BuildDir log=logs/$Board-$($Action.ToLower()).log]" -ForegroundColor Cyan

# 공통 idf.py 인자 — 보드별 빌드 디렉토리 + sdkconfig + CMake 캐시 BOARD 변수
# ─── 빌드타임 변형 오버라이드 (-Pcf/-Rotary/-Oled/-Rotate → -D BOARD_OVR_*) ───
#   OTA 처럼 한 보드에서 PCF/로터리/OLED 변형을 골라 빌드(boards/board_select.h 가 적용).
#   안 준 항목은 **빈 값**으로 넘겨 CMake 가 건너뛴다(보드 기본 + 캐시 sticky 방지).
if ($Pcf    -and $Pcf    -notin @('8574','8575'))             { Write-Host "[ovr] -Pcf 는 8574|8575"             -ForegroundColor Red; exit 1 }
if ($Rotary -and $Rotary -notin @('ec11','ec05'))             { Write-Host "[ovr] -Rotary 는 ec11|ec05"          -ForegroundColor Red; exit 1 }
# -Rotate: 0|180 (+m 좌우반전) | 90|270 (시계/반시계 90° 회전). 예: m0, 90, 270.
$rotMirror = $false; $rotBase = $Rotate; $rot90 = ''
if ($Rotate -match '^[mM](0|180)$') { $rotMirror = $true; $rotBase = $Matches[1] }
elseif ($Rotate -in @('90','270'))  { $rot90 = $Rotate; $rotBase = '0' }
if ($Rotate -ne '' -and $rotBase -notin @('0','180'))         { Write-Host "[ovr] -Rotate 는 0|180|m0|m180|90|270" -ForegroundColor Red; exit 1 }
if ($Oled   -and $Oled   -notin @('72x40','128x64','64x128')) { Write-Host "[ovr] -Oled 는 72x40|128x64|64x128"  -ForegroundColor Red; exit 1 }
if ($Freq   -and $Freq   -notmatch '^44[0-9]\.\d{2}$')        { Write-Host "[ovr] -Freq 는 447.xx 형식(예 447.70|447.72)" -ForegroundColor Red; exit 1 }
if ($Lp     -and $Lp     -notin @('on','off'))                { Write-Host "[ovr] -Lp 는 on|off"  -ForegroundColor Red; exit 1 }
if ($Bat    -and $Bat    -notin @('on','off'))                { Write-Host "[ovr] -Bat 는 on|off" -ForegroundColor Red; exit 1 }
if ($Rot    -and $Rot    -notin @('ab','ba'))                  { Write-Host "[ovr] -Rot 는 ab|ba"  -ForegroundColor Red; exit 1 }
if ($Ant    -and $Ant    -notin @('int','ext'))                { Write-Host "[ovr] -Ant 는 int|ext" -ForegroundColor Red; exit 1 }

$ovrLR   = if ($Pcf -eq '8575') { '1' } elseif ($Pcf -eq '8574') { '0' } else { '' }
$ovrROT  = if ($Rotary -eq 'ec05') { '1' } elseif ($Rotary -eq 'ec11') { '0' } else { '' }
$ovrR180  = if ($rotBase -eq '180') { '1' } elseif ($rotBase -eq '0') { '0' } else { '' }
$ovrFLIPX = if ($Rotate -eq '') { '' } elseif ($rotMirror) { '1' } else { '0' }
$ovrR90   = if ($rot90) { $rot90 } elseif ($Rotate -ne '') { '0' } else { '' }
$ovrFreq  = if ($Freq -ne '') { "${Freq}f" } else { '' }   # 예 447.70f (CC1101 register 목표 주파수)
# ★2026-08-16 -Lp / -Bat
#  -Lp off : LP 프로그램을 빌드에서 뺀다. PCF8574(1바이트) 개체에서 필수 —
#            lp_core/pcf_lp_config.h 의 LP_PCF_NBYTES 가 2 고정이라 _Static_assert 가 깨진다.
#  -Bat off: 충전/배터리 측정 회로가 없는 보드. 켜 두면 플로팅 핀을 읽어 잔량%가 헛돈다.
$ovrLp   = if ($Lp  -eq 'off') { '0' } elseif ($Lp  -eq 'on') { '1' } else { '' }
$ovrBat  = if ($Bat -eq 'off') { '0' } elseif ($Bat -eq 'on') { '1' } else { '' }
# -Rot ba : 엔코더 A/B 가 뒤바뀌게 배선된 기판(예: COM3 H2). 방향이 통째로 반대가 된다.
$ovrRot  = if ($Rot -eq 'ba') { '1' } elseif ($Rot -eq 'ab') { '0' } else { '' }
$ovrAnt  = if ($Ant -eq 'ext') { '1' } elseif ($Ant -eq 'int') { '0' } else { '' }
$ow=''; $oh=''; $ofx=''; $ooff=''
switch ($Oled) {
    '72x40'  { $ow='72';  $oh='40';  $ofx='1'; $ooff='28' }
    '128x64' { $ow='128'; $oh='64';  $ofx='0'; $ooff='0'  }
    '64x128' { $ow='64';  $oh='128'; $ofx='0'; $ooff='0'  }
}
$OvrDefs = @(
    "-D","BOARD_OVR_HAS_LR_BUTTONS=$ovrLR",
    "-D","BOARD_OVR_ROT_HALF_STEP=$ovrROT",
    "-D","BOARD_OVR_OLED_ROTATE_180=$ovrR180",
    "-D","BOARD_OVR_OLED_FLIP_X=$ovrFLIPX",
    "-D","BOARD_OVR_OLED_ROTATE_90=$ovrR90",
    "-D","BOARD_OVR_OLED_WIDTH=$ow",
    "-D","BOARD_OVR_OLED_HEIGHT=$oh",
    "-D","BOARD_OVR_OLED_FIXUP=$ofx",
    "-D","BOARD_OVR_OLED_COL_OFFSET=$ooff",
    "-D","BOARD_OVR_FREQ=$ovrFreq",
    "-D","BOARD_OVR_PCF_LP_CORE=$ovrLp",
    "-D","BOARD_OVR_HAS_BAT_ADC=$ovrBat",
    "-D","BOARD_OVR_ROT_AB_SWAP=$ovrRot",
    "-D","BOARD_OVR_RF_EXT_ANT=$ovrAnt"
)
if ($Pcf -or $Rotary -or $Oled -or $Rotate -ne '' -or $Freq -ne '' -or $Lp -or $Bat -or $Rot -or $Ant) {
    Write-Host "[ovr] 변형 빌드: Pcf=$Pcf Rotary=$Rotary Oled=$Oled Rotate=$Rotate Freq=$Freq Lp=$Lp Bat=$Bat Rot=$Rot Ant=$Ant" -ForegroundColor Cyan
    if ($Oled) {
        Write-Host "[ovr] ⚠ OLED 해상도 override 는 렌더러·패널크기(ssd1306_init)만 -D 로 바꾼다." -ForegroundColor Yellow
        Write-Host "       물리 컬럼 오프셋은 SSD1306 라이브러리 Kconfig(CONFIG_OFFSETX, sdkconfig)라" -ForegroundColor Yellow
        Write-Host "       sdkconfig 의 CONFIG_OFFSETX 를 $ooff 로 맞춰야 정상(72x40=28 / 128x64·64x128=0)." -ForegroundColor Yellow
    }
}

$IdfArgs = @(
    "-B", $BuildDir,
    "-D", "SDKCONFIG=$SdkconfigFile",
    "-D", "SDKCONFIG_DEFAULTS=$SdkconfigDef",
    "-D", "BOARD=$Board"
) + $OvrDefs
# ★2026-08-31 -Jobs → ninja 병렬 개수. 위 param 주석 참조.
#  ※idf.py 에는 `-j` 옵션이 **없다**(Error: No such option: -j). IDF 는 환경변수
#    IDF_PY_BUILD_JOBS 를 읽어 ninja 에 -j 로 넘긴다(tools/idf_py_actions/tools.py:539).
if ($Jobs -gt 0) {
    $env:IDF_PY_BUILD_JOBS = "$Jobs"
    Write-Host "[jobs] 병렬 컴파일 $Jobs 개로 제한 (IDF_PY_BUILD_JOBS)" -ForegroundColor DarkYellow
}
# 2026-07-22 OLED 단독 부팅 테스트 — app_main 최상단에서 단락(다른 기능 전부 스킵)
if ($OledTest) {
    $IdfArgs += @("-D", "OLED_ONLY_TEST=1")
    Write-Host "[test] ★OLED 단독 부팅 테스트 빌드 (OLED_ONLY_TEST=1) — Matter/RF/버튼/배터리 전부 비활성" -ForegroundColor Magenta
} else {
    # -OledTest 미지정 시 명시적 0 → 직전 테스트 빌드의 CMake 캐시(=1) 잔존 방지(정상 펌웨어 복귀).
    $IdfArgs += @("-D", "OLED_ONLY_TEST=0")
}

# idf.py 실행 + logs/ 자동 tee (menuconfig 는 인터랙티브 → tee 제외).
#  ★ 파라미터명을 $Args 로 쓰면 PowerShell 자동 변수 $Args 와 충돌해 splat 이
#    비어 idf.py 가 인자 없이 실행→도움말만 출력(빌드 no-op)된다. $IdfArgList 사용.
function Invoke-Idf {
    param([Parameter(Mandatory=$true)][string[]]$IdfArgList, [switch]$Interactive)
    if ($Interactive) {
        & python.exe "$IDF_PATH\tools\idf.py" @IdfArgList
    } else {
        & python.exe "$IDF_PATH\tools\idf.py" @IdfArgList 2>&1 | Tee-Object -FilePath $LogPath
    }
}

switch ($Action.ToLower()) {
    "set-target" {
        # 경고: 검증된 sdkconfig 를 defaults 에서 재생성하면 WindowCovering
        #   클러스터 등 빌드 구성이 누락돼 링크 실패한다.
        if (Test-Path $SdkconfigFile) { Copy-Item $SdkconfigFile "$SdkconfigFile.verified.bak" -Force }
        Write-Host "[set-target] 검증 sdkconfig 백업: $SdkconfigFile.verified.bak" -ForegroundColor Yellow
        Write-Host "  링크 깨지면: ./build.ps1 -Board $Board -Action restore-sdkconfig" -ForegroundColor Yellow
        Remove-Item -Force $SdkconfigFile -ErrorAction SilentlyContinue
        Invoke-Idf -IdfArgList (@() + $IdfArgs + @("set-target", $IdfTarget))
    }
    "restore-sdkconfig" {
        $bak = "$SdkconfigFile.verified.bak"
        if (Test-Path $bak) {
            Copy-Item $bak $SdkconfigFile -Force
            Write-Host "[restore] 검증 sdkconfig 복원 완료 ($SdkconfigFile)" -ForegroundColor Green
        } else { Write-Host "[restore] $bak 없음" -ForegroundColor Red }
    }
    "build" {
        Invoke-Idf -IdfArgList (@() + $IdfArgs + @("build"))
    }
    "menuconfig" {
        Invoke-Idf -IdfArgList (@() + $IdfArgs + @("menuconfig")) -Interactive
    }
    "flash" {
        Invoke-Idf -IdfArgList (@() + $IdfArgs + @("-p", $Port, "flash"))
    }
    "monitor" {
        # esp-idf-monitor: 네이티브 USB(USB-Serial-JTAG) 리셋·재열거를 올바르게 처리.
        #  Ctrl+] 로 종료. (백그라운드 캡처 시 tee 로 logs/<board>-monitor.log 기록)
        Invoke-Idf -IdfArgList (@() + $IdfArgs + @("-p", $Port, "monitor"))
    }
    "erase" {
        Invoke-Idf -IdfArgList (@() + $IdfArgs + @("-p", $Port, "erase-flash"))
    }
    "clean" {
        Invoke-Idf -IdfArgList (@() + $IdfArgs + @("fullclean"))
    }
    "ota-image" {
        # 보드별 Matter OTA 이미지(.ota) 생성 — 디바이스가 보고하는 VID/PID/버전과
        #  일치해야 Provider 가 올바른 보드 이미지를 매칭한다.
        #  VID/PID/버전은 빌드된 보드 sdkconfig 에서 읽는다(보드별 PID 자동 반영).
        $bin = "$BuildDir/somfy_blinds.bin"
        if (-not (Test-Path $bin)) {
            Write-Host "[ota-image] $bin 없음 — 먼저 build 하세요." -ForegroundColor Red; exit 1
        }
        if (-not (Test-Path $SdkconfigFile)) {
            Write-Host "[ota-image] $SdkconfigFile 없음 — set-target/build 먼저." -ForegroundColor Red; exit 1
        }
        function Get-Cfg([string]$key) {
            $m = Select-String -Path $SdkconfigFile -Pattern "^$key=(.+)$" | Select-Object -First 1
            if ($m) { return $m.Matches[0].Groups[1].Value.Trim('"') } else { return $null }
        }
        # ★ $pid 는 PowerShell 자동 변수(프로세스 ID) — 충돌 회피 위해 $prodid 사용.
        $vid    = Get-Cfg "CONFIG_DEVICE_VENDOR_ID"
        $prodid = Get-Cfg "CONFIG_DEVICE_PRODUCT_ID"
        $ver    = Get-Cfg "CONFIG_DEVICE_SOFTWARE_VERSION_NUMBER"
        if (-not $vid -or -not $prodid -or -not $ver) {
            Write-Host "[ota-image] VID/PID/버전 파싱 실패 (sdkconfig 확인)" -ForegroundColor Red; exit 1
        }
        # ── HW 변형(variant) 태그 — 같은 PID 안에서 PCF8574/8575·EC11/EC05 빌드를
        #    구분한다. boards/<board>.h 의 플래그를 읽는다(미정의 = 기본 0). 펌웨어
        #    board_select.h 의 BOARD_HW_VARIANT_* 와 동일 규칙. (.bin 은 안 바뀜)
        function Get-BoardFlag([string]$name) {
            if (Select-String -Path $boardHeader -Pattern "^\s*#\s*define\s+$name\s+1\b" -Quiet) { 1 } else { 0 }
        }
        function Get-BoardNum([string]$name, [int]$def) {
            $m = Select-String -Path $boardHeader -Pattern "^\s*#\s*define\s+$name\s+(\d+)" | Select-Object -First 1
            if ($m) { [int]$m.Matches[0].Groups[1].Value } else { $def }
        }
        # 변형 태그 — build.ps1 -Pcf/-Rotary/-Oled/-Rotate 오버라이드가 있으면 그걸,
        # 없으면 boards/<board>.h 값을 쓴다(빌드한 변형을 .ota 태그에 동일 반영).
        # ⚠ build 때와 같은 변형 옵션을 ota-image 에도 줘야 태그가 일치한다.
        $pcf  = if ($Pcf)        { $Pcf } elseif (Get-BoardFlag "BOARD_HAS_LR_BUTTONS") { "8575" } else { "8574" }
        $enc  = if ($Rotary)     { $Rotary } elseif (Get-BoardFlag "BOARD_ROT_HALF_STEP") { "ec05" } else { "ec11" }
        $orot = if ($Rotate -ne ''){ "r$rotBase" + $(if ($rotMirror) { 'm' } else { '' }) } elseif ((Get-BoardNum 'BOARD_OLED_ROTATE_180' 1) -ne 0) { "r180" } else { "r0" }
        $ores = if ($Oled)       { $Oled } else { "$(Get-BoardNum 'BOARD_OLED_WIDTH' 72)x$(Get-BoardNum 'BOARD_OLED_HEIGHT' 40)" }
        $oled = "$ores$orot"
        $variant = "$pcf.$enc.$oled"
        # VERSION 은 4자리 zero-pad 문자열(예 35 → 0035). vn(Matter 숫자 비교)은 원본 유지.
        $verPad = "{0:D4}" -f [int]$ver
        New-Item -ItemType Directory -Force -Path "$PROJ/dist" | Out-Null
        $out = "$PROJ/dist/somfy_blinds_${Board}_${pcf}_${enc}_${oled}_v${verPad}.ota"
        Write-Host "[ota-image] board=$Board VID=$vid PID=$prodid ver=$verPad variant=$variant -> dist/somfy_blinds_${Board}_${pcf}_${enc}_${oled}_v${verPad}.ota" -ForegroundColor Cyan
        & python.exe "$OTA_TOOL" create -v $vid -p $prodid -vn $ver -vs "$verPad+$variant" -da sha256 $bin $out 2>&1 | Tee-Object -FilePath $LogPath
        if (Test-Path $out) {
            Write-Host "[ota-image] 생성 완료: $out ($((Get-Item $out).Length) bytes)" -ForegroundColor Green
        }
    }
}
if ($LASTEXITCODE -ne 0) { Write-Host "실패! (exit $LASTEXITCODE) — 로그: logs/$Board-$($Action.ToLower()).log" -ForegroundColor Red; exit 1 }
Write-Host "`n[somfy_blinds $Action board=$Board] 완료 (로그: logs/$Board-$($Action.ToLower()).log)" -ForegroundColor Green
