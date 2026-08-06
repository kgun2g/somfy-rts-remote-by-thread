#!/usr/bin/env pwsh
# Somfy 테스트 빌드/플래시 (재사용). ESP32-C6 전용 v3.5.
#  - 정상 build/ 와 분리된 build_test/ 에 산출(테스트 펌웨어 보존·재활용).
#  - 프로젝트 루트의 검증된 sdkconfig 를 그대로 공유(재생성 안 함).
#  - -Mode virtual : -DSOMFY_SELFTEST=1   → 부팅 직후 블라인드 1~5 가상
#                     프레임 검증(CC1101/RF 불필요).
#  - -Mode onair   : -DSOMFY_ONAIR_TEST=1 → 부팅 직후 실제 CC1101 로
#                     주파수 변경/버튼/블라인드/ALL 온에어 RF 송신 검증.
#  두 모드는 상호 배타(택일) — 같은 build_test/ 에 산출.
# 사용: ./test/build_test.ps1 [-Action build|flash] [-Mode virtual|onair] [-Port COMx]
param(
    [string]$Action = "build",
    [string]$Mode   = "virtual",
    [string]$Port   = "COM3"
)

# 두 정의를 항상 함께 전달(미선택 모드는 0) → build_test/ 의 CMake 캐시가
# 이전 모드 값을 묵혀 두 테스트가 동시 컴파일되는 것을 방지(상호 배타 보장).
switch ($Mode.ToLower()) {
    "virtual"  { $TestDef = @("-DSOMFY_SELFTEST=1","-DSOMFY_ONAIR_TEST=0","-DSOMFY_STRESS_TEST=0","-DSOMFY_RXDECODE_TEST=0","-DSOMFY_TXPROBE_TEST=0","-DSOMFY_RXBYTE_TEST=0","-DSOMFY_TXDECODE_TEST=0","-DSOMFY_CWTEST_TEST=0") }
    "onair"    { $TestDef = @("-DSOMFY_SELFTEST=0","-DSOMFY_ONAIR_TEST=1","-DSOMFY_STRESS_TEST=0","-DSOMFY_RXDECODE_TEST=0","-DSOMFY_TXPROBE_TEST=0","-DSOMFY_RXBYTE_TEST=0","-DSOMFY_TXDECODE_TEST=0","-DSOMFY_CWTEST_TEST=0") }
    "stress"   { $TestDef = @("-DSOMFY_SELFTEST=0","-DSOMFY_ONAIR_TEST=0","-DSOMFY_STRESS_TEST=1","-DSOMFY_RXDECODE_TEST=0","-DSOMFY_TXPROBE_TEST=0","-DSOMFY_RXBYTE_TEST=0","-DSOMFY_TXDECODE_TEST=0","-DSOMFY_CWTEST_TEST=0") }
    "rxdecode" { $TestDef = @("-DSOMFY_SELFTEST=0","-DSOMFY_ONAIR_TEST=0","-DSOMFY_STRESS_TEST=0","-DSOMFY_RXDECODE_TEST=1","-DSOMFY_TXPROBE_TEST=0","-DSOMFY_RXBYTE_TEST=0","-DSOMFY_TXDECODE_TEST=0","-DSOMFY_CWTEST_TEST=0") }
    "txprobe"  { $TestDef = @("-DSOMFY_SELFTEST=0","-DSOMFY_ONAIR_TEST=0","-DSOMFY_STRESS_TEST=0","-DSOMFY_RXDECODE_TEST=0","-DSOMFY_TXPROBE_TEST=1","-DSOMFY_RXBYTE_TEST=0","-DSOMFY_TXDECODE_TEST=0","-DSOMFY_CWTEST_TEST=0") }
    "rxbyte"   { $TestDef = @("-DSOMFY_SELFTEST=0","-DSOMFY_ONAIR_TEST=0","-DSOMFY_STRESS_TEST=0","-DSOMFY_RXDECODE_TEST=0","-DSOMFY_TXPROBE_TEST=0","-DSOMFY_RXBYTE_TEST=1","-DSOMFY_TXDECODE_TEST=0","-DSOMFY_CWTEST_TEST=0") }
    "txdecode" { $TestDef = @("-DSOMFY_SELFTEST=0","-DSOMFY_ONAIR_TEST=0","-DSOMFY_STRESS_TEST=0","-DSOMFY_RXDECODE_TEST=0","-DSOMFY_TXPROBE_TEST=0","-DSOMFY_RXBYTE_TEST=0","-DSOMFY_TXDECODE_TEST=1","-DSOMFY_CWTEST_TEST=0") }
    "cwtest"   { $TestDef = @("-DSOMFY_SELFTEST=0","-DSOMFY_ONAIR_TEST=0","-DSOMFY_STRESS_TEST=0","-DSOMFY_RXDECODE_TEST=0","-DSOMFY_TXPROBE_TEST=0","-DSOMFY_RXBYTE_TEST=0","-DSOMFY_TXDECODE_TEST=0","-DSOMFY_CWTEST_TEST=1") }
    default   { Write-Host "알 수 없는 -Mode '$Mode' (virtual|onair|stress|rxdecode|txprobe|rxbyte|txdecode|cwtest)" -ForegroundColor Red; exit 1 }
}

# ─── 경로는 시스템 환경 변수에서만 읽는다 (개인 경로 비노출) ──────────────
#   RTS_BLINDS_THREAD_PATH / IDF_PATH / ESP_MATTER_PATH / ESP_SSD1306_PATH 를
#   시스템 환경 변수에 등록. 기존 셸이면 Machine 스코프에서 보충한다.
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
function ConvertTo-FSlash([string]$p) { $p -replace '\\','/' }

$IDF_PATH    = ConvertTo-FSlash $env:IDF_PATH
$MATTER_PATH = ConvertTo-FSlash $env:ESP_MATTER_PATH
$RISCV_GCC   = "$env:USERPROFILE\.espressif\tools\riscv32-esp-elf\esp-14.2.0_20241119\riscv32-esp-elf\bin"
$CMAKE_BIN   = "$env:USERPROFILE\.espressif\tools\cmake\3.30.2\bin"
$NINJA_BIN   = "$env:USERPROFILE\.espressif\tools\ninja\1.12.1"
$PIGWEED_BIN = "$MATTER_PATH/connectedhomeip/connectedhomeip/third_party/pigweed/repo/environment/cipd/packages/pigweed"
$VENV        = "$env:USERPROFILE\.espressif\python_env\idf5.4_py3.14_env\Scripts"
$ROM_ELF_DIR = "$env:USERPROFILE\.espressif\tools\esp-rom-elfs\20241011"
$PW_ENV_ROOT = "$MATTER_PATH/connectedhomeip/connectedhomeip/third_party/pigweed/repo/environment"
$PROJ        = ConvertTo-FSlash $env:RTS_BLINDS_THREAD_PATH
$BUILDDIR    = "$PROJ/build_test"

Remove-Item Env:MSYSTEM -ErrorAction SilentlyContinue
$env:PATH                   = "$VENV;$RISCV_GCC;$CMAKE_BIN;$NINJA_BIN;$PIGWEED_BIN;$env:PATH"
$env:IDF_PATH               = $IDF_PATH
$env:ESP_MATTER_PATH        = $MATTER_PATH
$env:ESP_MATTER_DEVICE_PATH = "$MATTER_PATH/device_hal/device/esp32c6_devkit_c"
$env:IDF_TOOLS_PATH         = "$env:USERPROFILE\.espressif"
$env:IDF_PYTHON_ENV_PATH    = "$env:USERPROFILE\.espressif\python_env\idf5.4_py3.14_env"
$env:ESP_ROM_ELF_DIR        = $ROM_ELF_DIR
$env:_PW_ACTUAL_ENVIRONMENT_ROOT = $PW_ENV_ROOT
$env:PYTHONIOENCODING       = "utf-8"
$env:SDKCONFIG_DEFAULTS = "sdkconfig.defaults;sdkconfig.defaults.c6_thread"

Set-Location $PROJ

switch ($Action.ToLower()) {
    "build" {
        Write-Host "[테스트 모드: $Mode → $($TestDef -join ' ')]" -ForegroundColor Cyan
        & python.exe "$IDF_PATH\tools\idf.py" -B "$BUILDDIR" $TestDef `
            -D SDKCONFIG_DEFAULTS="sdkconfig.defaults;sdkconfig.defaults.c6_thread" build
    }
    "flash" {
        & python.exe "$IDF_PATH\tools\idf.py" -B "$BUILDDIR" -p $Port flash
    }
}
if ($LASTEXITCODE -ne 0) { Write-Host "테스트 빌드 실패! (exit $LASTEXITCODE)" -ForegroundColor Red; exit 1 }
Write-Host "`n[somfy 테스트 $Mode/$Action] 완료 (build_test/)" -ForegroundColor Green
