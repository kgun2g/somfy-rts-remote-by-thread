# 테스트 하네스 — test/build_test.ps1 의 -Mode 로 택일 빌드(둘 다 별도 빌드).
#   -DSOMFY_SELFTEST=1   : 가상 프레임 검증 (CC1101/RF 불필요)
#   -DSOMFY_ONAIR_TEST=1 : 실제 CC1101 온에어 RF 송신 검증
set(_test_srcs "")
set(_test_incs "")
if(SOMFY_SELFTEST)
    list(APPEND _test_srcs "../test/somfy_cases/somfy_selftest.c")
    set(_test_incs "../test/somfy_cases")
endif()
if(SOMFY_ONAIR_TEST)
    list(APPEND _test_srcs "../test/somfy_cases/somfy_onair_test.c")
    set(_test_incs "../test/somfy_cases")
endif()
if(SOMFY_STRESS_TEST)
    list(APPEND _test_srcs "../test/somfy_cases/somfy_stress_test.c")
    set(_test_incs "../test/somfy_cases")
endif()
if(SOMFY_RXDECODE_TEST)
    list(APPEND _test_srcs "../test/somfy_cases/somfy_rxdecode_test.c")
    set(_test_incs "../test/somfy_cases")
endif()
if(SOMFY_TXPROBE_TEST)
    list(APPEND _test_srcs "../test/somfy_cases/somfy_txprobe_test.c")
    set(_test_incs "../test/somfy_cases")
endif()
if(SOMFY_RXBYTE_TEST)
    list(APPEND _test_srcs "../test/somfy_cases/somfy_rxbyte_test.c")
    set(_test_incs "../test/somfy_cases")
endif()
if(SOMFY_TXDECODE_TEST)
    list(APPEND _test_srcs "../test/somfy_cases/somfy_txdecode_test.c")
    set(_test_incs "../test/somfy_cases")
endif()
if(SOMFY_CWTEST_TEST)
    list(APPEND _test_srcs "../test/somfy_cases/somfy_cwtest_test.c")
    set(_test_incs "../test/somfy_cases")
endif()
# 2026-07-22 OLED 단독 부팅 테스트 (app_main 최상단 단락 — 다른 기능 전부 비활성)
if(OLED_ONLY_TEST)
    list(APPEND _test_srcs "../test/somfy_cases/oled_only_test.c")
    set(_test_incs "../test/somfy_cases")
endif()

idf_component_register(SRCS "app_main.cpp" "app_driver.cpp"
                            "cc1101.c" "somfy_rts.c" "blind_manager.c"
                            "matter_blinds_shim.cpp" "somfy_app.c"
                            "oled_ui.c" "button_handler.c" "thread_provision.c"
                            "efuse_commissionable.cpp" "boot_diag.c"
                            ${_test_srcs}
                       PRIV_INCLUDE_DIRS  "." "boards" ${_test_incs})
# ※`REQUIRES ulp` 를 여기 넣지 말 것. `main` 은 원래 **모든 컴포넌트를 자동 의존**하는데
#   REQUIRES 를 한 번이라도 명시하면 그 자동 동작이 꺼져 esp_matter 등이 통째로 끊긴다
#   (실제로 "efuse_commissionable.h ... esp_matter is not in the requirements list" 로 실패).
#   ulp 헤더도 자동 의존으로 이미 보인다.

# ─── ★2026-08-13 (③) LP 코어 PCF8574 폴링 프로그램 ────────────────────────
#   LP 코어가 PCF 를 폴링하고 로터리 쿼드러처까지 디코딩한다(main/lp_core/pcf_poll.c).
#   HP 는 공유 RAM 만 읽으므로 I2C 비트뱅이 사라지고 깨우기 주기를 늘릴 수 있다.
#
#   ★보드별 분리 — LP_I2C 전용핀(C6 고정 6/7)에 PCF 가 물린 보드에서만 빌드한다.
#     · xiao-c6 : BOARD_PCF_LP_CORE=1 → LP 프로그램 포함
#     · esp32-h2: LP 코어/LP_I2C 자체가 없음 → 아예 빌드 안 함
#     · 공유 I2C(22/23) 로 폴백된 개체 → 바이너리엔 있지만 런타임 판정에서 안 씀
#   즉 이 블록이 빠져도 나머지 코드는 그대로 컴파일된다(button_handler.c 가
#   BOARD_PCF_LP_CORE 로 가드).
set(_lp_core_enabled 0)
if(BOARD STREQUAL "xiao-c6")
    set(_lp_core_enabled 1)
endif()
# ★2026-08-16 -Lp off 로 끌 수 있게 한다.
#  왜: LP 프로그램은 lp_core/pcf_lp_config.h 의 LP_PCF_NBYTES 가 **2 고정**이라
#  PCF8574(1바이트) 개체에서는 button_handler.c 의 _Static_assert 가 빌드를 깬다.
#  ExternalProject 라 -D 로 폭을 넘길 수도 없다(위 주석 참조).
#  → LP 를 안 쓰는 개체(공유 I2C 배선 등)는 아예 빌드에서 빼는 게 유일한 방법이다.
if(DEFINED BOARD_OVR_PCF_LP_CORE AND "${BOARD_OVR_PCF_LP_CORE}" STREQUAL "0")
    set(_lp_core_enabled 0)
    message(STATUS "LP core: -Lp off 로 비활성")
endif()
if(_lp_core_enabled AND CONFIG_ULP_COPROC_TYPE_LP_CORE)
    set(ulp_app_name lp_core_${COMPONENT_NAME})
    set(ulp_lp_core_sources "lp_core/pcf_poll.c")
    # 아래 파일들이 자동 생성 헤더(${ulp_app_name}.h)를 include 한다.
    set(ulp_exp_dep_srcs "button_handler.c")
    ulp_embed_binary(${ulp_app_name} "${ulp_lp_core_sources}" "${ulp_exp_dep_srcs}")
    # ★읽기 폭(PCF8574 1바이트 / PCF8575 2바이트)은 lp_core/pcf_lp_config.h 에 두고
    #   LP·HP 가 **같이 include** 한다. 빌드 정의(-D)로는 못 넘긴다 —
    #   ulp_embed_binary 의 타깃은 ExternalProject 라 target_compile_options 가
    #   "non-compilable target type" 으로 실패한다(실제로 겪음).
    #   어긋남은 button_handler.c 의 _Static_assert 가 빌드에서 잡는다.
    target_compile_definitions(${COMPONENT_LIB} PRIVATE SOMFY_LP_CORE_BUILT=1)
    message(STATUS "LP core PCF poll program: enabled (${ulp_app_name})")
else()
    message(STATUS "LP core PCF poll program: disabled (board=${BOARD})")
endif()

# ─── 보드 선택 (-DBOARD=esp32-c6 등) ─────────────────────────
#   build.ps1 가 cmake 호출 시 BOARD=<name> 캐시 변수 전달.
#   기본값 esp32-c6 (현 PCB v3.0). 매핑은 boards/board_select.h 참고.
if(NOT DEFINED BOARD)
    set(BOARD "esp32-c6" CACHE STRING "Target board name (esp32-c6/gnpe-c6, xiao-c6, esp32-h2)")
endif()
string(TOUPPER "${BOARD}" _BOARD_UPPER)
string(REPLACE "-" "_" _BOARD_MACRO "${_BOARD_UPPER}")
target_compile_definitions(${COMPONENT_LIB} PRIVATE "BOARD_${_BOARD_MACRO}=1")
message(STATUS "Building for board: ${BOARD} (compile define BOARD_${_BOARD_MACRO})")

# ─── 빌드타임 변형 오버라이드 (build.ps1 -Pcf/-Rotary/-Oled/-Rotate) ───
#   build.ps1 가 -D BOARD_OVR_*=값 캐시 변수를 넘기면 컴파일 정의로 전달한다.
#   boards/board_select.h 가 이를 보고 보드 기본값(PCF/로터리/OLED)을 덮어쓴다.
#   하나도 안 주면 보드 기본 그대로(바이너리 동일).
foreach(_ovr
        BOARD_OVR_HAS_LR_BUTTONS BOARD_OVR_ROT_HALF_STEP
        BOARD_OVR_OLED_WIDTH BOARD_OVR_OLED_HEIGHT BOARD_OVR_OLED_COL_OFFSET
        BOARD_OVR_OLED_FIXUP BOARD_OVR_OLED_ROTATE_180 BOARD_OVR_OLED_FLIP_X
        BOARD_OVR_OLED_ROTATE_90 BOARD_OVR_FREQ
        BOARD_OVR_PCF_LP_CORE BOARD_OVR_HAS_BAT_ADC BOARD_OVR_ROT_AB_SWAP)
    if(DEFINED ${_ovr} AND NOT "${${_ovr}}" STREQUAL "")
        target_compile_definitions(${COMPONENT_LIB} PRIVATE "${_ovr}=${${_ovr}}")
        message(STATUS "Variant override: ${_ovr}=${${_ovr}}")
    endif()
endforeach()

if(SOMFY_SELFTEST)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE SOMFY_SELFTEST=1)
endif()
if(SOMFY_ONAIR_TEST)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE SOMFY_ONAIR_TEST=1)
endif()
if(SOMFY_STRESS_TEST)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE SOMFY_STRESS_TEST=1)
endif()
if(SOMFY_RXDECODE_TEST)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE SOMFY_RXDECODE_TEST=1)
endif()
if(SOMFY_TXPROBE_TEST)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE SOMFY_TXPROBE_TEST=1)
endif()
if(SOMFY_RXBYTE_TEST)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE SOMFY_RXBYTE_TEST=1)
endif()
if(SOMFY_TXDECODE_TEST)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE SOMFY_TXDECODE_TEST=1)
endif()
if(SOMFY_CWTEST_TEST)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE SOMFY_CWTEST_TEST=1)
endif()
if(OLED_ONLY_TEST)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE OLED_ONLY_TEST=1)
endif()

if (CONFIG_ENABLE_SET_CERT_DECLARATION_API)
    target_add_binary_data(${COMPONENT_TARGET} "certification_declaration/certification_declaration.der" BINARY)
endif()

target_compile_options(${COMPONENT_LIB} PRIVATE "-DCHIP_HAVE_CONFIG_H")

# ★ 시각 동기화 근본 수정: 빌드마다 갱신되는 build_epoch.h 생성.
#   - configure 시 1회 생성(include 보장)
#   - 매 빌드(ALL) gen_build_epoch.cmake 재실행 → 항상 최신 epoch
#     (다른 파일만 고친 증분 빌드에서도 시드가 절대 안 묵음)
set(BUILD_EPOCH_HEADER "${CMAKE_CURRENT_BINARY_DIR}/build_epoch.h")
execute_process(COMMAND ${CMAKE_COMMAND}
    -DOUT=${BUILD_EPOCH_HEADER}
    -P ${CMAKE_CURRENT_LIST_DIR}/gen_build_epoch.cmake)
add_custom_target(gen_build_epoch ALL
    BYPRODUCTS "${BUILD_EPOCH_HEADER}"
    COMMAND ${CMAKE_COMMAND}
        -DOUT=${BUILD_EPOCH_HEADER}
        -P ${CMAKE_CURRENT_LIST_DIR}/gen_build_epoch.cmake
    COMMENT "Regenerating fresh build_epoch.h (build-time UTC epoch)"
    VERBATIM)
add_dependencies(${COMPONENT_LIB} gen_build_epoch)
target_include_directories(${COMPONENT_LIB} PRIVATE "${CMAKE_CURRENT_BINARY_DIR}")

# 대형 C 소스(oled_ui/somfy_app/button_handler 등)의 사소한 미사용
#   경고를 에러로 승격하지 않는다.
target_compile_options(${COMPONENT_LIB} PRIVATE
    "-Wno-error=unused-const-variable"
    "-Wno-error=unused-variable"
    "-Wno-error=unused-function"
    "-Wno-error=unused-but-set-variable")
