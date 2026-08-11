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
        BOARD_OVR_OLED_ROTATE_90 BOARD_OVR_FREQ)
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
