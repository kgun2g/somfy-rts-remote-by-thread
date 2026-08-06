# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)

if(NOT DEFINED ENV{ESP_MATTER_PATH})
    message(FATAL_ERROR "Please set ESP_MATTER_PATH to the path of esp-matter repo")
endif(NOT DEFINED ENV{ESP_MATTER_PATH})

if(NOT DEFINED ENV{ESP_SSD1306_PATH})
    message(FATAL_ERROR "Please set ESP_SSD1306_PATH to the path of esp-idf-ssd1306 repo")
endif(NOT DEFINED ENV{ESP_SSD1306_PATH})

# 이 프로젝트는 ESP32-C6 전용이다.
if(NOT DEFINED ENV{ESP_MATTER_DEVICE_PATH})
    set(ENV{ESP_MATTER_DEVICE_PATH} $ENV{ESP_MATTER_PATH}/device_hal/device/esp32c6_devkit_c)
endif(NOT DEFINED ENV{ESP_MATTER_DEVICE_PATH})

set(PROJECT_VER "3.5")
set(PROJECT_VER_NUMBER 35)

set(ESP_MATTER_PATH $ENV{ESP_MATTER_PATH})
set(MATTER_SDK_PATH ${ESP_MATTER_PATH}/connectedhomeip/connectedhomeip)
# 시스템 환경변수가 백슬래시(D:\...)일 수 있어 CMake 경로(forward slash)로 정규화.
file(TO_CMAKE_PATH "$ENV{ESP_SSD1306_PATH}" ESP_SSD1306_PATH)

# esp-idf / esp-matter SDK 빌드 글루(외부 의존성). 아래 경로의 일부는
# esp-matter SDK 가 공용 빌드 스크립트를 자체 트리에 두고 있어 불가피하다
# (IDF 의 tools/cmake/project.cmake 와 동일한 SDK 내부 경로 성격).
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
include(${ESP_MATTER_PATH}/examples/common/cmake_common/components_include.cmake)
include($ENV{ESP_MATTER_DEVICE_PATH}/esp_matter_device.cmake)

set(EXTRA_COMPONENT_DIRS
    "${ESP_MATTER_PATH}/examples/common"
    "${MATTER_SDK_PATH}/config/esp32/components"
    "${ESP_MATTER_PATH}/components"
    "${ESP_MATTER_PATH}/device_hal/device"
    "${ESP_SSD1306_PATH}/components"
    ${extra_components_dirs_append})

project(somfy_blinds)

idf_build_set_property(CXX_COMPILE_OPTIONS "-std=gnu++17;-Os;-DCHIP_HAVE_CONFIG_H;-Wno-overloaded-virtual" APPEND)
idf_build_set_property(C_COMPILE_OPTIONS "-Os" APPEND)
# For RISCV chips, project_include.cmake sets -Wno-format, but does not clear various
# flags that depend on -Wformat
idf_build_set_property(COMPILE_OPTIONS "-Wno-format-nonliteral;-Wno-format-security" APPEND)
