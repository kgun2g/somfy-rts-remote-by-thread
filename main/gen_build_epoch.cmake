# 빌드 시각(UTC Unix epoch)을 매 빌드마다 새로 기록하는 헤더 생성.
# ★ __DATE__/__TIME__ 은 해당 매크로가 든 .c 가 재컴파일될 때만 갱신되어
#   다른 파일만 고친 증분 빌드에서 시드가 고정되는 문제가 있었다.
#   이 스크립트는 add_custom_target(ALL) 로 매 빌드 무조건 실행되어
#   항상 현재 시각으로 build_epoch.h 를 다시 쓴다.
string(TIMESTAMP BUILD_EPOCH "%s" UTC)
set(_content "#pragma once\n#define BUILD_EPOCH_UNIX ${BUILD_EPOCH}LL\n")
set(_need_write TRUE)
if(EXISTS "${OUT}")
    file(READ "${OUT}" _old)
    if(_old STREQUAL _content)
        set(_need_write FALSE)
    endif()
endif()
if(_need_write)
    file(WRITE "${OUT}" "${_content}")
endif()
