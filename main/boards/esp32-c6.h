#pragma once
/*
 * boards/esp32-c6.h  — DEPRECATED 별칭
 * ──────────────────────────────────────────────────────────
 * 구 "esp32-c6" board 키는 브랜드 분리(2026-06)로 "gnpe-c6" 로 이름이 바뀜.
 * 이 파일은 backward 호환을 위해 GNPE 핀맵을 그대로 include 한다.
 * 신규 작업은 boards/gnpe-c6.h (또는 boards/xiao-c6.h) 를 직접 사용할 것.
 * ──────────────────────────────────────────────────────────
 */
#include "boards/gnpe-c6.h"
