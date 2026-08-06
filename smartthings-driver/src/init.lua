--[[
  Somfy Blinds 3-Shade (composed) — SmartThings Matter Edge driver

  목적: composed Matter 기기(root EP0 + WindowCovering EP1·EP2·EP3)를 SmartThings 카드 1개 안에
        블라인드 3개 컨트롤로 노출. 스톡 matter-window-covering 은 composed 의 다중 동일-타입
        endpoint 를 1개만 매핑하므로, 여기서 profile 의 세 component(main, blind2, blind3)를 기기의
        WindowCovering endpoint 들에 직접 매핑한다.

  매핑 원리(SmartThings Matter Lua 표준):
    - set_component_to_endpoint_fn : capability 명령(cmd.component) → Matter endpoint
    - set_endpoint_to_component_fn : Matter 속성 보고(endpoint) → SmartThings component
      (device:emit_event_for_endpoint 이 이 함수를 사용해 올바른 component 로 이벤트 발생)

  좌표 변환: Matter CurrentPositionLiftPercent100ths 는 0=완전 열림 / 10000=완전 닫힘,
            SmartThings shadeLevel 은 0=닫힘 / 100=열림  →  level = 100 - value/100
            Tilt(CurrentPositionTiltPercent100ths ↔ shadeTiltLevel)도 동일 패턴.
]]

local capabilities = require "st.capabilities"
local clusters     = require "st.matter.clusters"
local MatterDriver = require "st.matter.driver"
local log          = require "log"

local WindowCovering = clusters.WindowCovering

-- ── WindowCovering 클러스터를 가진 endpoint 목록(정렬) ────────────────────────
--   root(EP0)에는 WindowCovering 이 없으므로 블라인드 endpoint 만 정렬되어 반환된다.
--   [1] = main, [2] = blind2, [3] = blind3.
local function wc_endpoints(device)
  local eps = device:get_endpoints(WindowCovering.ID)
  table.sort(eps)
  return eps
end

local function component_to_endpoint(device, component_id)
  local eps = wc_endpoints(device)
  if component_id == "blind2" then
    return eps[2] or eps[1]
  elseif component_id == "blind3" then
    return eps[3] or eps[1]
  end
  return eps[1]               -- main (endpoint 가 1개뿐이면 그것)
end

local function endpoint_to_component(device, ep)
  local eps = wc_endpoints(device)
  if eps[3] ~= nil and ep == eps[3] then
    return "blind3"
  elseif eps[2] ~= nil and ep == eps[2] then
    return "blind2"
  end
  return "main"
end

-- ── lifecycle: 매핑 함수 등록 + 구독 ─────────────────────────────────────────
local function device_init(driver, device)
  device:set_component_to_endpoint_fn(component_to_endpoint)
  device:set_endpoint_to_component_fn(endpoint_to_component)
  device:subscribe()
end

-- ── Matter 속성 → capability(Lift 위치) ─────────────────────────────────────
local function current_position_handler(driver, device, ib, response)
  if ib.data == nil or ib.data.value == nil then return end
  local level = 100 - math.floor(ib.data.value // 100)   -- 0..100 (열림%)
  if level < 0 then level = 0 elseif level > 100 then level = 100 end

  device:emit_event_for_endpoint(ib.endpoint_id,
    capabilities.windowShadeLevel.shadeLevel(level))

  local shade
  if level <= 0 then
    shade = capabilities.windowShade.windowShade.closed()
  elseif level >= 100 then
    shade = capabilities.windowShade.windowShade.open()
  else
    shade = capabilities.windowShade.windowShade.partially_open()
  end
  device:emit_event_for_endpoint(ib.endpoint_id, shade)
end

-- ── Matter 속성 → capability(Tilt 슬랫 각도) ────────────────────────────────
local function current_tilt_position_handler(driver, device, ib, response)
  if ib.data == nil or ib.data.value == nil then return end
  local level = 100 - math.floor(ib.data.value // 100)   -- 0..100
  if level < 0 then level = 0 elseif level > 100 then level = 100 end
  device:emit_event_for_endpoint(ib.endpoint_id,
    capabilities.windowShadeTiltLevel.shadeTiltLevel(level))
end

-- ── capability 명령 → Matter ────────────────────────────────────────────────
local function handle_open(driver, device, cmd)
  local ep = component_to_endpoint(device, cmd.component)
  device:send(WindowCovering.server.commands.UpOrOpen(device, ep))
end

local function handle_close(driver, device, cmd)
  local ep = component_to_endpoint(device, cmd.component)
  device:send(WindowCovering.server.commands.DownOrClose(device, ep))
end

local function handle_pause(driver, device, cmd)
  local ep = component_to_endpoint(device, cmd.component)
  device:send(WindowCovering.server.commands.StopMotion(device, ep))
end

local function handle_set_shade_level(driver, device, cmd)
  local ep    = component_to_endpoint(device, cmd.component)
  local level = cmd.args.shadeLevel                 -- 0..100 (열림%)
  local hundredths = (100 - level) * 100            -- → Matter lift 100ths(0=열림)
  device:send(WindowCovering.server.commands.GoToLiftPercentage(device, ep, hundredths))
end

local function handle_set_tilt_level(driver, device, cmd)
  local ep    = component_to_endpoint(device, cmd.component)
  -- ★ windowShadeTiltLevel.setShadeTiltLevel 의 인자명은 'level' (lift 는 'shadeLevel').
  local level = cmd.args.level or cmd.args.shadeTiltLevel   -- 0..100
  if level == nil then return end
  local hundredths = (100 - level) * 100            -- → Matter tilt 100ths
  device:send(WindowCovering.server.commands.GoToTiltPercentage(device, ep, hundredths))
end

local function handle_refresh(driver, device, cmd)
  device:send(WindowCovering.attributes.CurrentPositionLiftPercent100ths:read(device))
  device:send(WindowCovering.attributes.CurrentPositionTiltPercent100ths:read(device))
end

-- ── driver template ─────────────────────────────────────────────────────────
local somfy_blinds_template = {
  lifecycle_handlers = {
    init = device_init,
  },
  matter_handlers = {
    attr = {
      [WindowCovering.ID] = {
        [WindowCovering.attributes.CurrentPositionLiftPercent100ths.ID] = current_position_handler,
        [WindowCovering.attributes.CurrentPositionTiltPercent100ths.ID] = current_tilt_position_handler,
      },
    },
  },
  subscribed_attributes = {
    [capabilities.windowShade.ID] = {
      WindowCovering.attributes.CurrentPositionLiftPercent100ths,
    },
    [capabilities.windowShadeLevel.ID] = {
      WindowCovering.attributes.CurrentPositionLiftPercent100ths,
    },
    [capabilities.windowShadeTiltLevel.ID] = {
      WindowCovering.attributes.CurrentPositionTiltPercent100ths,
    },
  },
  capability_handlers = {
    [capabilities.windowShade.ID] = {
      [capabilities.windowShade.commands.open.NAME]  = handle_open,
      [capabilities.windowShade.commands.close.NAME] = handle_close,
      [capabilities.windowShade.commands.pause.NAME] = handle_pause,
    },
    [capabilities.windowShadeLevel.ID] = {
      [capabilities.windowShadeLevel.commands.setShadeLevel.NAME] = handle_set_shade_level,
    },
    [capabilities.windowShadeTiltLevel.ID] = {
      [capabilities.windowShadeTiltLevel.commands.setShadeTiltLevel.NAME] = handle_set_tilt_level,
    },
    [capabilities.refresh.ID] = {
      [capabilities.refresh.commands.refresh.NAME] = handle_refresh,
    },
  },
}

local somfy_driver = MatterDriver("somfy-blinds-3", somfy_blinds_template)
somfy_driver:run()
