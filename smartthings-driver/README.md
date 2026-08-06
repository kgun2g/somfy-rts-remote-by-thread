# Somfy Blinds 3-Shade — SmartThings custom Edge driver

composed Matter 기기(root EP0 + WindowCovering EP1·EP2·EP3)를 **SmartThings 카드 1개 안에 블라인드 3개
컨트롤**(component `main` / `blind2` / `blind3`)로 노출한다. 스톡 `matter-window-covering` driver 가
composed 의 다중 동일-타입 endpoint 를 1개만 매핑하는 한계를 우회한다.

> 펌웨어는 이미 WindowCovering endpoint 3개를 정상 노출한다(Apple/Google Home 은 추가 작업 없이 3개가
> 보인다). 이 driver 는 **SmartThings 쪽 매핑**만 보완한다.

## 구조
```
smartthings-driver/
  config.yml                 # driver 메타(name, packageKey, matter 권한)
  fingerprints.yml           # VID 0xFFF1 / PID 0x8001 → profile 자동 할당
  profiles/somfy-blinds-3.yml# component 3개(main, blind2, blind3) 정의
  src/init.lua               # endpoint ↔ component 매핑 + WindowCovering 핸들러
```

## 배포 (SmartThings CLI)

> ⚠️ **인자 주의** — 명령마다 인자가 다르다:
> - `edge:channels:assign`  인자 = **driverId** (채널은 대화형/`--channel`)
> - `edge:channels:enroll`  인자 = **hubId**    (채널은 대화형/`--channel`)
> - `edge:drivers:install`  인자 = **driverId** (허브는 대화형/`--hub`)
>
> **`channelId` 를 인자로 주면 안 된다** — driverId/hubId 자리로 잘못 해석돼 `404 Missing driver`
> 또는 `500` 이 난다. 헷갈리면 **인자 없이** 실행해 대화형으로 고르는 게 가장 안전하다.

```bash
# 0) CLI 설치 (Node 필요)
npm install -g @smartthings/cli

# 1) 패키지화(빌드+업로드) — 끝에 Driver Id 가 출력된다(메모)
smartthings edge:drivers:package ./smartthings-driver

# 2) 배포 채널 생성 — Channel Id 가 출력된다
#  Channel name : somfy-rts-thread
#  Channel description : somfy-rts-thread smartthings
#  url : xxxxx.com, example.com
smartthings edge:channels:create

# 3) 채널에 driver 할당  (인자 없이 → driver 선택 → 채널 선택)
smartthings edge:channels:assign
#   (인자를 줄 거면 driverId 만: smartthings edge:channels:assign <driverId>)

# 4) 내 허브를 채널에 등록  (인자 없이 → 허브 선택 → 채널 선택)
smartthings edge:channels:enroll
#   (인자를 줄 거면 hubId 만: smartthings edge:channels:enroll <hubId>)

# 5) 허브에 driver 설치  (인자 없이 → 허브 선택 → driver 선택)
smartthings edge:drivers:install
#   (인자를 줄 거면 driverId 만: smartthings edge:drivers:install <driverId>)
```

## 기기에 driver 적용
- **이미 페어링된 기기**: SmartThings 앱 → 해당 블라인드 기기 → 점 3개(설정) → **Driver** →
  목록에서 `Somfy Blinds 3-Shade (composed)` 선택. 카드가 3-component 로 다시 그려진다.
- 또는 기기를 삭제하고 재페어링하면 fingerprint(VID/PID)로 이 profile 이 자동 적용된다.

## 동작
- `main` → 첫 WindowCovering endpoint, `blind2` → 둘째, `blind3` → 셋째 (정렬 순).
- 명령: 열기/닫기/정지(`windowShade`) + 위치 %(`windowShadeLevel.setShadeLevel`) + **틸트 %(`windowShadeTiltLevel.setShadeTiltLevel`, arg=`level`)**.
- 위치 변환: Matter `CurrentPositionLiftPercent100ths`(0=열림,10000=닫힘) ↔ ST `shadeLevel`(0=닫힘,100=열림).

## 참고/주의
- SmartThings Matter Lua 라이브러리 버전에 따라 일부 API 명칭이 다를 수 있다. 패키지화 시 오류가 나면
  메시지에 맞춰 `src/init.lua` 를 조정한다(특히 `emit_event_for_endpoint` / `set_*_to_*_fn` / command 시그니처).
- tilt(틸트)도 **포함** — profile 의 3개 component 모두 `windowShadeTiltLevel` capability 선언 +
  `src/init.lua` 에 `CurrentPositionTiltPercent100ths` 구독 / `GoToTiltPercentage` 송신 핸들러 구현
  (command arg = `level`). 펌웨어가 틸트를 7단계 step burst 로 변환한다.
- 근거: SmartThings 는 "한 기기당 driver 1개" 구조라 composed 다중 endpoint 는 커스텀 driver 로만
  3-component 노출이 가능하다(community.smartthings.com/t/composite-matter-device/283091).
