# -*- coding: utf-8 -*-
"""ADC 유닛 수명 — "절전에 들 때 유닛이 살아 있으면 안 된다"를 검증한다.

왜 필요한가 (2026-10-13)
────────────────────────
얕은잠 중 MODEM·TOP 도메인이 한 번도 꺼지지 않았다. 범인은 IDF ADC 드라이버였다:
    esp_adc/adc_oneshot.c  adc_oneshot_new_unit()
      #elif ADC_LL_ADC_FE_ON_MODEM_DOMAIN       (C6 = 1)
        esp_sleep_pd_config(ESP_PD_DOMAIN_MODEM, ESP_PD_OPTION_ON)
유닛이 **존재하는 동안** 모뎀 도메인이 절전 중에도 켜진다. del_unit 이 풀어준다.
펌웨어는 배터리 전압용 유닛을 부팅 때 만들어 영원히 살려뒀다.

수정: 잴 때만 만들고 재자마자 지운다(somfy_app.c _bat_adc_open/_bat_adc_close).
콘솔 진단(intpd)이 다른 태스크에서 같은 유닛을 쓰므로 참조 카운트로 묶는다.

이 시뮬레이터가 지키는 불변식
  ① 아무도 안 쓰는 순간에는 유닛이 없다  → MODEM 도메인을 끌 수 있다
  ② 누가 쓰는 중에는 다른 쪽이 close 해도 유닛이 안 사라진다 (사용 중 삭제 = 크래시)
  ③ 대조군(keep)에서는 옛 동작 그대로 유닛이 상주한다

★★2026-10-13 실측으로 뒤집힘 — 위 전제("유닛이 없어야 좋다")는 **틀렸다.**
  유닛을 지워 MODEM 을 끄게 했더니 배터리 진입 28초 뒤 Thread 가 멈추고 절전이 0% 가
  됐다(4.2시간에 90→37%, 대조군은 정상). 이 시뮬레이터는 "유닛이 없어진다"는 **논리**만
  검증했고 "MODEM 을 꺼도 무선이 산다"는 **가정**은 검증하지 않았다 — 통과가 안전을
  뜻하지 않았다. 하드웨어 가정은 시뮬레이터로 못 잰다.
  → 펌웨어 기본값을 상주(keep)로 되돌렸고, 아래 ⑤ 가 그 기본값을 지킨다:
  ⑤ NVS 키가 없는 보드는 반드시 상주 모드다 (실험 모드는 명시적으로 켰을 때만)
"""
import sys

DEFAULT_FIX = 0      # somfy_app.c _bat_adc_init: `uint8_t fix = 0;` (2026-10-13 1 → 0)


def keep_from_nvs(nvs):
    """_bat_adc_init 의 A/B 스위치 — NVS diag/adcfix 가 없으면 기본값을 쓴다."""
    return nvs.get('adcfix', DEFAULT_FIX) == 0


class Adc:
    def __init__(self, keep=False):
        self.unit = None          # None = 유닛 없음
        self.ref = 0
        self.keep = keep
        self.created = 0          # new_unit 호출 횟수

    def init(self):               # _bat_adc_init
        self.unit = object(); self.created += 1
        if not self.keep:
            self.unit = None

    def open(self):               # _bat_adc_open
        if self.unit is None:
            self.unit = object(); self.created += 1
        self.ref += 1
        return True

    def close(self):              # _bat_adc_close
        if self.ref > 0:
            self.ref -= 1
        if self.ref == 0 and not self.keep and self.unit is not None:
            self.unit = None

    def modem_can_power_down(self):
        return self.unit is None  # 유닛이 있으면 IDF 가 MODEM 을 강제 ON


def main():
    # ① 수정: 초기화 뒤, 그리고 매 측정 뒤 유닛이 없어야 한다
    a = Adc(); a.init()
    assert a.modem_can_power_down(), '초기화 직후 유닛이 남으면 첫 절전부터 MODEM 이 켜진다'
    for _ in range(1000):                       # 5초마다 측정
        a.open(); assert a.unit is not None
        a.close(); assert a.modem_can_power_down(), '측정 뒤 유닛이 남았다'
    assert a.ref == 0

    # ② 진단(intpd)이 잡고 있는 동안 배터리 측정이 끼어들어도 유닛이 살아 있어야 한다
    a = Adc(); a.init()
    a.open()                                    # intpd 시작
    a.open(); a.close()                         # 그 사이 배터리 측정 1회
    assert a.unit is not None, '★사용 중인 유닛을 지웠다 — 진단이 NULL 핸들로 크래시'
    a.close()                                   # intpd 끝
    assert a.modem_can_power_down() and a.ref == 0

    # ③ close 가 더 많이 불려도 음수로 안 간다
    a = Adc(); a.init(); a.close(); a.close()
    assert a.ref == 0 and a.modem_can_power_down()

    # ④ 대조군: 유닛 상주 → MODEM 못 끔 (옛 동작 재현)
    k = Adc(keep=True); k.init()
    k.open(); k.close()
    assert not k.modem_can_power_down(), '대조군은 유닛이 남아야 비교가 된다'
    assert k.created == 1, '대조군은 유닛을 다시 만들지 않는다'

    # ⑤ 기본값 — NVS 키가 없는 보드(새 보드, NVS 초기화 뒤)는 반드시 상주여야 한다
    assert keep_from_nvs({}) is True, '★NVS 키가 없을 때 실험 모드로 떨어지면 배터리에서 Thread 가 멈춘다'
    assert keep_from_nvs({'adcfix': 0}) is True
    assert keep_from_nvs({'adcfix': 1}) is False      # 실험은 명시적으로 켰을 때만
    d = Adc(keep=keep_from_nvs({})); d.init(); d.open(); d.close()
    assert not d.modem_can_power_down(), '기본 설정에서는 유닛이 상주해 MODEM 이 켜져 있어야 한다'

    print('ADC 유닛 수명 5케이스 통과 — 기본: 상주(안전) / 실험(adcfix=1): 측정 사이 유닛 없음')
    return 0


if __name__ == '__main__':
    sys.stdout.reconfigure(encoding='utf-8', errors='replace')
    sys.exit(main())
