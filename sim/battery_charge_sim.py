#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
배터리 충전 회로 가상 시뮬레이션 + 펌웨어 충전 추정 로직 테스트
==================================================================
이 프로젝트의 실제 충전 회로를 물리 모델로 시뮬레이션하고, 펌웨어
(`somfy_app.c::_estimate_battery_percent`)의 충전량 추정 휴리스틱을
검증한다. 하드웨어 없이 동작(가상).

모델
----
1) 단셀 Li-ion(LiPo) 셀  : 용량 Q[mAh], OCV(SoC) 곡선, 등가 충전 과전압 저항 R_eff.
2) 선형 충전 IC(CC/CV)   : MCP73831(GNPE) / SGM40567-4.2(XIAO).
     - CC: 정전류 I_chg (단자전압 < 4.2V 동안)
     - CV: 단자전압 4.2V 고정, 전류 점감 → I_term 도달 시 종료(완충)
     - STAT: 충전 중 LOW(active), 완충 시 HIGH(open-drain 해제)
3) 쿨롱 카운팅으로 '실제 SoC' 계산 → 펌웨어 시간기반 추정과 비교.

보드별 실제 사양(보드 헤더/펌웨어 주석 기준)
  - GNPE  : MCP73831, 600 mAh, 300 mA(0.5C), STAT=active-LOW(IO3, GPIO 読取 가능)
  - XIAO  : SGM40567-4.2, 120 mA(R10=200K), 4.2V 컷오프, NCHG=LED 전용(GPIO 불가)
  - 펌웨어: CHG_FULL_DURATION = 2.5h, 5%→100% 선형(ADC 없음)
"""
import os
import sys
import csv
try:
    sys.stdout.reconfigure(encoding="utf-8")   # Windows cp949 콘솔에서도 한글/기호 출력
except Exception:
    pass
import numpy as np
import matplotlib
matplotlib.use("Agg")               # 디스플레이 없이 PNG 저장
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))

# ── 펌웨어 상수 (somfy_app.c 와 일치) ────────────────────────────────
FW_OLD_FULL_MIN = 150.0   # (구) 고정 CHG_FULL_DURATION = 150 min (2.5 h)
FW_START_PCT    = 5.0      # 충전 시작 초기값
FW_FULL_PCT     = 100.0
FW_CV_OVERHEAD  = 1.25     # CV 단계 보정 ×5/4 (somfy_app.c 와 동일)

def fw_full_min(q_mah, i_chg):
    """(신) 보드파생 완충시간[min] = (용량/충전전류) h × 1.25.
       somfy_app.c 의 CHG_FULL_DURATION_MS = BATT_MAH*3600000/CHG_MA*5/4 와 동일."""
    return (q_mah / i_chg) * 60.0 * FW_CV_OVERHEAD

def fw_estimate_pct(t_min, full_min):
    """펌웨어 _estimate_battery_percent() — 시간기반 선형 추정(완충시간 인자화)."""
    p = FW_START_PCT + (FW_FULL_PCT - FW_START_PCT) * (np.asarray(t_min) / full_min)
    return np.clip(p, FW_START_PCT, FW_FULL_PCT)

# ── 단셀 Li-ion OCV(SoC) 곡선 (대표적 LiCoO2 LiPo) ───────────────────
_SOC_PTS = np.array([0, 5, 10, 20, 40, 60, 75, 85, 95, 100]) / 100.0
_OCV_PTS = np.array([3.20, 3.50, 3.62, 3.72, 3.80, 3.90, 4.00, 4.08, 4.15, 4.20])

def ocv(soc):
    return float(np.interp(soc, _SOC_PTS, _OCV_PTS))

V_REG = 4.20            # 충전 조정전압 (4.2V 셀)

# ── 충전 회로 구성 (보드별) ──────────────────────────────────────────
CONFIGS = [
    dict(key="gnpe",  name="GNPE  600mAh @300mA (MCP73831)",
         q_mah=600, i_chg=300, iterm_frac=0.07, r_eff=0.40,
         stat="active-LOW (IO3, GPIO 読取 가능)"),
    dict(key="xiao600", name="XIAO  600mAh @120mA (SGM40567)",
         q_mah=600, i_chg=120, iterm_frac=0.10, r_eff=0.40,
         stat="NCHG = LED 전용 (GPIO 불가)"),
    dict(key="xiao250", name="XIAO  250mAh @120mA (SGM40567)",
         q_mah=250, i_chg=120, iterm_frac=0.10, r_eff=0.40,
         stat="NCHG = LED 전용 (GPIO 불가)"),
]

def simulate(cfg, dt_s=5.0, soc0=0.03, t_max_h=12.0):
    """CC/CV 선형 충전 시간영역 시뮬레이션. 각 스텝의 상태+인가전류 기록."""
    q     = cfg["q_mah"]
    i_chg = cfg["i_chg"]
    i_term = cfg["iterm_frac"] * i_chg
    r     = cfg["r_eff"]

    t, soc, I, V, ph, stat = [], [], [], [], [], []
    cur, tt = soc0, 0.0
    while tt <= t_max_h * 3600.0:
        o = ocv(cur)
        # IR 강하는 전류[A]×저항[Ω] = V. 전류는 mA 로 들고다니므로 /1000.
        v_cc = o + (i_chg / 1000.0) * r      # CC 가정 시 단자전압
        if v_cc < V_REG:                     # ── CC 단계
            i, v, phase = i_chg, v_cc, "CC"
        else:                                # ── CV 단계 (4.2V 고정)
            i = min(i_chg, max(0.0, (V_REG - o) / r * 1000.0))   # → mA
            v, phase = V_REG, "CV"
        charging = not (phase == "CV" and i <= i_term and cur > 0.5)

        t.append(tt / 60.0)                  # 분
        soc.append(cur * 100.0)              # %
        I.append(i); V.append(v); ph.append(phase)
        stat.append(0 if charging else 1)    # STAT: 0=LOW(충전중), 1=HIGH(완충)

        if not charging:                     # 완충 → 종료
            break
        cur = min(1.0, cur + i * (dt_s / 3600.0) / q)   # 쿨롱 적산
        tt += dt_s

    return dict(t=np.array(t), soc=np.array(soc), I=np.array(I),
                V=np.array(V), phase=ph, stat=np.array(stat),
                i_term=i_term, cfg=cfg)

# ── 시뮬레이션 실행 ──────────────────────────────────────────────────
results = [simulate(c) for c in CONFIGS]

# ── 콘솔 리포트 + 테스트(PASS/WARN/FAIL) ─────────────────────────────
def fmt_min(m):
    return f"{int(m//60)}h{int(m%60):02d}m"

print("=" * 74)
print(" 배터리 충전 회로 시뮬레이션 — CC/CV 선형 충전 (V_reg=4.20V)")
print("=" * 74)

all_pass = True
test_rows = []
for r in results:
    cfg = r["cfg"]
    t, soc, I, V, ph = r["t"], r["soc"], r["I"], r["V"], r["phase"]
    # 주요 지표
    cc_mask = np.array([p == "CC" for p in ph])
    _cv = np.where(~cc_mask)[0]
    cv_start_idx = int(_cv[0]) if _cv.size else len(t) - 1
    t_cc_end = t[cv_start_idx]
    soc_cc_end = soc[cv_start_idx]
    t_term = t[-1]
    soc_full = soc[-1]
    # 펌웨어 추정: (구) 고정 2.5h  vs  (신) 보드파생 완충시간
    full_new = fw_full_min(cfg["q_mah"], cfg["i_chg"])
    err_old_max = float(np.abs(fw_estimate_pct(t, FW_OLD_FULL_MIN) - soc).max())
    err_new_max = float(np.abs(fw_estimate_pct(t, full_new) - soc).max())
    soc_at_old = float(np.interp(FW_OLD_FULL_MIN, t, soc))  # 구: 150min 시점 실제 SoC
    soc_at_new = float(np.interp(full_new, t, soc))         # 신: 파생 완충시점 실제 SoC

    print(f"\n● {cfg['name']}")
    print(f"   CC→CV 전환 : {fmt_min(t_cc_end)}  (SoC {soc_cc_end:4.1f}%)")
    print(f"   완충(종료) : {fmt_min(t_term)}  (SoC {soc_full:4.1f}%, I_term {r['i_term']:.0f}mA)")
    print(f"   STAT       : {cfg['stat']}")
    print(f"   펌웨어 추정 vs 실제 SoC:")
    print(f"     · (구) 고정 2.5h  : 150min 에 100% 표시 → 실제 {soc_at_old:5.1f}%  (최대오차 {err_old_max:4.1f}%p)")
    print(f"     · (신) 파생 {full_new:3.0f}min : 완충시점 실제 {soc_at_new:5.1f}%  (최대오차 {err_new_max:4.1f}%p)")

    # ── 회로 정합성 테스트 ──
    def chk(name, ok):
        global all_pass
        all_pass = all_pass and ok
        print(f"     [{'PASS' if ok else 'FAIL'}] {name}")
        return ok
    cc_i = I[cc_mask]
    chk("CC 단계 전류 = I_chg(±2%)",
        cc_i.size == 0 or np.all(np.abs(cc_i - cfg["i_chg"]) <= 0.02 * cfg["i_chg"]))
    chk("단자전압 ≤ 4.2V(+20mV)", np.all(V <= V_REG + 0.02))
    chk("SoC 단조 증가", np.all(np.diff(soc) >= -1e-9))
    chk("완충 SoC ≥ 98%", soc_full >= 98.0)
    chk("CV 종료전류 ≤ I_term", I[-1] <= r["i_term"] + 1e-6)

    # ── 펌웨어 휴리스틱 정확도 판정 (신 보드파생 기준) ──
    if soc_at_new >= 90:
        verdict = "PASS  (보드파생 추정이 실제와 정합)"
    elif soc_at_new >= 70:
        verdict = "WARN  (다소 과대추정 — 표시용이라 안전엔 무관)"
    else:
        verdict = "FAIL  (BOARD_BATT_MAH/CHG_MA 가 실제 셀과 불일치)"
        all_pass = False
    old_verd = "OK" if soc_at_old >= 90 else ("WARN" if soc_at_old >= 70 else "FAIL")
    print(f"   ▶ 판정: (구)고정 → {old_verd}  /  (신)보드파생 → {verdict}")
    test_rows.append((cfg["name"], soc_at_old, full_new, soc_at_new, verdict.split()[0]))

print("\n" + "=" * 74)
print(f" 종합: {'전체 PASS' if all_pass else '일부 FAIL/WARN — 아래 표 참고'}")
print("=" * 74)
print(" 구성                              구:150m실제   신:완충   신:완충시점실제   판정")
for name, socold, fnew, socnew, verd in test_rows:
    print(f" {name:32s} {socold:8.1f}%   {fnew:5.0f}m   {socnew:10.1f}%   {verd:>4s}")

# ── CSV(GNPE 기준 시계열) ────────────────────────────────────────────
csv_path = os.path.join(HERE, "battery_charge_gnpe.csv")
g = results[0]
with open(csv_path, "w", newline="", encoding="utf-8") as f:
    w = csv.writer(f)
    w.writerow(["t_min", "SoC_true_%", "I_mA", "Vterm_V", "phase",
                "STAT", "fw_old_%", "fw_new_%"])
    g_full_new = fw_full_min(g["cfg"]["q_mah"], g["cfg"]["i_chg"])
    fw_old = fw_estimate_pct(g["t"], FW_OLD_FULL_MIN)
    fw_new = fw_estimate_pct(g["t"], g_full_new)
    for i in range(len(g["t"])):
        w.writerow([f"{g['t'][i]:.3f}", f"{g['soc'][i]:.2f}", f"{g['I'][i]:.1f}",
                    f"{g['V'][i]:.4f}", g["phase"][i], g["stat"][i],
                    f"{fw_old[i]:.1f}", f"{fw_new[i]:.1f}"])

# ── 그래프(PNG) ──────────────────────────────────────────────────────
fig, ax = plt.subplots(2, 2, figsize=(12, 8))
fig.suptitle("Somfy blind controller — Battery charge circuit simulation (CC/CV, 4.2V)",
             fontsize=13, fontweight="bold")
colors = ["#1f77b4", "#d62728", "#2ca02c"]

for r, c in zip(results, colors):
    lbl = r["cfg"]["name"].replace("  ", " ")
    ax[0, 0].plot(r["t"], r["I"], c, label=lbl)
    ax[0, 1].plot(r["t"], r["V"], c, label=lbl)
    ax[1, 0].plot(r["t"], r["soc"], c, label=lbl)

ax[0, 0].set(title="Charge current (CC then CV taper)", xlabel="time [min]", ylabel="I [mA]")
ax[0, 0].grid(alpha=0.3); ax[0, 0].legend(fontsize=8)
ax[0, 1].set(title="Terminal voltage", xlabel="time [min]", ylabel="V [V]")
ax[0, 1].axhline(4.2, ls="--", c="gray", lw=0.8); ax[0, 1].grid(alpha=0.3); ax[0, 1].legend(fontsize=8)
ax[1, 0].set(title="True SoC (coulomb count)", xlabel="time [min]", ylabel="SoC [%]")
ax[1, 0].grid(alpha=0.3); ax[1, 0].legend(fontsize=8)

# 펌웨어 추정 vs 실제 SoC (핵심 테스트): OLD 고정 2.5h  vs  NEW 보드파생
tmax = max(r["t"][-1] for r in results)
tg = np.linspace(0, tmax, 400)
ax[1, 1].plot(tg, fw_estimate_pct(tg, FW_OLD_FULL_MIN), "k--", lw=2,
              label="OLD fixed 2.5h (all boards)")
for r, c in zip(results, colors):
    nm = r["cfg"]["name"].split("(")[0].strip()
    full_new = fw_full_min(r["cfg"]["q_mah"], r["cfg"]["i_chg"])
    tt = np.linspace(0, max(r["t"][-1], full_new), 200)
    ax[1, 1].plot(r["t"], r["soc"], c, lw=1.6, label="true: " + nm)
    ax[1, 1].plot(tt, fw_estimate_pct(tt, full_new), c, ls=":", lw=1.3)  # NEW 보드파생
ax[1, 1].set(title="TEST: OLD fixed (black --) vs NEW derived (dotted) vs true SoC",
             xlabel="time [min]", ylabel="%")
ax[1, 1].grid(alpha=0.3); ax[1, 1].legend(fontsize=7)

plt.tight_layout(rect=[0, 0, 1, 0.96])
png_path = os.path.join(HERE, "battery_charge_sim.png")
plt.savefig(png_path, dpi=110)
print(f"\n그래프 저장: {os.path.relpath(png_path, os.path.dirname(HERE))}")
print(f"CSV   저장: {os.path.relpath(csv_path, os.path.dirname(HERE))}")
