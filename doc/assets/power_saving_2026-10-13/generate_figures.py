"""Report-only battery arithmetic and figures; never connects to the device.

Uses ReportLab charts and PDFium to create SVG/PNG files in this directory.
Assumed per-wake charge is a sensitivity parameter, not a measurement.
"""
import csv
import io
import json
from pathlib import Path

import pypdfium2 as pdfium
from reportlab.graphics import renderPDF, renderSVG
from reportlab.graphics.charts.barcharts import HorizontalBarChart
from reportlab.graphics.shapes import Drawing, Line, Rect, String
from reportlab.lib.colors import HexColor
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont

OUT = Path(__file__).resolve().parent
FONT = "MalgunReport"
pdfmetrics.registerFont(TTFont(FONT, "C:/Windows/Fonts/malgun.ttf"))
NOMINAL = 700.0
USABLE = NOMINAL * 0.8
BASE = 4.56
Q_UC = 50.0
BLUE = HexColor("#2563A6")
ORANGE = HexColor("#CA7830")
GREEN = HexColor("#22805D")
INK = HexColor("#173146")
MUTED = HexColor("#5E7180")
GRID = HexColor("#D9E1E6")
WHITE = HexColor("#FFFFFF")


def metrics(current):
    return {
        "current_mA": current,
        "mAh_per_day": current * 24,
        "nominal_percent_per_day": current * 24 / NOMINAL * 100,
        "days_700mAh": NOMINAL / current / 24,
        "days_560mAh": USABLE / current / 24,
        "gain_days_560mAh": USABLE / current / 24 - USABLE / BASE / 24,
        "current_reduction_percent": (BASE - current) / BASE * 100,
    }


items = [
    ("LP 버튼 중복 대기 제거", 2 / 0.11 - 1 / 0.10),
    ("main: 100ms → 이벤트 + 1초 점검", 10 - 1),
    ("OLED OFF: 500ms → 이벤트", 2),
    ("hold 유휴: 500ms → 이벤트", 2),
    ("LP 안전망 연장 · 추가분 비교", 10 - 1),
]
item_rows = []
for name, rate in items:
    reduction = rate * Q_UC / 1000
    low = metrics(BASE - rate * 25 / 1000)
    high = metrics(BASE - rate * 100 / 1000)
    item_rows.append({
        "name": name, "nominal_wake_reduction_per_s": rate,
        "assumed_charge_uC": Q_UC, "conditional_reduction_mA": reduction,
        **metrics(BASE - reduction),
        "gain_days_at_25uC": low["gain_days_560mAh"],
        "gain_days_at_100uC": high["gain_days_560mAh"],
    })

case_rows = [
    {"name": "CC 보정 전 · 장기 기록", "kind": "record_model", **metrics(5.62)},
    {"name": "CC 보정 후 · 기준선", "kind": "record_model", **metrics(BASE)},
    {"name": "가정: 기상 10회/초 감소", "kind": "conditional", **metrics(BASE - .5)},
    {"name": "가정: 기상 20회/초 감소", "kind": "conditional", **metrics(BASE - 1)},
    {"name": "목표: 평균 2.50mA", "kind": "target", **metrics(2.5)},
    {"name": "15일 목표: 평균 1.50mA", "kind": "target", **metrics(1.5)},
]


def label(d, x, y, s, size=11, color=INK, anchor="start"):
    d.add(String(x, y, s, fontName=FONT, fontSize=size,
                 fillColor=color, textAnchor=anchor))


def bars(d, values, x, y, w, h, maximum, step, colors):
    chart = HorizontalBarChart()
    chart.x, chart.y, chart.width, chart.height = x, y, w, h
    chart.data = [list(reversed(values))]
    chart.categoryAxis.visible = False
    chart.valueAxis.valueMin = 0
    chart.valueAxis.valueMax = maximum
    chart.valueAxis.valueStep = step
    chart.valueAxis.visible = False
    chart.barSpacing = 0
    chart.groupSpacing = 13
    for tick in range(0, int(maximum) + 1, step):
        tx = x + tick / maximum * w
        d.add(Line(tx, y, tx, y + h, strokeColor=GRID, strokeWidth=.65))
        label(d, tx, y - 16, str(tick), 10, MUTED, "middle")
    for i, color in enumerate(reversed(colors)):
        chart.bars[(0, i)].fillColor = color
        chart.bars[(0, i)].strokeColor = color
    d.add(chart)
    row_h = h / len(values)
    return [y + h - (i + .5) * row_h for i in range(len(values))]


def save(d, stem):
    renderSVG.drawToFile(d, str(OUT / (stem + ".svg")))
    pdf = pdfium.PdfDocument(io.BytesIO(renderPDF.drawToString(d)))
    page = pdf[0]
    image = page.render(scale=2).to_pil()
    image.save(OUT / (stem + ".png"))
    page.close()
    pdf.close()


def overview():
    d = Drawing(1000, 570)
    d.add(Rect(0, 0, 1000, 570, fillColor=WHITE, strokeColor=None))
    label(d, 28, 536, "배터리 소모율과 예상 사용 시간", 22)
    label(d, 28, 509, "700mAh 셀 / 시간 계산은 사용 가능 용량 560mAh(80%) / 새 실측이 아닌 기록 모델·조건부 계산", 11, MUTED)
    label(d, 310, 465, "하루 배터리 소모 · 정격 700mAh 대비 %", 12)
    label(d, 684, 465, "사용 가능 560mAh로 버티는 시간 · 일", 12)
    cs = [BLUE, BLUE, ORANGE, ORANGE, GREEN, GREEN]
    ypos = bars(d, [r["nominal_percent_per_day"] for r in case_rows],
                310, 138, 285, 300, 24, 6, cs)
    bars(d, [r["days_560mAh"] for r in case_rows],
         684, 138, 258, 300, 18, 3, cs)
    d.add(Line(684 + 15 / 18 * 258, 138, 684 + 15 / 18 * 258, 438,
               strokeColor=GREEN, strokeWidth=1.2, strokeDashArray=[4, 3]))
    for r, y in zip(case_rows, ypos):
        label(d, 28, y - 4, r["name"], 12)
        label(d, 606, y - 4, f'{r["nominal_percent_per_day"]:.2f}%', 11)
        label(d, 953, y - 4, f'{r["days_560mAh"]:.2f}', 11)
    label(d, 901, 446, "15일", 10, GREEN, "middle")
    label(d, 28, 91, "파랑: 과거 기록의 전류 모델   주황: 기상 1회당 50µC를 가정한 계획용 사례   초록: 달성해야 할 목표", 11, MUTED)
    label(d, 28, 66, "하루 소모 5.33% 이하면 80% 용량으로 15일. 현재 기준선은 15.63%/일 → 약 66% 감축 필요.", 12)
    label(d, 28, 40, "조건부 사례는 수정 효과를 실측한 값이 아니다. 같은 조건의 기상·전류 측정으로 바꿔야 한다.", 11, MUTED)
    save(d, "battery_life_comparison")


def benefits():
    d = Drawing(1000, 515)
    d.add(Rect(0, 0, 1000, 515, fillColor=WHITE, strokeColor=None))
    label(d, 28, 481, "수정 항목별 효과의 크기 · 조건부 추가 사용 시간", 22)
    label(d, 28, 453, "기준 4.56mA·560mAh / 기상 1회당 추가 전하 50µC / 이벤트 전달 및 주기 변경이 정상 동작하는 경우", 11, MUTED)
    vals = [r["gain_days_560mAh"] * 24 for r in item_rows]
    ypos = bars(d, vals, 370, 147, 490, 250, 18, 3, [ORANGE] * len(vals))
    for r, y, v in zip(item_rows, ypos, vals):
        label(d, 28, y - 4, r["name"], 12)
        label(d, 882, y - 4, f'+{v:.1f}시간', 12)
    label(d, 600, 107, "기준선보다 늘어나는 시간 · 시간", 12, INK, "middle")
    label(d, 28, 76, "이 그래프는 효과 실측이 아니다. 기상당 전하를 25~100µC로 바꾸면 증가 시간도 달라진다.", 11, MUTED)
    label(d, 28, 50, "개별 결과를 더하지 않는다. 동시 적용 시 기상 겹침과 전원 상태 변화 때문에 효과가 중복된다.", 11, MUTED)
    label(d, 28, 24, "LP 안전망 연장은 중복 대기 정리 후의 추가분만 같은 기준선에서 비교. MODEM·TOP은 미측정.", 11, MUTED)
    save(d, "conditional_item_benefits")


if __name__ == "__main__":
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / "calculations.json").write_text(json.dumps({
        "nominal_mAh": NOMINAL, "usable_mAh": USABLE, "baseline_mA": BASE,
        "assumed_charge_per_wake_uC": Q_UC,
        "items": item_rows, "cases": case_rows,
    }, ensure_ascii=False, indent=2), encoding="utf-8")
    for name, rows in [("item_benefits.csv", item_rows), ("life_cases.csv", case_rows)]:
        with (OUT / name).open("w", encoding="utf-8-sig", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=rows[0].keys())
            writer.writeheader()
            writer.writerows(rows)
    overview()
    benefits()
    print(json.dumps(item_rows, ensure_ascii=False, indent=2))
