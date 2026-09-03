#!/usr/bin/env python3
"""Render the Polish simulation guide to PDF without external network access."""

from pathlib import Path
from xml.sax.saxutils import escape

from bs4 import BeautifulSoup, NavigableString, Tag
from reportlab.lib import colors
from reportlab.lib.enums import TA_CENTER
from reportlab.lib.pagesizes import A4
from reportlab.lib.styles import ParagraphStyle, getSampleStyleSheet
from reportlab.lib.units import mm
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.platypus import (
    BaseDocTemplate, Frame, ListFlowable, ListItem, PageBreak, PageTemplate,
    Paragraph, Preformatted, Spacer, Table, TableStyle,
)

HERE = Path(__file__).resolve().parent
SOURCE = HERE / "system_symulacji.html"
OUTPUT = HERE / "system_symulacji.pdf"

pdfmetrics.registerFont(TTFont("DejaVu", "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"))
pdfmetrics.registerFont(TTFont("DejaVu-Bold", "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"))
pdfmetrics.registerFont(TTFont("DejaVuMono", "/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf"))


def footer(canvas, doc):
    canvas.saveState()
    canvas.setFont("DejaVu", 8)
    canvas.setFillColor(colors.HexColor("#66757c"))
    canvas.drawString(18 * mm, 10 * mm, "System symulacji quadcoptera")
    canvas.drawRightString(A4[0] - 17 * mm, 10 * mm, f"Strona {doc.page}")
    canvas.restoreState()


doc = BaseDocTemplate(
    str(OUTPUT), pagesize=A4,
    leftMargin=17 * mm, rightMargin=17 * mm,
    topMargin=16 * mm, bottomMargin=17 * mm,
    title="System symulacji quadcoptera",
    author="Dokumentacja quadcopter_sim",
)
frame = Frame(doc.leftMargin, doc.bottomMargin, doc.width, doc.height, id="main")
doc.addPageTemplates([PageTemplate(id="guide", frames=frame, onPage=footer)])

base = getSampleStyleSheet()
styles = {
    "title": ParagraphStyle("TitlePL", fontName="DejaVu-Bold", fontSize=25, leading=31,
                            textColor=colors.HexColor("#123f66"), alignment=TA_CENTER, spaceAfter=10 * mm),
    "subtitle": ParagraphStyle("SubtitlePL", fontName="DejaVu", fontSize=14, leading=20,
                               textColor=colors.HexColor("#47636f"), alignment=TA_CENTER, spaceAfter=4 * mm),
    "h2": ParagraphStyle("H2PL", fontName="DejaVu-Bold", fontSize=16, leading=20,
                         textColor=colors.HexColor("#12658a"), spaceBefore=7 * mm, spaceAfter=3 * mm,
                         keepWithNext=True),
    "h3": ParagraphStyle("H3PL", fontName="DejaVu-Bold", fontSize=12, leading=15,
                         textColor=colors.HexColor("#174d69"), spaceBefore=4 * mm, spaceAfter=2 * mm,
                         keepWithNext=True),
    "body": ParagraphStyle("BodyPL", fontName="DejaVu", fontSize=9.4, leading=13.2,
                           textColor=colors.HexColor("#202631"), spaceAfter=2.2 * mm),
    "small": ParagraphStyle("SmallPL", fontName="DejaVu", fontSize=8, leading=10,
                            textColor=colors.HexColor("#596970")),
    "equation": ParagraphStyle("EquationPL", fontName="DejaVu", fontSize=11, leading=15,
                               alignment=TA_CENTER, spaceBefore=2 * mm, spaceAfter=3 * mm),
    "flow": ParagraphStyle("FlowPL", fontName="DejaVuMono", fontSize=8.6, leading=13,
                           alignment=TA_CENTER, backColor=colors.HexColor("#eef5f7"),
                           borderColor=colors.HexColor("#aac4cf"), borderWidth=0.6,
                           borderPadding=8, spaceBefore=3 * mm, spaceAfter=4 * mm),
    "box": ParagraphStyle("BoxPL", fontName="DejaVu", fontSize=9, leading=13,
                          backColor=colors.HexColor("#edf8fb"), borderColor=colors.HexColor("#19a8c5"),
                          borderWidth=0.8, borderPadding=8, spaceBefore=3 * mm, spaceAfter=4 * mm),
    "pre": ParagraphStyle("PrePL", fontName="DejaVuMono", fontSize=7.8, leading=10.5,
                          textColor=colors.HexColor("#eaf6fa"), backColor=colors.HexColor("#17232c"),
                          borderPadding=8, spaceBefore=2 * mm, spaceAfter=3 * mm),
}


def plain(tag):
    return " ".join(tag.get_text(" ", strip=True).split())


def paragraph_text(tag):
    text = plain(tag)
    return escape(text)


def make_table(tag):
    rows = []
    for tr in tag.find_all("tr", recursive=False):
        cells = tr.find_all(["th", "td"], recursive=False)
        rows.append([Paragraph(escape(plain(cell)), styles["small"]) for cell in cells])
    if not rows:
        return None
    widths = [doc.width / len(rows[0])] * len(rows[0])
    table = Table(rows, colWidths=widths, repeatRows=1, hAlign="LEFT")
    table.setStyle(TableStyle([
        ("FONTNAME", (0, 0), (-1, 0), "DejaVu-Bold"),
        ("BACKGROUND", (0, 0), (-1, 0), colors.HexColor("#17698d")),
        ("TEXTCOLOR", (0, 0), (-1, 0), colors.white),
        ("GRID", (0, 0), (-1, -1), 0.4, colors.HexColor("#9aacb5")),
        ("VALIGN", (0, 0), (-1, -1), "TOP"),
        ("LEFTPADDING", (0, 0), (-1, -1), 5),
        ("RIGHTPADDING", (0, 0), (-1, -1), 5),
        ("TOPPADDING", (0, 0), (-1, -1), 5),
        ("BOTTOMPADDING", (0, 0), (-1, -1), 5),
    ]))
    for row in range(2, len(rows), 2):
        table.setStyle(TableStyle([("BACKGROUND", (0, row), (-1, row), colors.HexColor("#eef5f7"))]))
    return table


soup = BeautifulSoup(SOURCE.read_text(encoding="utf-8"), "html.parser")
story = []
for element in soup.body.children:
    if isinstance(element, NavigableString) or not isinstance(element, Tag):
        continue
    classes = element.get("class", [])
    if "cover" in classes:
        story.extend([Spacer(1, 38 * mm), Paragraph("System symulacji quadcoptera", styles["title"]),
                      Paragraph("ROS 2 Humble · Gazebo Classic · RViz 2", styles["subtitle"]),
                      Paragraph("Architektura, sterowanie, model wirników i wizualizacja", styles["subtitle"]),
                      Spacer(1, 55 * mm),
                      Paragraph("Dokumentacja pakietu <b>quadcopter_sim</b><br/>28 sierpnia 2026", styles["subtitle"]),
                      PageBreak()])
    elif "toc" in classes:
        story.append(Paragraph("Spis treści", styles["h2"]))
        items = [ListItem(Paragraph(escape(plain(li)), styles["body"])) for li in element.find_all("li")]
        story.extend([ListFlowable(items, bulletType="1", leftIndent=8 * mm), PageBreak()])
    elif element.name == "h2":
        if "page-break" in classes:
            story.append(PageBreak())
        story.append(Paragraph(paragraph_text(element), styles["h2"]))
    elif element.name == "h3":
        story.append(Paragraph(paragraph_text(element), styles["h3"]))
    elif element.name == "p":
        story.append(Paragraph(paragraph_text(element), styles["small"] if "small" in classes else styles["body"]))
    elif element.name == "pre":
        story.append(Preformatted(element.get_text().strip(), styles["pre"]))
    elif element.name in ("ul", "ol"):
        items = [ListItem(Paragraph(escape(plain(li)), styles["body"])) for li in element.find_all("li", recursive=False)]
        story.append(ListFlowable(items, bulletType="1" if element.name == "ol" else "bullet", leftIndent=8 * mm))
        story.append(Spacer(1, 2 * mm))
    elif element.name == "table":
        table = make_table(element)
        if table:
            story.extend([table, Spacer(1, 3 * mm)])
    elif element.name == "div":
        text = escape(plain(element))
        style = styles["flow"] if "flow" in classes else styles["equation"] if "equation" in classes else styles["box"]
        story.append(Paragraph(text, style))

doc.build(story)
print(OUTPUT)
