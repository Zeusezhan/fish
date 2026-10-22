# -*- coding: utf-8 -*-
"""把控制系统设计内容填入官方报告模板。"""
import copy
import os

from docx import Document
from docx.enum.table import WD_ALIGN_VERTICAL
from docx.enum.text import WD_ALIGN_PARAGRAPH, WD_LINE_SPACING
from docx.oxml.ns import qn
from docx.shared import Mm, Pt
from docx.table import _Cell

SRC = r"D:\Stm32\工科大赛\fish\报告素材\template.docx"
DST = r"D:\Stm32\工科大赛\fish\反恐排爆救援机器人-设计报告-控制系统设计.docx"
FLOW = r"D:\Stm32\工科大赛\fish\报告素材\程序流程图.png"

FONT_CN = "宋体"
SIZE = Pt(10.5)          # 五号
LINE = Pt(20)            # 行距固定值 20 磅

HEAD_IDEA = "1、控制系统设计思路"
HEAD_DEV = "2、主要器件选择"

IDEA = [
    ("p", "（1）总体架构：按“感知—决策—执行”三层划分，主控为 STM32F407ZGT6，各层经串口解耦；决策层以任务状态机运行，读码后依次执行排爆、反恐、救援，以视觉反馈为退出条件。"),
    ("p", "（2）感知层：MaixCAM 低位前视负责巡线、A4 二维码读取并给出靶心偏差；K230 高位俯视负责爆炸物与靶标颜色、人质形状识别；JY901S 以 I2C、50Hz 输出航向角与角速度，作航向基准。"),
    ("p", "（3）执行层：4 台张大头 Emm_V5.0（地址 1~4）挂 USART1 驱动麦轮；塔吊 z 轴与 xy 轴两台步进挂 USART3 作绝对定位；塔吊旋转与夹爪各由 1 路 50Hz PWM 舵机驱动。"),
    ("p", "（4）塔吊解算：塔吊为“旋转—伸缩—升降—夹爪”四自由度极坐标机构，视觉给出目标相对底盘中心的方位角 θ 与距离 r，主控按 θ 换算旋转舵机脉宽、按 r 换算伸缩轴脉冲，再下降、夹取、抬升，形成抓放闭环。"),
    ("p", "（5）底盘闭环：麦轮逆运动学解算四轮转速并限幅；航向 PD 在 TIM3 的 20ms 中断中按 u=Kp·e−Kd·ωz 计算，以角速度抑制过冲，±2° 死区消抖，四轮以广播地址 0x00 同步触发。"),
    ("p", "（6）可靠性：主循环 30ms 限频下发且值不变不重发；中断只算不发，规避 HAL_Delay 与 SysTick 死锁；上电闭环关闭、电机只使能不动作，按键确认后启动。"),
    ("p", "（7）创新点：双视觉异构分工，前视巡线与俯视色彩识别互补；极坐标塔吊以两轴一步进覆盖全场取放点，结构轻、惯量小；惯性高频保持、视觉低频修正的融合航向。"),
]

DEV = [
    "主控 STM32F407ZGT6：Cortex-M4F 168MHz、1MB Flash/192KB RAM、硬件浮点，3 路 USART 带 DMA 与多路 PWM，满足双视觉、多机总线与舵机并发。",
    "底盘驱动 张大头 Emm_V5.0 闭环步进 ×4（地址 1~4）：42 步进、1.8°/步、16 细分、编码器闭环、串口单总线编址，麦轮底盘不失步。",
    "塔吊 z/xy 轴 张大头 Emm_V5.0 ×2（地址 1、2）：位置模式，按实测标定脉冲当量换算，z 行程 150mm、xy 行程 80mm，断电自保持。",
    "旋转/夹爪 舵机 2×MG996R（金属齿、15kg·cm、50Hz PWM、5~7.4V）：一路经减速带动横臂旋转，一路驱动双指爪夹取爆炸物。",
    "姿态 维特智能 JY901S：九轴融合、航向精度 0.1°、回传 50Hz、I2C 地址 0x50，作航向闭环反馈源。",
    "视觉 K230（CanMV-K230，6TOPS KPU）高位俯视，负责爆炸物与靶标颜色、人质形状识别；MaixCAM（SG2002，1TOPS NPU）低位前视，负责巡线、二维码读取与靶心定位。",
    "打靶 650nm 5mW 可调焦红光激光模组：Φ12 金属封装、3~5V 工作，配恒流驱动板与 AO3400 低边开关，固装于塔吊横臂末端随旋转对准；1m 处光斑 ≤Φ5mm。",
    "人机 0.96 英寸 OLED（SSD1306，软件 I2C）+ 3 个独立按键：显示参数与状态，支持一键启动。",
    "电源 4S 锂聚合物电池 14.8V：经 DC-DC 稳压 5V/3.3V 供主控与视觉，母线直供步进驱动器。",
]

NOTE = [
    ("p", "流程图说明：上电先完成时钟、外设与显示初始化，再对 JY901S 做自检并把回传率提到 50Hz，随后清零航向、进入按键待命，避免上电误动作。"),
    ("p", "一键启动后，MaixCAM 巡线并读出 A4 二维码，主控解析出“排爆物颜色—靶标颜色—人质形状”三位任务码，状态机据此依次执行排爆、反恐、救援三个子任务；每完成一项即回到巡线主干，直到三项全部完成，最后返回出发区停车。"),
    ("p", "排爆与救援由塔吊完成：K230 俯视给出目标的方位角与距离，横臂旋转到位、伸缩轴伸出、z 轴下降，夹爪闭合后抬升收回，再送至排爆桶或返回区释放；夹取前后各做一次颜色/形状复核，不符则重新对位。反恐时 MaixCAM 给出靶心像素偏差，底盘转向与横臂旋转分两级对准，偏差进入阈值后点亮激光并保持短时，触发即熄灭。"),
]


def clear_cell(cell):
    tc = cell._tc
    for p in tc.findall(qn("w:p")):
        tc.remove(p)


def new_par(cell, text="", bold=False, align=WD_ALIGN_PARAGRAPH.JUSTIFY,
            indent=False, size=SIZE, line=LINE):
    p = cell.add_paragraph()
    pf = p.paragraph_format
    pf.alignment = align
    pf.line_spacing_rule = WD_LINE_SPACING.EXACTLY
    pf.line_spacing = line
    pf.space_before = Pt(0)
    pf.space_after = Pt(0)
    if indent:
        pf.first_line_indent = Pt(size.pt * 2)
    # 关键：模板启用了文档网格(linePitch=15.6磅)，必须关闭“对齐到网格”，
    # 否则 LibreOffice/Word 会把 20 磅固定行距量化到网格，行高翻倍撑破版式。
    pPr = p._p.get_or_add_pPr()
    pPr.set(qn("w:snapToGrid"), "0")
    if text:
        run = p.add_run(text)
        run.bold = bold
        run.font.size = size
        run.font.name = FONT_CN
        rfonts = run._element.get_or_add_rPr().get_or_add_rFonts()
        rfonts.set(qn("w:eastAsia"), FONT_CN)
        rfonts.set(qn("w:ascii"), FONT_CN)
        rfonts.set(qn("w:hAnsi"), FONT_CN)
    return p


def find_cell(table, needle):
    for row in table.rows:
        for tc in row._tr.findall(qn("w:tc")):
            cell = _Cell(tc, table)
            if needle in cell.text:
                return cell
    raise KeyError(needle)


doc = Document(SRC)
table = doc.tables[1]

# ---------- 1、控制系统设计思路 ----------
c_idea = find_cell(table, HEAD_IDEA)
c_idea.vertical_alignment = WD_ALIGN_VERTICAL.TOP
clear_cell(c_idea)
new_par(c_idea, HEAD_IDEA, bold=True, align=WD_ALIGN_PARAGRAPH.LEFT)
for kind, text in IDEA:
    new_par(c_idea, text, indent=True)

# ---------- 2、主要器件选择 ----------
c_dev = find_cell(table, HEAD_DEV)
c_dev.vertical_alignment = WD_ALIGN_VERTICAL.TOP
clear_cell(c_dev)
new_par(c_dev, HEAD_DEV, bold=True, align=WD_ALIGN_PARAGRAPH.LEFT)
for line in DEV:
    new_par(c_dev, line, align=WD_ALIGN_PARAGRAPH.JUSTIFY)

# ---------- 程序流程图 ----------
c_flow = find_cell(table, "(流程图)")
c_flow.vertical_alignment = WD_ALIGN_VERTICAL.CENTER
clear_cell(c_flow)
p = new_par(c_flow, "", align=WD_ALIGN_PARAGRAPH.CENTER)
p.add_run().add_picture(FLOW, width=Mm(148))

# ---------- 说明 ----------
c_note = find_cell(table, "说  明")
c_note.vertical_alignment = WD_ALIGN_VERTICAL.TOP
clear_cell(c_note)
new_par(c_note, "说  明", bold=True, align=WD_ALIGN_PARAGRAPH.CENTER)
for kind, text in NOTE:
    new_par(c_note, text, indent=True)

doc.save(DST)
print("saved", DST)
