# -*- coding: utf-8 -*-
"""生成控制系统程序流程图（PNG），供设计报告插入。"""
import os
from PIL import Image, ImageDraw, ImageFont

FONT = r"C:\Windows\Fonts\simhei.ttf"
OUT = r"D:\Stm32\工科大赛\fish\报告素材\程序流程图.png"

W = 1320
MARGIN_X = 70
BOX_W = 1180
PAD_Y = 22
GAP = 46
LINE_H = 42
TXT_SIZE = 31

f_txt = ImageFont.truetype(FONT, TXT_SIZE)
f_small = ImageFont.truetype(FONT, 26)

# (type, lines)  type: start / proc / dec / end
NODES = [
    ("start", ["开始"]),
    ("proc",  ["系统初始化：时钟、GPIO、DMA、I2C、USART、TIM、OLED"]),
    ("proc",  ["JY901S 姿态自检，回传率设为 50Hz，航向清零"]),
    ("proc",  ["按键待命，一键启动"]),
    ("proc",  ["MaixCAM 巡线行驶，读取 A4 二维码"]),
    ("proc",  ["解析三位任务码：排爆物颜色／靶标颜色／人质形状"]),
    ("proc",  ["排爆：K230 识别颜色→塔吊旋转伸缩对位→下降夹取→投入排爆桶"]),
    ("proc",  ["反恐：环心定位→底盘与横臂两级对准→650nm 激光触发"]),
    ("proc",  ["救援：K230 识别形状→塔吊抓取→运回返回区释放"]),
    ("dec",   ["三项任务全部完成？"]),
    ("proc",  ["返回出发区，停车"]),
    ("end",   ["结束"]),
]

# 预计算高度
heights = []
for t, lines in NODES:
    if t == "dec":
        heights.append(2 * (len(lines) * LINE_H) + 90)
    else:
        heights.append(len(lines) * LINE_H + 2 * PAD_Y)

H = 2 * 40 + sum(heights) + GAP * (len(NODES) - 1) + 20
img = Image.new("RGB", (W, H), "white")
d = ImageDraw.Draw(img)


def text_w(s, font):
    b = d.textbbox((0, 0), s, font=font)
    return b[2] - b[0]


def center_text(x_center, y_top, lines, font):
    for i, s in enumerate(lines):
        w = text_w(s, font)
        d.text((x_center - w / 2, y_top + i * LINE_H), s, font=font, fill="black")


x0 = MARGIN_X
x1 = MARGIN_X + BOX_W
xc = (x0 + x1) / 2
y = 40
top_centers = []

for idx, (t, lines) in enumerate(NODES):
    h = heights[idx]
    if t == "start":
        # 圆角扁框
        d.rounded_rectangle([x0 + 400, y, x1 - 400, y + h], radius=h // 2,
                            outline="black", width=4)
        center_text(xc, y + (h - len(lines) * LINE_H) / 2 + 2, lines, f_txt)
    elif t == "end":
        d.rounded_rectangle([x0 + 400, y, x1 - 400, y + h], radius=h // 2,
                            outline="black", width=4)
        center_text(xc, y + (h - len(lines) * LINE_H) / 2 + 2, lines, f_txt)
    elif t == "dec":
        d.polygon([(xc, y), (x1, y + h / 2), (xc, y + h), (x0, y + h / 2)],
                  outline="black", width=4, fill="white")
        center_text(xc, y + (h - len(lines) * LINE_H) / 2 + 2, lines, f_txt)
    else:
        d.rounded_rectangle([x0, y, x1, y + h], radius=10, outline="black", width=4)
        center_text(xc, y + (h - len(lines) * LINE_H) / 2 + 2, lines, f_txt)

    top_centers.append((y, y + h))
    if idx < len(NODES) - 1:
        # 主干箭头
        d.line([(xc, y + h), (xc, y + h + GAP)], fill="black", width=4)
        ay = y + h + GAP
        d.polygon([(xc - 11, ay - 16), (xc + 11, ay - 16), (xc, ay)], fill="black")
    y += h + GAP

# 回环：判定框左侧 -> 节点 5（MaixCAM 巡线行驶）
dec_idx = 9
loop_idx = 4
dy_top, dy_bot = top_centers[dec_idx]
ly_top, ly_bot = top_centers[loop_idx]
lx = x0 - 48                      # 回环竖线尽量靠左，给“否”标留出空间
mid_dec = (dy_top + dy_bot) / 2
d.line([(x0, mid_dec), (lx, mid_dec)], fill="black", width=4)
d.line([(lx, mid_dec), (lx, (ly_top + ly_bot) / 2)], fill="black", width=4)
d.line([(lx, (ly_top + ly_bot) / 2), (x0 - 2, (ly_top + ly_bot) / 2)], fill="black", width=4)
d.polygon([(x0 - 2, (ly_top + ly_bot) / 2 - 11), (x0 - 2, (ly_top + ly_bot) / 2 + 11),
           (x0 + 14, (ly_top + ly_bot) / 2)], fill="black")
# “否”标在回环竖线右侧、横线上方，既不压线也不出画布
d.text((lx + 6, mid_dec - 34), "否", font=f_small, fill="black")
d.text((xc + 14, (dy_bot + GAP * 0.25)), "是", font=f_small, fill="black")

img.save(OUT, dpi=(300, 300))
print("saved", OUT, img.size)
