# -*- coding: utf-8 -*-
"""逐段估算行数。"""
import unicodedata

CELL_MM = 182.6 - 2 * 1.9
CHAR_MM = 10.5 * 25.4 / 72


def count_lines(text, indent_chars=0):
    avail = CELL_MM - indent_chars * CHAR_MM
    lines, cur = 1, 0.0
    for ch in text:
        w = CHAR_MM if unicodedata.east_asian_width(ch) in ("W", "F") else CHAR_MM * 0.52
        if cur + w > avail:
            lines += 1
            cur = w
        else:
            cur += w
    return lines


from build_report import IDEA, DEV, NOTE

print("---- 思路 ----")
tot = 1
for kind, t in IDEA:
    n = count_lines(t, 2)
    tot += n
    print(f"  {n}行  ({len(t)}字)  {t[:28]}...")
print(f"小计（含标题）{tot} 行 / 可用 15.3 行")

print("---- 器件 ----")
tot2 = 1
for t in DEV:
    n = count_lines(t, 0)
    tot2 += n
    print(f"  {n}行  ({len(t)}字)  {t[:28]}...")
print(f"小计（含标题）{tot2} 行 / 可用 17.6 行")
print(f"两格合计 {tot + tot2} 行 / 页面可用 34.2 行")
