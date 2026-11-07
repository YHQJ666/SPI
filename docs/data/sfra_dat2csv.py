#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""sfra_dat2csv.py —— CCS "Save Memory" 的 5 个 .dat → 填模板 → 导出 ZOH 校正后的 CSV

流程
----
1. 读 `&FreqVect.dat`、`&OL_MagVect.dat`、`&OL_PhaseVect.dat`、`&Plant_MagVect.dat`、
   `&Plant_PhaseVect.dat`（带不带 `&`、文件名里有空格都行；同名多个取最新的）
2. 填进 `SFRA结果换算模板.xlsx` 的 A~E 列（第 2 行起，模板里的公式不动）；
   顺带把 F~M 的缓存值按新数据更新，两个图表的引用区间也改成实际点数
3. 用模板同一套公式在 Python 里算一遍，写 `频率 + 4 列 ZOH 校正值` 的 CSV
   （行尾强制 LF）
4. `--design` 时直接把新 CSV 喂给 `sfra_pi_design.py`（不用再改它的 --csv 默认值）

公式（与模板 J~M 列一致）
------------------------
    raw/65536            → dB / 度
    mag_corr = mag - 20*log10(|sin(pi*f/fs)/(pi*f/fs)|)
    ph_corr  = ph  + 180*f/fs
    fs = 20000 Hz（`--fs` 可改）

用法
----
    python sfra_dat2csv.py                          # 自动找 .dat，输出 SFRA_<时间>.csv/.xlsx
    python sfra_dat2csv.py -o SFRA_boost_open.csv   # 指定输出名（xlsx 同名）
    python sfra_dat2csv.py --design --mode PI       # 转完直接设计 KDC/fz0
    python sfra_dat2csv.py -d D:\\sweep --template 模板.xlsx --no-xlsx

只依赖标准库。
"""

from __future__ import annotations

import argparse
import datetime as _dt
import math
import os
import re
import shlex
import subprocess
import sys
import zipfile

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
for _s in (sys.stdout, sys.stderr):
    try:
        _s.reconfigure(encoding="utf-8")
    except Exception:
        pass

# 五个信号：键 → 文件名里的特征串
SIGNALS = [
    ("freq", "FreqVect"),
    ("ol_mag", "OL_MagVect"),
    ("ol_phase", "OL_PhaseVect"),
    ("plant_mag", "Plant_MagVect"),
    ("plant_phase", "Plant_PhaseVect"),
]


# ----------------------------------------------------------------------------- 找文件

def norm_name(name):
    """`&OL_MagVect .dat` → `olmagvect`。"""
    base = os.path.splitext(os.path.basename(name))[0]
    return base.replace("&", "").replace(" ", "").replace("_", "").lower()


def find_dat_files(dirpath):
    """在目录里按特征串找 5 个 .dat，同名多个取 mtime 最新的。"""
    found = {}
    for name in os.listdir(dirpath):
        if not name.lower().endswith(".dat"):
            continue
        key_base = norm_name(name)
        for key, token in SIGNALS:
            if token.replace("_", "").lower() in key_base:
                path = os.path.join(dirpath, name)
                if key not in found or os.path.getmtime(path) > os.path.getmtime(found[key]):
                    found[key] = path
    return found


def read_ti_dat(path):
    """读 CCS 的 TI Data 格式：第一行 `1651 <fmt> <addr> <page> <len> ...`，往后是数值。"""
    vals = []
    with open(path, "r", encoding="utf-8", errors="replace") as fh:
        header = fh.readline().strip()
        for line in fh:
            tok = line.strip()
            if not tok:
                continue
            try:
                vals.append(float(tok))
            except ValueError:
                try:
                    vals.append(float(int(tok, 16)))  # 有些导出版本是十六进制
                except ValueError:
                    continue
    return header, vals


# ----------------------------------------------------------------------------- 换算

def sinc_db(f, fs):
    """20*log10|sin(pi f/fs)/(pi f/fs)|，f=0 时取 0 dB。"""
    if f == 0.0:
        return 0.0
    x = math.pi * f / fs
    return 20.0 * math.log10(abs(math.sin(x) / x))


def corrected(freq, ol_mag_raw, ol_ph_raw, pl_mag_raw, pl_ph_raw, fs):
    """按模板 J~M 列的公式，返回 4 个校正后的列表。"""
    j, k, l, m = [], [], [], []
    for i, f in enumerate(freq):
        j.append(ol_mag_raw[i] / 65536.0 - sinc_db(f, fs))
        k.append(ol_ph_raw[i] / 65536.0 + 180.0 * f / fs)
        l.append(pl_mag_raw[i] / 65536.0 - sinc_db(f, fs))
        m.append(pl_ph_raw[i] / 65536.0 + 180.0 * f / fs)
    return j, k, l, m


# ----------------------------------------------------------------------------- xlsx 补丁

def _num(v):
    """数字 → 单元格文本。"""
    if isinstance(v, float):
        if v == int(v) and abs(v) < 1e6:
            return "%d" % int(v)
        return repr(v)
    return "%d" % int(v)


def set_cell(xml, coord, value):
    """把 <c r="A2"...> 的值改成 value（None = 清空），保留样式属性。"""
    pat = re.compile(r'<c r="%s"([^>]*?)(?:/>|>(.*?)</c>)' % re.escape(coord), re.S)
    m = pat.search(xml)
    if not m:
        return xml
    attrs = re.sub(r'\s+t="[^"]*"', "", m.group(1) or "")
    if value is None:
        new = '<c r="%s"%s/>' % (coord, attrs)
    else:
        new = '<c r="%s"%s><v>%s</v></c>' % (coord, attrs, _num(value))
    return xml[:m.start()] + new + xml[m.end():]


def set_cached(xml, coord, value):
    """只更新公式单元格的缓存 <v>，公式（含 shared 公式）原样保留。"""
    pat = re.compile(r'<c r="%s"([^>]*?)>(.*?)</c>' % re.escape(coord), re.S)
    m = pat.search(xml)
    if not m:
        return xml
    attrs = re.sub(r'\s+t="[^"]*"', "", m.group(1) or "")
    fm = re.search(r"<f\b[^>]*(?:/>|>.*?</f>)", m.group(2), re.S)
    fxml = fm.group(0) if fm else ""
    new = '<c r="%s"%s>%s<v>%s</v></c>' % (coord, attrs, fxml, _num(value))
    return xml[:m.start()] + new + xml[m.end():]


def clear_cached(xml, coord):
    """保留公式、去掉缓存值（<v>）。"""
    pat = re.compile(r'<c r="%s"[^>]*>(.*?)</c>' % re.escape(coord), re.S)
    m = pat.search(xml)
    if not m:
        return xml
    inner = re.sub(r"<v>.*?</v>", "", m.group(1), flags=re.S)
    return xml[:m.start()] + m.group(0)[: m.group(0).index(">") + 1] + inner + "</c>" + xml[m.end():]


def patch_sheet(xml, freq, ol_mag, ol_ph, pl_mag, pl_ph, corr, last_row, first_row=2):
    """写 A~E 原始值、F~M 缓存值；多余行清空。"""
    j, k, l, m = corr
    for i in range(len(freq)):
        r = first_row + i
        xml = set_cell(xml, "A%d" % r, freq[i])
        xml = set_cell(xml, "B%d" % r, ol_mag[i])
        xml = set_cell(xml, "C%d" % r, ol_ph[i])
        xml = set_cell(xml, "D%d" % r, pl_mag[i])
        xml = set_cell(xml, "E%d" % r, pl_ph[i])
        xml = set_cached(xml, "F%d" % r, ol_mag[i] / 65536.0)
        xml = set_cached(xml, "G%d" % r, ol_ph[i] / 65536.0)
        xml = set_cached(xml, "H%d" % r, pl_mag[i] / 65536.0)
        xml = set_cached(xml, "I%d" % r, pl_ph[i] / 65536.0)
        xml = set_cached(xml, "J%d" % r, j[i])
        xml = set_cached(xml, "K%d" % r, k[i])
        xml = set_cached(xml, "L%d" % r, l[i])
        xml = set_cached(xml, "M%d" % r, m[i])
    for r in range(first_row + len(freq), last_row + 1):
        for col in "ABCDE":
            xml = set_cell(xml, "%s%d" % (col, r), None)
        for col in "FGHIJKLM":
            xml = clear_cached(xml, "%s%d" % (col, r))
    return xml


def patch_charts(xml, last_row, cols=("A", "J", "K", "L", "M")):
    """把图表的引用区间改成 2..last_row，并去掉旧缓存。"""
    for col in cols:
        xml = re.sub(r"\$%s\$2:\$%s\$\d+" % (col, col),
                     "$%s$2:$%s$%d" % (col, col, last_row), xml)
    xml = re.sub(r"<c:numCache>.*?</c:numCache>", "", xml, flags=re.S)
    return xml


def build_xlsx(template, out, freq, ol_mag, ol_ph, pl_mag, pl_ph, corr):
    """复制模板 → 补丁 sheet + charts + calcPr → 新文件。"""
    with zipfile.ZipFile(template) as z:
        names = z.namelist()
        blobs = {n: z.read(n) for n in names}

    data_sheet = None
    for n in names:
        if re.match(r"xl/worksheets/sheet\d+\.xml$", n) and b'<f t="shared"' in blobs[n]:
            data_sheet = n
            break
    if data_sheet is None:
        raise RuntimeError("模板里没找到带公式的数据表")

    xml = blobs[data_sheet].decode("utf-8")
    rows = [int(v) for v in re.findall(r'<row r="(\d+)"', xml)]
    last_row = max(rows)
    if last_row - 1 < len(freq):
        raise RuntimeError(
            "模板数据区只有 %d 行（2~%d），这次有 %d 点；请先在模板里多留几行。"
            % (last_row - 1, last_row, len(freq)))
    xml = patch_sheet(xml, freq, ol_mag, ol_ph, pl_mag, pl_ph, corr, last_row)
    blobs[data_sheet] = xml.encode("utf-8")

    for n in names:
        if re.match(r"xl/charts/chart\d+\.xml$", n):
            blobs[n] = patch_charts(blobs[n].decode("utf-8"), 1 + len(freq)).encode("utf-8")

    wb = blobs["xl/workbook.xml"].decode("utf-8")
    if "fullCalcOnLoad" not in wb:
        wb = re.sub(r"<calcPr([^>]*?)/>", r'<calcPr\1 fullCalcOnLoad="1"/>', wb, count=1)
    blobs["xl/workbook.xml"] = wb.encode("utf-8")

    tmp = out + ".tmp"
    with zipfile.ZipFile(tmp, "w", zipfile.ZIP_DEFLATED) as z:
        for n in names:
            z.writestr(n, blobs[n])
    os.replace(tmp, out)
    return last_row


# ----------------------------------------------------------------------------- CSV

def write_csv(path, freq, corr):
    j, k, l, m = corr
    lines = ["Frequency (Hz),OL Magnitude (dB),OL Phase (Deg),Plant Magnitude (dB),Plant Phase (Deg)"]
    for i in range(len(freq)):
        lines.append("%.6f,%.6f,%.6f,%.6f,%.6f" % (freq[i], j[i], k[i], l[i], m[i]))
    text = "\n".join(lines) + "\n"
    with open(path, "w", encoding="utf-8", newline="") as fh:
        fh.write(text)
    with open(path, "rb") as fh:
        raw = fh.read()
    if b"\r" in raw:
        raise RuntimeError("CSV 里出现了 CR，行尾不是纯 LF")
    return len(lines) - 1


# ----------------------------------------------------------------------------- 主程序

def main(argv=None):
    ap = argparse.ArgumentParser(description="CCS .dat → 模板 xlsx → ZOH 校正 CSV")
    ap.add_argument("-d", "--dir", default=SCRIPT_DIR, help=".dat 所在目录（默认脚本目录）")
    ap.add_argument("--template", default=os.path.join(SCRIPT_DIR, "SFRA结果换算模板.xlsx"))
    ap.add_argument("-o", "--out", default=None,
                    help="输出 CSV 名（xlsx 同名）；默认 SFRA_<时间>.csv")
    ap.add_argument("--fs", type=float, default=20000.0, help="开关/ISR 频率，默认 20 kHz")
    ap.add_argument("--no-xlsx", action="store_true", help="只写 CSV，不动模板")
    ap.add_argument("--design", action="store_true", help="转完调用 sfra_pi_design.py")
    ap.add_argument("--mode", default=None, help="透传给设计脚本：PI 或 P")
    ap.add_argument("--design-args", default="", help="额外透传给设计脚本的参数")
    args = ap.parse_args(argv)

    files = find_dat_files(args.dir)
    missing = [k for k, _ in SIGNALS if k not in files]
    if missing:
        ap.error("没找到这些 .dat: %s（目录 %s）" % (", ".join(missing), args.dir))

    print("输入:")
    data = {}
    for key, token in SIGNALS:
        hdr, vals = read_ti_dat(files[key])
        data[key] = vals
        print("  %-10s %-42s %3d 点  hdr=[%s]" % (key, os.path.basename(files[key]), len(vals), hdr))

    n = len(data["freq"])
    for key, _ in SIGNALS:
        if len(data[key]) != n:
            ap.error("点数不一致：%s 有 %d 点，FreqVect 有 %d 点" % (key, len(data[key]), n))
    freq = data["freq"]
    print("频率 %.1f ~ %.1f Hz，共 %d 点" % (freq[0], freq[-1], n))

    corr = corrected(freq, data["ol_mag"], data["ol_phase"],
                     data["plant_mag"], data["plant_phase"], args.fs)
    print("Plant 校正后：%.1f dB / %.1f° @ %.0f Hz，%.1f dB / %.1f° @ %.0f Hz"
          % (corr[2][0], corr[3][0], freq[0], corr[2][-1], corr[3][-1], freq[-1]))

    ol_max = max(corr[0])
    if ol_max <= 0.0:
        print("[!] 警告: OL 幅值全频段最高只有 %.1f dB（< 0 dB），没有 0 dB 穿越点；" % ol_max)
        print("    设计脚本会报无解，先确认这次的环路增益 / LOOP_MODE / 采集通道。")

    out = args.out or os.path.join(args.dir, "SFRA_" + _dt.datetime.now().strftime("%Y%m%d-%H%M") + ".csv")
    out = os.path.abspath(out)
    cnt = write_csv(out, freq, corr)
    print("写出 CSV : %s（%d 行，LF 行尾）" % (out, cnt))

    if not args.no_xlsx:
        xlsx = os.path.splitext(out)[0] + ".xlsx"
        last = build_xlsx(args.template, xlsx, freq, data["ol_mag"], data["ol_phase"],
                          data["plant_mag"], data["plant_phase"], corr)
        print("写出 XLSX: %s（A~E 已填，F~M 公式保留，图表区间 2~%d）" % (xlsx, 1 + n))

    if args.design:
        designer = os.path.join(SCRIPT_DIR, "sfra_pi_design.py")
        cmd = [sys.executable, designer, "--csv", out]
        if args.mode:
            cmd += ["--mode", args.mode]
        cmd += shlex.split(args.design_args)
        print("\n调用: " + " ".join('"%s"' % c if " " in c else c for c in cmd))
        sys.stdout.flush()
        rc = subprocess.run(cmd).returncode
        if rc != 0:
            print("[warn] sfra_pi_design.py 退出码 %d" % rc)
    return out


if __name__ == "__main__":
    main()
