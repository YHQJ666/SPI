#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""sfra_pi_design.py -- 由 SFRA 扫频 CSV 自动设计 TI 补偿器 "Series PI (Pole Zero)" 的 KDC 与 fz0

背景 / 约定
-----------
TI powerSUITE 的 Compensation Designer 里，PI 选 "Series PI (Pole Zero)" 时：

    Gc(s) = KDC * (1 + wz/s)          wz = 2*pi*fz0

    |Gc| = KDC * sqrt(1 + (fz0/f)^2)
    <Gc  = -atan(fz0/f) = -90deg + atan(f/fz0)

GUI 里 fz0 的单位是 kHz（"Enter Zero Location in kHz"），KDC 无量纲。
本工程 20 kHz 采样时它落盘成 DCL_PI_C1_C2_L1 的两个系数（双线性/Tustin）：

    C1 = KDC * (1 + pi*fz0/fs)        u(k) = u(k-1) + C1*e(k) + C2*e(k-1)
    C2 = KDC * (pi*fz0/fs - 1)

本脚本的穿越频率 / 相位裕度 / 幅值裕度 判据与 GUI 状态栏完全一致（逐采样点、取**第一次**
穿越），因此算出来的数直接填进 GUI 就能对上：

    fc  : 第一个满足 |L(f_i-1)| > 0 dB >= |L(f_i)| 的采样点 f_i
    PM  : 180 + <L(fc)
    GM  : 第一次 <L(f_i) <= -180deg 的采样点处, -|L| dB

计算模式（宏 COMP_MODE，文件顶部；命令行 --mode 可临时覆盖）
---------------------------------------------------------
    "PI" : Series PI (Pole Zero)  Gc(s) = KDC * (1 + wz/s)   设计 KDC 与 fz0
    "P"  : 单 P（纯比例）          Gc(s) = KDC                只设计 KDC，fz0 = 0

用法
----
    python sfra_pi_design.py                              # 按宏 COMP_MODE（默认 PI）搜索
    python sfra_pi_design.py --mode P                     # 切到单 P 计算
    python sfra_pi_design.py --verify 0.134 0.4           # 复现 GUI 截图里的旧设计（PI）
    python sfra_pi_design.py --verify 2.0                 # 单 P：只给 KDC
    python sfra_pi_design.py --frontier                   # 打印 fc / fz0 / PM / GM 权衡表
    python sfra_pi_design.py --csv SFRAData.csv --fc-max 2000 --gm-min 5 --pm-min 45 --pm-max 60

输出：控制台报告 + 图/逐点 CSV/报告三件套，按模式分名：pi_design_pi_* / pi_design_p_*

只依赖标准库；有 Pillow 时才画 PNG，没有也能跑（跳过图）。
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import sys

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

# Windows 控制台默认 GBK，中文容易乱码；终端支持 UTF-8 时强制切过去
for _s in (sys.stdout, sys.stderr):
    try:
        _s.reconfigure(encoding="utf-8")
    except Exception:
        pass

# =====================================================================
# 【宏定义】补偿器计算模式：改这一行切换，命令行 --mode 可临时覆盖
#   "PI" = Series PI (Pole Zero)：Gc(s) = KDC·(1 + wz/s)，设计 KDC 与 fz0
#   "P"  = 单 P（纯比例）：        Gc(s) = KDC，          只设计 KDC，fz0 记 0
# =====================================================================
COMP_MODE = "PI"          # <-- 要单 P 就改成 "P"


# ----------------------------------------------------------------------------- 数据

class PlantData:
    """一条扫频曲线：频率、幅值(dB)、相位(deg, 已 unwrap)。"""

    def __init__(self, freq, mag_db, phase_deg, source=""):
        self.f = [float(x) for x in freq]
        self.mag_db = [float(x) for x in mag_db]
        self.phase = unwrap_deg([float(x) for x in phase_deg])
        self.source = source

    def __len__(self):
        return len(self.f)

    def mag_lin(self, i):
        return 10.0 ** (self.mag_db[i] / 20.0)


def unwrap_deg(values):
    """把 ±180 卷绕的相位展开成连续曲线（按点序，允许频率非等间隔）。"""
    if not values:
        return []
    out = [float(values[0])]
    for v in values[1:]:
        v = float(v)
        prev = out[-1]
        while v - prev > 180.0:
            v -= 360.0
        while v - prev < -180.0:
            v += 360.0
        out.append(v)
    return out


def _col(header, *keys):
    """在表头里找同时包含全部 key 的列号。"""
    for i, h in enumerate(header):
        low = h.lower()
        if all(k.lower() in low for k in keys):
            return i
    return None


def load_sfra_csv(path):
    """读 TI SFRA / 补偿器导出的 CSV。"""
    with open(path, newline="", encoding="utf-8-sig") as fh:
        rows = [r for r in csv.reader(fh) if r and any(c.strip() for c in r)]
    if len(rows) < 3:
        raise ValueError("CSV 行数太少: %s" % path)

    header = [h.strip() for h in rows[0]]
    i_f = _col(header, "freq")
    i_pm = _col(header, "plant", "mag")
    i_pp = _col(header, "plant", "phase")
    if i_f is None or i_pm is None or i_pp is None:
        # 退化格式：3 列
        i_f, i_pm, i_pp = 0, 1, 2

    freq, mags, phases = [], [], []
    for r in rows[1:]:
        try:
            if len(r) <= max(i_f, i_pm, i_pp):
                continue
            freq.append(float(r[i_f]))
            mags.append(float(r[i_pm]))
            phases.append(float(r[i_pp]))
        except ValueError:
            continue
    if len(freq) < 5:
        raise ValueError("没读到足够的扫频点: %s" % path)
    return PlantData(freq, mags, phases, os.path.basename(path))


# ----------------------------------------------------------------------------- 格式化工具

def fmt_khz(hz):
    if hz is None:
        return "None"
    return "%.5f" % (hz / 1000.0)


def safe_fmt(val, fmt_spec="%.2f"):
    """安全格式化：当值为 None 时返回 'None'，避免程序崩溃"""
    return "None" if val is None else (fmt_spec % val)


# ----------------------------------------------------------------------------- 传函

def pi_comp(f, kdc, fz0_hz):
    """Series PI (Pole Zero)：返回 (|Gc| dB 列表, <Gc 度列表)。"""
    md, pd = [], []
    for fi in f:
        x = fz0_hz / fi if fi > 0 else 0.0
        md.append(20.0 * math.log10(kdc * math.hypot(1.0, x)))
        pd.append(-math.degrees(math.atan(x)))
    return md, pd


def loop(plant, kdc, fz0_hz):
    """开环 L = Plant * Gc：返回 (|L| dB 列表, <L 度列表)。"""
    md, pd = pi_comp(plant.f, kdc, fz0_hz)
    return ([plant.mag_db[i] + md[i] for i in range(len(plant))],
            [plant.phase[i] + pd[i] for i in range(len(plant))])


def crossings(f, ldb, lph):
    """找出所有 0 dB 穿越与 -180deg 穿越的索引（供报告用）。"""
    ones, phases = [], []
    for i in range(1, len(f)):
        if (ldb[i - 1] > 0.0) != (ldb[i] > 0.0):
            ones.append(i)
        if (lph[i - 1] > -180.0) != (lph[i] > -180.0):
            phases.append(i)
    return ones, phases


def gui_margins(f, ldb, lph):
    """与补偿器 GUI 状态栏一致的判据：取第一次穿越的采样点。"""
    fc_i = gm_i = None
    for i in range(1, len(f)):
        if ldb[i - 1] > 0.0 >= ldb[i]:
            fc_i = i
            break
    for i in range(1, len(f)):
        if lph[i] <= -180.0:
            gm_i = i
            break
    return {
        "fc_i": fc_i,
        "gm_i": gm_i,
        "fc": f[fc_i] if fc_i is not None else None,
        "pm": (180.0 + lph[fc_i]) if fc_i is not None else None,
        "gm": (-ldb[gm_i]) if gm_i is not None else None,
        "gm_f": f[gm_i] if gm_i is not None else None,
    }


# ----------------------------------------------------------------------------- 搜索

def logspace(lo, hi, n):
    if n < 2:
        return [lo]
    a, b = math.log10(lo), math.log10(hi)
    return [10.0 ** (a + (b - a) * k / (n - 1)) for k in range(n)]


def evaluate(plant, kdc, fz0):
    ldb, lph = loop(plant, kdc, fz0)
    m = gui_margins(plant.f, ldb, lph)
    m["kdc"], m["fz0"] = kdc, fz0
    return m, ldb, lph


def candidates(plant, fs, fc_min, fc_max, gm_min, pm_lo, pm_hi,
               fz_lo=2.0, fz_hi=None, n_fz=320, eps_db=-0.02, mode="PI"):
    """在 (fc, fz0) 网格上枚举设计，返回全部可行解。单 P 时 fz0 固定为 0。"""
    out = []
    if mode == "P":
        fz_grid = [0.0]
    else:
        if fz_hi is None:
            fz_hi = 0.5 * fs
        fz_grid = logspace(fz_lo, fz_hi, n_fz)
    for i, fc in enumerate(plant.f):
        if not (fc_min <= fc <= fc_max) or i == 0:
            continue
        for fz0 in fz_grid:
            kdc = 10.0 ** (eps_db / 20.0) / (plant.mag_lin(i) * math.hypot(1.0, fz0 / fc))
            ldb, lph = loop(plant, kdc, fz0)
            # 只认第一次穿越恰好落在 fc 这一点
            fc_i = None
            for j in range(1, len(plant)):
                if ldb[j - 1] > 0.0 >= ldb[j]:
                    fc_i = j
                    break
            if fc_i != i:
                continue
            gm_i = None
            for j in range(1, len(plant)):
                if lph[j] <= -180.0:
                    gm_i = j
                    break
            if gm_i is None:
                continue
            pm = 180.0 + lph[fc_i]
            gm = -ldb[gm_i]
            if pm_lo <= pm <= pm_hi and gm >= gm_min:
                out.append({"fc": fc, "fz0": fz0, "kdc": kdc, "pm": pm,
                            "gm": gm, "gm_f": plant.f[gm_i]})
    return out


def best_design(feasible, gm_min, pm_lo, pm_hi, pad_gm=0.2, pad_pm=1.0):
    """优先：留 0.2 dB / 1 deg 余量后 fc 最大；不够再放宽到边界。"""
    strict = [r for r in feasible
              if r["gm"] >= gm_min + pad_gm and pm_lo + pad_pm <= r["pm"] <= pm_hi - pad_pm]
    if strict:
        return max(strict, key=lambda r: (round(r["fc"], 3), r["gm"])), "strict"
    if feasible:
        return max(feasible, key=lambda r: (round(r["fc"], 3), r["gm"])), "boundary"
    return None, "none"


def near_miss(plant, fs, fc_min, fc_max, gm_min, pm_lo, pm_hi, mode="PI", n=3, n_fz=200):
    """无解时按加权偏差找最接近的几个点（GM 1 dB 记 4 分，PM 1° 记 1 分，fc 50 Hz 记 1 分）。"""
    fz_grid = [0.0] if mode == "P" else logspace(2.0, 0.5 * fs, n_fz)
    cands = []
    for i, fc in enumerate(plant.f):
        if i == 0:
            continue
        for fz0 in fz_grid:
            kdc = 10 ** (-0.02 / 20.0) / (plant.mag_lin(i) * math.hypot(1.0, fz0 / fc))
            m, _, _ = evaluate(plant, kdc, fz0)
            if m["fc"] is None or m["gm"] is None:
                continue
            if abs(m["fc"] - fc) > 1e-6:
                continue
            dgm = max(0.0, gm_min - m["gm"])
            dpm = max(0.0, pm_lo - m["pm"]) + max(0.0, m["pm"] - pm_hi)
            dfc = max(0.0, m["fc"] - fc_max) + max(0.0, fc_min - m["fc"])
            cands.append({"fc": m["fc"], "fz0": fz0, "kdc": kdc, "pm": m["pm"],
                          "gm": m["gm"], "gm_f": m["gm_f"],
                          "dgm": dgm, "dpm": dpm, "dfc": dfc,
                          "score": 4.0 * dgm + dpm + dfc / 50.0})
    cands.sort(key=lambda r: r["score"])
    return cands[:n]


def p_bounds(plant, gm_min, pm_lo, pm_hi):
    """单 P：GM / PM 两个约束各自允许的 fc 范围（GM(fc) = |P(fc)| - |P(f_gm)|）。"""
    j = next((i for i in range(1, len(plant)) if plant.phase[i] <= -180.0), None)
    if j is None:
        return None
    gm_idx = [i for i in range(1, len(plant)) if plant.mag_db[i] - plant.mag_db[j] >= gm_min]
    pm_idx = [i for i in range(1, len(plant)) if pm_lo <= 180.0 + plant.phase[i] <= pm_hi]
    out = {"gm_f": plant.f[j],
           "gm_fc_max": plant.f[max(gm_idx)] if gm_idx else None,
           "pm_at_gm_fc": (180.0 + plant.phase[max(gm_idx)]) if gm_idx else None,
           "pm_fc_min": plant.f[min(pm_idx)] if pm_idx else None,
           "pm_fc_max": plant.f[max(pm_idx)] if pm_idx else None}
    out["gm_at_pm_lo"] = plant.mag_db[min(pm_idx)] - plant.mag_db[j] if pm_idx else None
    out["gm_at_pm_hi"] = plant.mag_db[max(pm_idx)] - plant.mag_db[j] if pm_idx else None
    return out


# ----------------------------------------------------------------------------- 输出

def dcl_coeffs(kdc, fz0, fs):
    """Tustin 离散化到 DCL_PI_C1_C2_L1。"""
    a = math.pi * fz0 / fs
    c1 = kdc * (1.0 + a)
    c2 = kdc * (a - 1.0)
    kp = 0.5 * (c1 - c2)
    ki_t = c1 + c2
    return {"C1": c1, "C2": c2, "Kp": kp, "KiT": ki_t, "Ki": ki_t * fs}


def status_line(m):
    return ("Folg_cf %s kHz  Gain Margin %s dB  Phase Margin %s Degrees"
            % (fmt_khz(m["fc"]), safe_fmt(m["gm"]), safe_fmt(m["pm"])))


def print_design(rec, plant, fs, mode="PI"):
    k = dcl_coeffs(rec["kdc"], rec["fz0"], fs)
    if mode == "P":
        print("  KDC  = %.6f      fz0 = 0（单 P，无零点）" % rec["kdc"])
    else:
        print("  KDC  = %.6f      fz0 = %.3f kHz" % (rec["kdc"], rec["fz0"] / 1000.0))
    print("  fc   = %s Hz    PM = %s deg    GM = %s dB (@ %s Hz)"
          % (safe_fmt(rec.get("fc")), safe_fmt(rec.get("pm")), safe_fmt(rec.get("gm")), safe_fmt(rec.get("gm_f"), "%.0f")))
    print("  --> GUI 状态栏应为: Folg_cf %s kHz  Gain Margin %s dB  Phase Margin %s Degrees"
          % (fmt_khz(rec.get("fc")), safe_fmt(rec.get("gm")), safe_fmt(rec.get("pm"))))
    if mode == "P":
        print("  单 P: Kp = %.6f (= KDC),  Ki = 0" % k["Kp"])
    else:
        print("  模拟 PI: Kp = %.6f (= KDC),  Ki = %.1f rad/s" % (k["Kp"], k["Ki"]))
    print("  DCL @%g kHz: C1 = %.6f, C2 = %.6f   (u(k)=u(k-1)+C1*e(k)+C2*e(k-1))"
          % (fs / 1000.0, k["C1"], k["C2"]))
    g100 = plant.mag_db[0] + 20 * math.log10(rec["kdc"] * math.hypot(1.0, rec["fz0"] / plant.f[0]))
    print("  100 Hz 环路增益 = %.1f dB    Ki*T = %.6f" % (g100, k["KiT"]))


def print_verify(plant, kdc, fz_khz, mode="PI"):
    if mode == "P":
        fz_khz = 0.0
        print("KDC = %.6f   (单 P, fz0 = 0)" % kdc)
    else:
        print("KDC = %.6f   fz0 = %.3f kHz" % (kdc, fz_khz))
    m, _, _ = evaluate(plant, kdc, fz_khz * 1000.0)
    print("  fc = %s kHz  (idx %s)" % (fmt_khz(m.get("fc")), m.get("fc_i")))
    print("  GM = %s dB   @ %s Hz" % (safe_fmt(m.get("gm")), safe_fmt(m.get("gm_f"), "%.2f")))
    print("  PM = %s deg  @ %s Hz" % (safe_fmt(m.get("pm")), safe_fmt(m.get("fc"), "%.2f")))
    print("  GUI 状态栏应为: %s" % status_line(m))
    return m


def frontier_rows(plant, fs, pm_lo, pm_hi, mode="PI", fc_max=2600.0):
    """每个 fc 采样点取 PM 窗口内 GM 最大的设计，用来展示带宽/裕度权衡。"""
    rows = []
    for fc in [f for f in plant.f if 700.0 <= f <= fc_max]:
        cand = candidates(plant, fs, fc, fc, 0.0, pm_lo, pm_hi, n_fz=320, mode=mode)
        if cand:
            rows.append(max(cand, key=lambda r: r["gm"]))
    return rows


def print_frontier(plant, fs, gm_min, pm_lo, pm_hi, mode="PI"):
    rows = frontier_rows(plant, fs, pm_lo, pm_hi, mode)
    print("%8s %8s %9s %7s %7s %9s" % ("fc[Hz]", "fz0[Hz]", "KDC", "PM[deg]", "GM[dB]", "gm_f[Hz]"))
    for b in rows:
        print("%8.1f %8.0f %9.5f %7.2f %7.2f %9.1f"
              % (b["fc"], b["fz0"], b["kdc"], b["pm"], b["gm"], b["gm_f"]))
    print("\n不同 GM 底线下的最大 fc（PM 仍在窗口内）:")
    for floor in (5.6, 5.4, 5.2, 5.0, 4.8, 4.5, 4.0, 3.6):
        ok = [r for r in rows if r["gm"] >= floor]
        if ok:
            b = max(ok, key=lambda r: r["fc"])
            print("  GM>=%.1f dB : fc_max = %7.1f Hz   fz0=%6.0f Hz  KDC=%.5f  PM=%.1f"
                  % (floor, b["fc"], b["fz0"], b["kdc"], b["pm"]))
        else:
            print("  GM>=%.1f dB : 无解" % floor)


# ----------------------------------------------------------------------------- 画图

def try_plot(plant, rec, old, path, fs, mode="PI"):
    try:
        from PIL import Image, ImageDraw, ImageFont
    except Exception as exc:  # 没有 Pillow 就不画
        print("  [plot] 跳过（Pillow 不可用: %s）" % exc)
        return False

    W, H = 1280, 860
    img = Image.new("RGB", (W, H), "white")
    d = ImageDraw.Draw(img)

    def font(size, bold=False):
        names = (["C:/Windows/Fonts/segoeuib.ttf", "C:/Windows/Fonts/arialbd.ttf"] if bold
                 else ["C:/Windows/Fonts/segoeui.ttf", "C:/Windows/Fonts/arial.ttf"])
        for n in names:
            try:
                return ImageFont.truetype(n, size)
            except Exception:
                continue
        return ImageFont.load_default()

    f_small, f_lab, f_title = font(12), font(14), font(17, True)

    LEFT, RIGHT = 96, 30
    TOP1, BOT1 = 74, 402
    TOP2, BOT2 = 476, 782
    fmin, fmax = 50.0, 10000.0
    MAG_LO, MAG_HI = -45.0, 45.0
    PH_LO, PH_HI = -290.0, 90.0

    def X(f):
        a = (math.log10(f) - math.log10(fmin)) / (math.log10(fmax) - math.log10(fmin))
        return LEFT + a * (W - LEFT - RIGHT)

    def Y1(db):
        a = (db - MAG_LO) / (MAG_HI - MAG_LO)
        return BOT1 - a * (BOT1 - TOP1)

    def Y2(deg):
        a = (deg - PH_LO) / (PH_HI - PH_LO)
        return BOT2 - a * (BOT2 - TOP2)

    def rot_text(xy, text, fnt, fill):
        tmp = Image.new("RGBA", (240, 40), (255, 255, 255, 0))
        td = ImageDraw.Draw(tmp)
        td.text((2, 2), text, font=fnt, fill=fill)
        bbox = td.textbbox((2, 2), text, font=fnt)
        rot = tmp.crop((0, 0, bbox[2] + 4, bbox[3] + 4)).rotate(90, expand=True)
        img.paste(rot, (int(xy[0]), int(xy[1])), rot)

    def dashed(p0, p1, fill, width=2, dash=7, gap=5):
        x0, y0 = p0
        x1, y1 = p1
        ln = math.hypot(x1 - x0, y1 - y0)
        if ln < 1e-6:
            return
        ux, uy = (x1 - x0) / ln, (y1 - y0) / ln
        s = 0.0
        while s < ln:
            e = min(s + dash, ln)
            d.line([(x0 + ux * s, y0 + uy * s), (x0 + ux * e, y0 + uy * e)], fill=fill, width=width)
            s = e + gap

    def clip_seg(p0, p1, ylo, yhi):
        (x0, y0), (x1, y1) = p0, p1
        ts = [0.0, 1.0]
        if y1 != y0:
            for b in (ylo, yhi):
                t = (b - y0) / (y1 - y0)
                if 0.0 < t < 1.0:
                    ts.append(t)
        ts = sorted(set(ts))
        out = []
        for t in ts:
            y = y0 + t * (y1 - y0)
            if ylo - 1e-9 <= y <= yhi + 1e-9:
                out.append((x0 + t * (x1 - x0), y))
        return out

    def curve(vals, yof, ylo, yhi, color, width=2, dash=False, top=None, bot=None):
        pts = [(X(f), yof(v)) for f, v in zip(plant.f, vals) if fmin <= f <= fmax]
        for k in range(len(pts) - 1):
            seg = clip_seg(pts[k], pts[k + 1], top, bot)
            if len(seg) < 2:
                continue
            if dash:
                dashed(seg[0], seg[1], color, width, dash=5, gap=4)
            else:
                d.line(seg, fill=color, width=width)

    # ---------- 网格 ----------
    for dec in range(1, 5):
        for m in range(2, 10):
            f = m * (10.0 ** dec)
            if fmin <= f <= fmax:
                d.line([(X(f), TOP1), (X(f), BOT2)], fill=(242, 242, 242))
    for f in (100.0, 1000.0, 10000.0):
        x = X(f)
        d.line([(x, TOP1), (x, BOT2)], fill=(222, 222, 222))
        d.text((x - 14, BOT2 + 8), ("%d" % f), fill=(60, 60, 60), font=f_small)
    for db in range(-40, 41, 10):
        y = Y1(db)
        d.line([(LEFT, y), (W - RIGHT, y)], fill=(235, 235, 235))
        d.text((LEFT - 36, y - 7), "%+d" % db, fill=(70, 70, 70), font=f_small)
    for deg in range(-270, 91, 45):
        y = Y2(deg)
        d.line([(LEFT, y), (W - RIGHT, y)], fill=(235, 235, 235))
        d.text((LEFT - 42, y - 7), "%+d" % deg, fill=(70, 70, 70), font=f_small)

    # 面板边框
    for (t, b) in ((TOP1, BOT1), (TOP2, BOT2)):
        d.rectangle([LEFT, t, W - RIGHT, b], outline=(190, 190, 190))

    # 参考线
    d.line([(LEFT, Y1(0)), (W - RIGHT, Y1(0))], fill=(150, 150, 150))
    d.line([(LEFT, Y2(-180)), (W - RIGHT, Y2(-180))], fill=(150, 150, 150))
    d.text((W - RIGHT - 52, Y1(0) - 17), "0 dB", fill=(140, 140, 140), font=f_small)
    d.text((W - RIGHT - 74, Y2(-180) - 17), "-180 deg", fill=(140, 140, 140), font=f_small)

    # ---------- 曲线 ----------
    cmd, cpd = pi_comp(plant.f, rec["kdc"], rec["fz0"])
    ldb, lph = loop(plant, rec["kdc"], rec["fz0"])
    if old is not None:
        ldb_o, lph_o = loop(plant, old["kdc"], old["fz0"])
        curve(ldb_o, Y1, MAG_LO, MAG_HI, (232, 152, 62), 2, True, TOP1, BOT1)
        curve(lph_o, Y2, PH_LO, PH_HI, (232, 152, 62), 2, True, TOP2, BOT2)
    curve(plant.mag_db, Y1, MAG_LO, MAG_HI, (155, 155, 155), 2, False, TOP1, BOT1)
    curve(plant.phase, Y2, PH_LO, PH_HI, (155, 155, 155), 2, False, TOP2, BOT2)
    curve(cmd, Y1, MAG_LO, MAG_HI, (46, 158, 72), 2, False, TOP1, BOT1)
    curve(cpd, Y2, PH_LO, PH_HI, (46, 158, 72), 2, False, TOP2, BOT2)
    curve(ldb, Y1, MAG_LO, MAG_HI, (32, 72, 200), 3, False, TOP1, BOT1)
    curve(lph, Y2, PH_LO, PH_HI, (32, 72, 200), 3, False, TOP2, BOT2)

    # ---------- 标记 ----------
    if rec.get("fc") is not None:
        xfc = X(rec["fc"])
        dashed((xfc, TOP1), (xfc, Y1(0)), (32, 72, 200), 1, 4, 4)
        dashed((xfc, Y2(-180 + rec["pm"])), (xfc, BOT2), (32, 72, 200), 1, 4, 4)
        d.ellipse([xfc - 5, Y1(0) - 5, xfc + 5, Y1(0) + 5], fill=(32, 72, 200))
        d.ellipse([xfc - 5, Y2(-180 + rec["pm"]) - 5, xfc + 5, Y2(-180 + rec["pm"]) + 5], fill=(32, 72, 200))
        d.text((xfc + 8, Y1(0) - 24), "fc = %.0f Hz" % rec["fc"], fill=(32, 72, 200), font=f_small)
        d.text((xfc + 8, Y2(-180 + rec["pm"]) + 8), "PM = %.1f deg" % rec["pm"], fill=(32, 72, 200), font=f_small)

    if rec.get("gm_f") is not None:
        xgm = X(rec["gm_f"])
        ygm = Y1(-rec["gm"])
        dashed((xgm, TOP1), (xgm, BOT2), (206, 62, 62), 1, 4, 4)
        d.ellipse([xgm - 5, ygm - 5, xgm + 5, ygm + 5], fill=(206, 62, 62))
        d.text((xgm + 8, ygm - 20), "GM = %.2f dB @ %.0f Hz" % (rec["gm"], rec["gm_f"]),
               fill=(206, 62, 62), font=f_small)

    if old is not None:
        m_o, _, _ = evaluate(plant, old["kdc"], old["fz0"])
        if m_o.get("fc") is not None:
            xo = X(m_o["fc"])
            d.ellipse([xo - 4, Y1(0) - 4, xo + 4, Y1(0) + 4], fill=(232, 152, 62))
            d.text((xo + 7, Y1(0) + 8), "old fc %.0f Hz" % m_o["fc"], fill=(200, 120, 40), font=f_small)

    # ---------- 标题 / 图例 / 轴标签 ----------
    if mode == "P":
        title = "SFRA plant -> Single P:  KDC = %.5f" % rec["kdc"]
    else:
        title = "SFRA plant -> Series PI:  KDC = %.5f,  fz0 = %.3f kHz" % (rec["kdc"], rec["fz0"] / 1000.0)
    d.text((LEFT, 16), title, fill=(30, 30, 30), font=f_title)
    d.text((LEFT, 44), "fc = %s Hz    PM = %s deg    GM = %s dB    fs = %g kHz"
           % (safe_fmt(rec.get("fc"), "%.1f"), safe_fmt(rec.get("pm")), safe_fmt(rec.get("gm")), fs / 1000.0),
           fill=(90, 90, 90), font=f_small)

    lx, ly = W - RIGHT - 210, 18
    comp_name = "Comp (P)" if mode == "P" else "Comp (PI)"
    for name, color, dash in (("Plant", (155, 155, 155), False),
                              (comp_name, (46, 158, 72), False),
                              ("Open loop", (32, 72, 200), False),
                              ("Old design", (232, 152, 62), True)):
        if name == "Old design" and old is None:
            continue
        if dash:
            dashed((lx, ly + 7), (lx + 34, ly + 7), color, 2, 5, 4)
        else:
            d.line([(lx, ly + 7), (lx + 34, ly + 7)], fill=color, width=3 if name == "Open loop" else 2)
        d.text((lx + 42, ly), name, fill=(60, 60, 60), font=f_small)
        ly += 19

    rot_text((14, TOP1 + 110), "Magnitude [dB]", f_lab, (60, 60, 60))
    rot_text((14, TOP2 + 110), "Phase [deg]", f_lab, (60, 60, 60))
    d.text((W // 2 - 58, H - 34), "Frequency [Hz]", fill=(60, 60, 60), font=f_lab)

    img.save(path)
    return True


def newest_csv(dirpath):
    """目录里最新的一个 CSV（排除本脚本自己产出的 pi_design_*）。"""
    cands = []
    for name in os.listdir(dirpath):
        low = name.lower()
        if not low.endswith(".csv") or low.startswith("pi_design"):
            continue
        p = os.path.join(dirpath, name)
        if os.path.isfile(p):
            cands.append((os.path.getmtime(p), p))
    cands.sort(reverse=True)
    return cands[0][1] if cands else None


# ----------------------------------------------------------------------------- 主程序

def main(argv=None):
    ap = argparse.ArgumentParser(description="由 SFRA CSV 设计补偿器参数（单 P 或 Series PI）")
    ap.add_argument("--csv", default=None,
                    help="SFRA CSV；不给就用脚本目录里最新的一个 CSV（排除 pi_design_*）")
    ap.add_argument("--mode", default=None,
                    help="补偿器模式：PI（默认，取文件顶部的宏 COMP_MODE）或 P（单比例）")
    ap.add_argument("--fs", type=float, default=20000.0, help="ISR/开关频率 Hz（默认 20 kHz）")
    ap.add_argument("--fc-min", type=float, default=0.5)
    ap.add_argument("--fc-max", type=float, default=2000.0)
    ap.add_argument("--gm-min", type=float, default=5.0, help="幅值裕度下限 dB")
    ap.add_argument("--pm-min", type=float, default=45.0)
    ap.add_argument("--pm-max", type=float, default=60.0)
    ap.add_argument("--verify", nargs="+", type=float, metavar="KDC [FZ_KHZ]",
                    help="只复现某个设计的 GUI 结果（PI 再给 fz0，单位 kHz）")
    ap.add_argument("--frontier", action="store_true", help="打印 fc/fz0 权衡表后退出")
    ap.add_argument("--old", nargs=2, type=float, metavar=("KDC", "FZ_KHZ"),
                    default=[0.134, 0.4], help="对照用的旧设计（默认 0.134 / 0.4 kHz，如果是单 P 建议用 --old 2.0 0）")
    ap.add_argument("--prefix", default=os.path.join(SCRIPT_DIR, "pi_design"))
    ap.add_argument("--no-plot", action="store_true")
    args = ap.parse_args(argv)

    mode = (args.mode or COMP_MODE).strip().upper()
    if mode not in ("P", "PI"):
        ap.error("--mode 只能是 P 或 PI（或改文件顶部的 COMP_MODE 宏）")
    label = ("单 P（Gc = KDC）" if mode == "P"
             else "Series PI (Pole Zero)，Gc = KDC·(1 + ωz/s)")
    prefix = args.prefix + ("_p" if mode == "P" else "_pi")

    if not args.csv:
        args.csv = newest_csv(SCRIPT_DIR)
        if not args.csv:
            ap.error("目录里没有 CSV，用 --csv 指定")
        print("自动选择最新 CSV: %s" % os.path.basename(args.csv))
    plant = load_sfra_csv(args.csv)
    print("数据: %s  %d 点  %.0f..%.0f Hz" % (plant.source, len(plant), plant.f[0], plant.f[-1]))
    print("模式: %s" % label)

    if args.verify:
        kdc = args.verify[0]
        fz_khz = args.verify[1] if len(args.verify) > 1 else 0.0
        print_verify(plant, kdc, fz_khz, mode)
        return 0

    if args.frontier:
        print_frontier(plant, args.fs, args.gm_min, args.pm_min, args.pm_max, mode)
        return 0

    old = None
    if args.old:
        kdc_o, fz_o = args.old
        m_o, _, _ = evaluate(plant, kdc_o, fz_o * 1000.0)
        old = {"kdc": kdc_o, "fz0": fz_o * 1000.0}
        print("对照旧设计: KDC=%.4f fz0=%.3f kHz -> fc=%s Hz  PM=%s  GM=%s"
              % (kdc_o, fz_o, safe_fmt(m_o.get("fc")), safe_fmt(m_o.get("pm")), safe_fmt(m_o.get("gm"))))

    print("\n搜索: fc %.3g..%.3g Hz, GM>=%.1f dB, PM %.0f..%.0f deg%s"
          % (args.fc_min, args.fc_max, args.gm_min, args.pm_min, args.pm_max,
             "" if mode == "P" else ", fz0<=fs/2"))
    feas = candidates(plant, args.fs, args.fc_min, args.fc_max,
                      args.gm_min, args.pm_min, args.pm_max, mode=mode)
    print("  可行解: %d 个" % len(feas))

    rec, how = best_design(feas, args.gm_min, args.pm_min, args.pm_max)
    if rec is None:
        print("\n[!] 该约束下无解，最接近的点：")
        near = near_miss(plant, args.fs, args.fc_min, args.fc_max,
                         args.gm_min, args.pm_min, args.pm_max, mode)
        if not near:
            print("  未能找到任何满足条件的穿越点。请检查数据或放宽搜索参数。")
            return 1
        for c in near:
            print("   fc=%7.1f Hz  fz0=%5.0f Hz  KDC=%.5f  PM=%6.2f deg  GM=%5.2f dB"
                  % (c["fc"], c["fz0"], c["kdc"], c["pm"], c["gm"]))
            print("      偏差: GM %.2f dB / PM %.1f deg / fc %.0f Hz"
                  % (-c["dgm"], c["dpm"], c["dfc"]))
        if mode == "P":
            bnd = p_bounds(plant, args.gm_min, args.pm_min, args.pm_max)
            if bnd and bnd.get("gm_fc_max") is not None and bnd.get("pm_fc_min") is not None:
                print("  单 P 分解: GM>=%.0f dB 只允许 fc<=%.0f Hz（那里 PM=%.1f deg）"
                      % (args.gm_min, bnd["gm_fc_max"], bnd["pm_at_gm_fc"]))
                print("            PM %.0f..%.0f deg 只允许 fc %.0f..%.0f Hz（那里 GM=%.2f..%.2f dB）"
                      % (args.pm_min, args.pm_max, bnd["pm_fc_min"], bnd["pm_fc_max"],
                         bnd["gm_at_pm_hi"], bnd["gm_at_pm_lo"]))
                print("            两段没有交集 -> 单 P 在该窗口内无解")
        rec = near[0]
        how = "near-miss"

    print("\n=== 推荐设计 (%s) ===" % how)
    print_design(rec, plant, args.fs, mode)

    if feas:
        alt = sorted(feas, key=lambda r: (r["fc"], -r["gm"]))
        pick = []
        for r in alt:
            if all(abs(r["fc"] - q["fc"]) > 1e-6 or abs(r["fz0"] - q["fz0"]) < 1e-9 for q in pick):
                pick.append(r)
        print("\n=== 备选（按 fc 升序，最多 8 个）===")
        print("%9s %9s %9s %8s %8s" % ("fc[Hz]", "fz0[Hz]", "KDC", "PM[deg]", "GM[dB]"))
        for r in pick[:8]:
            print("%9.1f %9.0f %9.5f %8.2f %8.2f"
                  % (r["fc"], r["fz0"], r["kdc"], r["pm"], r["gm"]))

    # 逐点数据
    ldb, lph = loop(plant, rec["kdc"], rec["fz0"])
    cmd, cpd = pi_comp(plant.f, rec["kdc"], rec["fz0"])
    csv_path = prefix + "_loop.csv"
    with open(csv_path, "w", newline="", encoding="utf-8") as fh:
        w = csv.writer(fh)
        w.writerow(["Freq(Hz)", "Plant_dB", "Plant_deg", "Comp_dB", "Comp_deg", "OL_dB", "OL_deg"])
        for i in range(len(plant)):
            w.writerow(["%.6f" % plant.f[i], "%.4f" % plant.mag_db[i], "%.4f" % plant.phase[i],
                        "%.4f" % cmd[i], "%.4f" % cpd[i], "%.4f" % ldb[i], "%.4f" % lph[i]])
    print("\n写出: %s" % csv_path)

    if not args.no_plot:
        png = prefix + "_bode.png"
        if try_plot(plant, rec, old, png, args.fs, mode):
            print("写出: %s" % png)

    # Markdown 报告
    md = prefix + "_report.md"
    k = dcl_coeffs(rec["kdc"], rec["fz0"], args.fs)
    m_o = old and evaluate(plant, old["kdc"], old["fz0"])[0]
    rows = frontier_rows(plant, args.fs, args.pm_min, args.pm_max, mode)
    ones, _ = crossings(plant.f, ldb, lph)
    with open(md, "w", encoding="utf-8") as fh:
        fh.write("# SFRA 扫频 → %s 设计结果\n\n"
                 % ("单 P（Kp）" if mode == "P" else "PI (KDC, fz0)"))
        fh.write("- 数据：`%s`，%d 点，%.0f–%.0f Hz，fs = %g kHz\n"
                 % (plant.source, len(plant), plant.f[0], plant.f[-1], args.fs / 1000))
        fh.write("- 模式：%s\n" % label)
        fh.write("- 约束：fc ∈ [%.3g, %.0f] Hz，GM ≥ %.1f dB，PM ∈ [%.0f, %.0f]°\n"
                 % (args.fc_min, args.fc_max, args.gm_min, args.pm_min, args.pm_max))
        fh.write("- 判据：与 Compensation Designer 状态栏一致 —— 逐采样点找**第一次** 0 dB 穿越 / "
                 "第一次 −180° 穿越\n\n")

        fh.write("## 参数\n\n| 参数 | 值 |\n|---|---|\n")
        fh.write("| **KDC** | **%.6f** |\n" % rec["kdc"])
        if mode == "P":
            fh.write("| fz0 | 0（单 P 无零点） |\n")
        else:
            fh.write("| **fz0** | **%.3f kHz** |\n" % (rec["fz0"] / 1000))
        fh.write("| 穿越频率 fc | %s Hz |\n| 相位裕度 PM | %s° |\n"
                 "| 幅值裕度 GM | %s dB @ %s Hz |\n\n"
                 % (safe_fmt(rec.get("fc")), safe_fmt(rec.get("pm")), safe_fmt(rec.get("gm")), safe_fmt(rec.get("gm_f"), "%.0f")))
        fh.write("GUI 状态栏应显示：`Folg_cf %s kHz  Gain Margin %s dB  Phase Margin %s Degrees`\n\n"
                 % (fmt_khz(rec.get("fc")), safe_fmt(rec.get("gm")), safe_fmt(rec.get("pm"))))
        fh.write("0 dB 穿越（任意方向）共 %d 次；GUI 只认第一次“由正到负”的穿越。\n\n" % len(ones))
        if mode == "P":
            fh.write("单 P：`u = Kp·e`，Kp = %.6f；按 DCL 写法 C1 = %.6f、C2 = %.6f"
                     "（C2 = −C1 把积分抵消掉）\n\n" % (k["Kp"], k["C1"], k["C2"]))
        else:
            fh.write("模拟 PI：Kp = KDC = %.6f，Ki = %.1f rad/s；"
                     "Tustin 离散（%g kHz）：C1 = %.6f，C2 = %.6f，"
                     "u(k)=u(k−1)+C1·e(k)+C2·e(k−1)\n\n"
                     % (k["Kp"], k["Ki"], args.fs / 1000, k["C1"], k["C2"]))

        if m_o:
            fh.write("## 与旧设计对比\n\n| | 旧 KDC=%.4f, fz0=%.3f kHz | 本次 |\n|---|---|---|\n"
                     % (old["kdc"], old["fz0"] / 1000))
            fh.write("| fc | %s Hz | %s Hz |\n| PM | %s° | %s° |\n| GM | %s dB | %s dB |\n\n"
                     % (safe_fmt(m_o.get("fc")), safe_fmt(rec.get("fc")),
                        safe_fmt(m_o.get("pm")), safe_fmt(rec.get("pm")),
                        safe_fmt(m_o.get("gm")), safe_fmt(rec.get("gm"))))

        bnd = None
        if mode == "P":
            bnd = p_bounds(plant, args.gm_min, args.pm_min, args.pm_max)
            if bnd:
                fh.write("## 单 P 为什么不满框\n\n")
                fh.write("单 P 的环路只是 `L = KDC·Plant`：`GM = |P(fc)|dB − |P(f_gm)|dB`"
                         "（f_gm = plant 自己的 −180° 点，%.0f Hz），`PM = 180 + φp(fc)`。"
                         "两个约束各自能占的 fc 区间刚好错开：\n\n" % bnd["gm_f"])
                fh.write("| 约束 | 允许的 fc | 另一项在那里的值 |\n|---|---|---|\n")
                if bnd.get("gm_fc_max") is not None:
                    fh.write("| GM ≥ %.0f dB | ≤ %.0f Hz | PM = %.1f° |\n"
                             % (args.gm_min, bnd["gm_fc_max"], bnd["pm_at_gm_fc"]))
                if bnd.get("pm_fc_min") is not None:
                    fh.write("| PM %.0f–%.0f° | %.0f–%.0f Hz | GM = %.2f–%.2f dB |\n\n"
                             % (args.pm_min, args.pm_max, bnd["pm_fc_min"], bnd["pm_fc_max"],
                                bnd["gm_at_pm_hi"], bnd["gm_at_pm_lo"]))
                fh.write("所以框内没有交点；要 GM > 5 dB 只能上 PI（见 `pi_design_pi_report.md`）。\n\n")
        else:
            fh.write("## 为什么 fc 上不去 ~1 kHz\n\n")
            fh.write("1. Plant 在 800 Hz–2 kHz 的幅值几乎不降（16.5 → 18 dB），环路的 −20 dB/dec "
                     "主要靠 PI 的积分段提供。\n")
            fh.write("2. 要把 PM 压到 45–60°，零点必须落在 fc 附近或 fc 之上"
                     "（fz0 ≈ 0.3–0.6·fc）；这时 comp 在 2.1–2.4 kHz 已贡献 −60~−75°，"
                     "−180° 穿越点被拉到 2.1–2.4 kHz，而 plant 在那里还有 17.5 dB，GM 只剩 3.6–4.3 dB。\n")
            fh.write("3. 反过来把 GM 做高只能减小 KDC，fc 随之下移。所以 GM ≥ 5 dB 把 fc 卡在 ~1 kHz，"
                     "再往上要么 GM < 5 dB，要么 PM > 60°。\n\n")

        fh.write("## 权衡表（每个 fc 取 PM 窗口内 GM 最大的解）\n\n")
        fh.write("| fc | fz0 | KDC | PM | GM |\n|---|---|---|---|---|\n")
        for r in rows[::max(1, len(rows) // 12)]:
            fh.write("| %.0f Hz | %.0f Hz | %.5f | %.1f° | %.2f dB |\n"
                     % (r["fc"], r["fz0"], r["kdc"], r["pm"], r["gm"]))
        fh.write("\n不同 GM 底线下的最大 fc：\n\n| GM 底线 | fc 上限 | 对应 fz0 | KDC | PM |\n|---|---|---|---|---|\n")
        for floor in (5.6, 5.4, 5.2, 5.0, 4.8, 4.5, 4.0):
            ok = [r for r in rows if r["gm"] >= floor]
            if ok:
                b = max(ok, key=lambda r: r["fc"])
                fh.write("| ≥%.1f dB | %.0f Hz | %.0f Hz | %.5f | %.1f° |\n"
                         % (floor, b["fc"], b["fz0"], b["kdc"], b["pm"]))
            else:
                fh.write("| ≥%.1f dB | 无解 | – | – | – |\n" % floor)

        fh.write("\n## 注意\n\n")
        if mode == "P":
            fh.write("1. 单 P 没有积分，低频没有额外增益，50 Hz 跟踪靠 plant 的 15.5 dB 直流增益 + KDC，会留静差。\n")
            fh.write("2. 单 P 的 GM、PM 都被 plant 绑死，两个窗口错开，框内无解；要 GM > 5 dB 只能上 PI。\n")
            fh.write("3. 数据 2.1–3 kHz 的相位有卷绕/噪声，PM 在 2 kHz 以上只能参考。\n")
            fh.write("4. 上板后建议再扫一次 SFRA 复核。\n")
        else:
            fh.write("1. PM 45–60° 在这条 plant 上只能靠把零点放到 fc 之上（≈4–6 kHz）实现；"
                     "零点放到 100–500 Hz 的“常规 PI”在 1 kHz 附近 PM 会到 110–130°。\n")
            fh.write("2. GM 对 2.1–3 kHz 的相位很敏感（这段数据本身有卷绕/噪声，"
                     "100 点里 2.76 kHz 处刚好翻过一次 ±180）。\n")
            fh.write("3. 本结果按原始 CSV 算（GUI 就是这么用的）。文档里的 ZOH 校正"
                     "（相位 +180·f/fs）会让 −180° 穿越点后移，GM 读数更高，不要和 GUI 混用。\n")
            fh.write("4. fz0 接近 fs/4，双线性离散后等效零点约 4.6 kHz，实际数字环路 PM 会高 1–3°；"
                     "上板后仍建议再扫一次 SFRA 复核。\n")
            fh.write("5. 旧设计（0.134 / 0.4 kHz）在 GUI 里 PM 显示 121°，但 |L| 在 780 Hz 之后"
                     "还反复穿越 0 dB（780–1070 Hz 间共 6 次，数据噪声），另有真实一次在 2.27 kHz、PM≈41°。"
                     "GUI 只报第一次，别只看状态栏。本脚本推荐的解只有 1 次 0 dB 穿越。\n")
    print("写出: %s" % md)
    return 0


if __name__ == "__main__":
    sys.exit(main())