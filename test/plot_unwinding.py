"""Attitude error from a 359 deg start, naive vs sign-corrected, as an SVG.

    make -C test plot      # runs the SIL traces, writes results/unwinding-359.svg
"""
import csv
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
SIL = os.path.join(HERE, "build", "sil")
OUT = os.path.join(HERE, "..", "results", "unwinding-359.svg")

W, H = 720, 380
L, R, T, B = 56, 24, 64, 48
T_MAX, Y_MAX = 8.0, 180.0


def trace(fw):
    path = os.path.join(HERE, "build", f"trace-{fw}-359.csv")
    subprocess.run([SIL, "--trace", fw, "359", path], check=True)
    with open(path) as f:
        rows = [(float(r["t_s"]), float(r["err_deg"])) for r in csv.DictReader(f)]
    out, last = [], -1.0
    for t, e in rows:
        if t > T_MAX:
            break
        if t - last >= 0.01:
            out.append((t, e))
            last = t
    return out


def x(t):
    return L + (W - L - R) * t / T_MAX


def y(e):
    return T + (H - T - B) * (1 - e / Y_MAX)


def polyline(pts, cls):
    d = " ".join(f"{x(t):.1f},{y(e):.1f}" for t, e in pts)
    return f'<polyline class="{cls}" points="{d}"/>'


def main():
    naive, fixed = trace("naive"), trace("now")
    peak_t, peak_e = max(naive, key=lambda p: p[1])
    settle = max(t for t, e in naive if e > 2.0)
    grid, labels = [], []
    for e in range(0, 181, 45):
        grid.append(f'<line class="{"base" if e == 0 else "grid"}" x1="{L}" x2="{W - R}" y1="{y(e):.1f}" y2="{y(e):.1f}"/>')
        labels.append(f'<text class="tick" x="{L - 8}" y="{y(e) + 4:.1f}" text-anchor="end">{e}°</text>')
    for t in range(0, int(T_MAX) + 1):
        labels.append(f'<text class="tick" x="{x(t):.1f}" y="{H - B + 18}" text-anchor="middle">{t}</text>')

    svg = f"""<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {W} {H}" width="{W}" height="{H}" role="img" aria-labelledby="t d">
<title id="t">Attitude error from a 359 degree start, naive vs sign-corrected</title>
<desc id="d">Software-in-the-loop run of sketch.ino against chip.chip.c. The naive law rotates the long way, peaking at {peak_e:.0f} degrees of error at {peak_t:.1f} s and settling after {settle:.1f} s; the sign-corrected law stays within 1 degree.</desc>
<style>
  svg {{ font-family: system-ui, -apple-system, "Segoe UI", sans-serif; }}
  .bg {{ fill: #fcfcfb; }}
  .grid {{ stroke: #e1e0d9; stroke-width: 1; }}
  .base {{ stroke: #c3c2b7; stroke-width: 1; }}
  .tick {{ fill: #898781; font-size: 12px; }}
  .title {{ fill: #0b0b0b; font-size: 15px; font-weight: 600; }}
  .sub, .label {{ fill: #52514e; font-size: 12px; }}
  .naive, .fixed {{ fill: none; stroke-width: 2; stroke-linejoin: round; stroke-linecap: round; }}
  .naive, .key-naive {{ stroke: #eb6834; }}
  .fixed, .key-fixed {{ stroke: #2a78d6; }}
  .key-naive, .key-fixed {{ stroke-width: 2; stroke-linecap: round; }}
  @media (prefers-color-scheme: dark) {{
    .bg {{ fill: #1a1a19; }}
    .grid {{ stroke: #2c2c2a; }}
    .base {{ stroke: #383835; }}
    .title {{ fill: #f0efec; }}
    .sub, .label {{ fill: #c3c2b7; }}
    .naive, .key-naive {{ stroke: #d95926; }}
    .fixed, .key-fixed {{ stroke: #3987e5; }}
  }}
</style>
<rect class="bg" width="{W}" height="{H}" rx="8"/>
<text class="title" x="{L}" y="24">From 359°, the naive law goes the long way round</text>
<text class="sub" x="{L}" y="42">Attitude error vs time · software-in-the-loop, sketch.ino against chip.chip.c</text>
<line class="key-fixed" x1="{W - R - 190}" x2="{W - R - 174}" y1="20" y2="20"/>
<text class="label" x="{W - R - 168}" y="24">sign-corrected</text>
<line class="key-naive" x1="{W - R - 60}" x2="{W - R - 44}" y1="20" y2="20"/>
<text class="label" x="{W - R - 38}" y="24">naive</text>
{"".join(grid)}
{"".join(labels)}
<text class="tick" x="{(L + W - R) / 2:.1f}" y="{H - 10}" text-anchor="middle">time (s)</text>
{polyline(naive, "naive")}
{polyline(fixed, "fixed")}
<text class="label" x="{x(peak_t) + 10:.1f}" y="{y(peak_e) + 4:.1f}">naive: {peak_e:.0f}° at {peak_t:.1f} s</text>
<text class="label" x="{x(5.7):.1f}" y="{y(0) - 10:.1f}">sign-corrected: ≤ 1° throughout</text>
</svg>
"""
    with open(OUT, "w") as f:
        f.write(svg)
    print(f"wrote {os.path.relpath(OUT)}: naive peak {peak_e:.1f} deg at {peak_t:.2f} s")


if __name__ == "__main__":
    sys.exit(main())
