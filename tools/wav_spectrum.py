#!/usr/bin/env python3
"""Report the frequency content of a WAV file as octave-band energy.

Pure standard library (no numpy). Averages the magnitude spectrum over several
windows and prints a bar chart of energy per octave band, plus the high-end
rolloff point. Use it to distinguish an analog filter rolloff (gentle, high
corner) from a digital/sample-rate problem (sharp cliff or odd shape).

Usage:
    python3 tools/wav_spectrum.py turntable.wav
"""

import argparse
import cmath
import math
import struct
import sys
import wave


def read_mono(path):
    w = wave.open(path, "rb")
    ch, width, sr, n = (w.getnchannels(), w.getsampwidth(),
                        w.getframerate(), w.getnframes())
    raw = w.readframes(n)
    w.close()

    if width == 2:
        fmt = "<%dh" % (len(raw) // 2)
        vals = struct.unpack(fmt, raw)
        scale = 32768.0
    elif width == 4:
        fmt = "<%di" % (len(raw) // 4)
        vals = struct.unpack(fmt, raw)
        scale = 2147483648.0
    elif width == 3:
        vals = []
        for i in range(0, len(raw), 3):
            b = raw[i:i + 3]
            v = b[0] | (b[1] << 8) | (b[2] << 16)
            if v & 0x800000:
                v -= 0x1000000
            vals.append(v)
        scale = 8388608.0
    elif width == 1:
        vals = [b - 128 for b in raw]
        scale = 128.0
    else:
        raise ValueError("unsupported sample width %d" % width)

    # Down-mix interleaved channels to mono, normalize to [-1, 1).
    mono = []
    for i in range(0, len(vals), ch):
        frame = vals[i:i + ch]
        mono.append(sum(frame) / (len(frame) * scale))
    return mono, sr


def fft(a):
    n = len(a)
    if n == 1:
        return a
    even = fft(a[0::2])
    odd = fft(a[1::2])
    out = [0] * n
    for k in range(n // 2):
        t = cmath.exp(-2j * math.pi * k / n) * odd[k]
        out[k] = even[k] + t
        out[k + n // 2] = even[k] - t
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("wav")
    ap.add_argument("--size", type=int, default=4096,
                    help="FFT window size (power of 2)")
    args = ap.parse_args()

    mono, sr = read_mono(args.wav)
    n = args.size
    if len(mono) < n:
        print("File too short for analysis", file=sys.stderr)
        return

    # Hann-windowed, averaged magnitude spectrum across the whole file.
    window = [0.5 - 0.5 * math.cos(2 * math.pi * i / (n - 1)) for i in range(n)]
    mag = [0.0] * (n // 2)
    blocks = 0
    for start in range(0, len(mono) - n, n):
        chunk = [mono[start + i] * window[i] for i in range(n)]
        spec = fft(chunk)
        for k in range(n // 2):
            mag[k] += abs(spec[k])
        blocks += 1
    mag = [m / blocks for m in mag]

    bin_hz = sr / n
    print(f"{args.wav}: {sr} Hz, {len(mono)} frames "
          f"({len(mono)/sr:.1f}s), {blocks} FFT blocks, bin={bin_hz:.1f} Hz\n")

    # Octave bands from 31.25 Hz up to Nyquist.
    bands = []
    f = 31.25
    while f < sr / 2:
        lo, hi = f / math.sqrt(2), f * math.sqrt(2)
        klo, khi = int(lo / bin_hz), min(int(hi / bin_hz), n // 2 - 1)
        energy = sum(mag[klo:khi + 1]) / max(1, khi - klo + 1)
        bands.append((f, energy))
        f *= 2

    peak = max(e for _, e in bands) or 1e-12
    print("  Center     dB   |bar")
    for fc, e in bands:
        db = 20 * math.log10(e / peak) if e > 0 else -99
        bar = "#" * max(0, int((db + 60) / 2))
        label = f"{fc/1000:.1f}k" if fc >= 1000 else f"{fc:.0f}"
        print(f"  {label:>6}  {db:6.1f}  |{bar}")

    # Report the highest band still within 10 dB of the peak (the usable top end).
    top = max((fc for fc, e in bands
               if e > 0 and 20 * math.log10(e / peak) > -10), default=0)
    print(f"\nHigh-end (-10 dB) extends to ~{top/1000:.1f} kHz "
          f"(Nyquist is {sr/2000:.1f} kHz)")


if __name__ == "__main__":
    main()
