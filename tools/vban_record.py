#!/usr/bin/env python3
"""Capture a VBAN audio stream to a WAV file for offline quality analysis.

This bypasses Music Assistant entirely: it decodes the raw VBAN packets the
ESP32 sends and writes the exact samples to disk. Use it to tell whether audio
problems originate at the ADC/analog filter or in the receiver/playback side.

Usage:
    python3 tools/vban_record.py out.wav                 # record until Ctrl-C
    python3 tools/vban_record.py out.wav --seconds 20    # record fixed length
    python3 tools/vban_record.py out.wav --stream turntable

To capture on this machine, temporarily point the ESP's `ma_server_ip` secret
at this computer's IP and reflash (the ESP sends unicast UDP to one address).
"""

import argparse
import socket
import struct
import sys
import wave

# VBAN sample-rate table: lower 5 bits of the format_SR byte index into this.
VBAN_SR = [
    6000, 12000, 24000, 48000, 96000, 192000, 384000,
    8000, 16000, 32000, 64000, 128000, 256000, 512000,
    11025, 22050, 44100, 88200, 176400, 352800, 705600,
]

# VBAN data type (lower 3 bits of format_bit) -> (sample width bytes, label).
VBAN_DATATYPE = {
    0: (1, "INT8"),
    1: (2, "INT16"),
    2: (3, "INT24"),
    3: (4, "INT32"),
    4: (4, "FLOAT32"),
}

HEADER_SIZE = 28


def parse_header(pkt):
    if len(pkt) < HEADER_SIZE or pkt[:4] != b"VBAN":
        return None
    fmt_sr, nbs, nbc, fmt_bit = pkt[4], pkt[5], pkt[6], pkt[7]
    stream = pkt[8:24].split(b"\x00", 1)[0].decode("ascii", "replace")
    frame = struct.unpack_from("<I", pkt, 24)[0]
    sr = VBAN_SR[fmt_sr & 0x1F] if (fmt_sr & 0x1F) < len(VBAN_SR) else None
    width, label = VBAN_DATATYPE.get(fmt_bit & 0x07, (None, "UNKNOWN"))
    return {
        "sample_rate": sr,
        "samples_per_frame": nbs + 1,
        "channels": nbc + 1,
        "width": width,
        "label": label,
        "stream": stream,
        "frame": frame,
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("output", help="output WAV path")
    ap.add_argument("--port", type=int, default=6980)
    ap.add_argument("--stream", default=None,
                    help="only record packets matching this VBAN stream name")
    ap.add_argument("--seconds", type=float, default=None,
                    help="stop after this many seconds of audio (default: Ctrl-C)")
    args = ap.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("0.0.0.0", args.port))
    print(f"Listening for VBAN on UDP :{args.port} "
          f"(stream filter: {args.stream or 'any'})")

    wav = None
    params = None
    frames_written = 0
    last_frame = None
    gaps = 0

    try:
        while True:
            pkt, _ = sock.recvfrom(2048)
            hdr = parse_header(pkt)
            if hdr is None:
                continue
            if args.stream and hdr["stream"] != args.stream:
                continue
            if hdr["width"] is None:
                print(f"Unsupported VBAN data type: {hdr['label']}", file=sys.stderr)
                continue

            if wav is None:
                params = hdr
                print(f"Stream '{hdr['stream']}': {hdr['sample_rate']} Hz, "
                      f"{hdr['channels']} ch, {hdr['label']} "
                      f"({hdr['width'] * 8}-bit)")
                if hdr["label"].startswith("FLOAT"):
                    print("FLOAT32 stream — not writing PCM WAV", file=sys.stderr)
                    return
                wav = wave.open(args.output, "wb")
                wav.setnchannels(hdr["channels"])
                wav.setsampwidth(hdr["width"])
                wav.setframerate(hdr["sample_rate"])

            # Detect dropped packets via the frame counter (gaps = network loss).
            if last_frame is not None and hdr["frame"] != (last_frame + 1) & 0xFFFFFFFF:
                gaps += 1
            last_frame = hdr["frame"]

            payload = pkt[HEADER_SIZE:]
            wav.writeframes(payload)
            frames_written += len(payload) // (params["width"] * params["channels"])

            if args.seconds and frames_written >= args.seconds * params["sample_rate"]:
                break
    except KeyboardInterrupt:
        pass
    finally:
        if wav is not None:
            wav.close()
            secs = frames_written / params["sample_rate"]
            print(f"\nWrote {frames_written} frames ({secs:.1f}s) to {args.output}")
            if gaps:
                print(f"WARNING: {gaps} packet-sequence gaps (network loss)")


if __name__ == "__main__":
    main()
