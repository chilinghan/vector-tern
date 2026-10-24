#!/usr/bin/env python3
"""Generate feature-rich procedural textures for the sim course.

Gazebo's default flat materials have no visual features, which makes any
VSLAM look broken. These textures give multi-scale corners and edges, similar
to posters, plywood grain, tape marks and clutter in a real venue.

Usage:
    python3 sim/tools/gen_textures.py [--count 8] [--size 1024] [--seed 0]
"""
import argparse
import random
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFilter

DEFAULT_OUT = Path(__file__).resolve().parents[1] / "models" / "vt_textures" / "materials" / "textures"


def _rand_color(rng, base=None, spread=90):
    if base is None:
        return tuple(rng.randint(0, 255) for _ in range(3))
    return tuple(max(0, min(255, c + rng.randint(-spread, spread))) for c in base)


def make_textured(size, rng):
    base = _rand_color(rng)
    img = Image.new("RGB", (size, size), base)
    draw = ImageDraw.Draw(img)

    # Multi-scale shapes: large patches down to small marks.
    for scale, count in ((size // 4, 12), (size // 12, 60), (size // 40, 400)):
        for _ in range(count):
            x, y = rng.randint(0, size), rng.randint(0, size)
            w, h = rng.randint(scale // 3, scale), rng.randint(scale // 3, scale)
            color = _rand_color(rng, base, spread=120)
            shape = rng.random()
            if shape < 0.45:
                draw.rectangle([x, y, x + w, y + h], fill=color)
            elif shape < 0.8:
                draw.ellipse([x, y, x + w, y + h], fill=color)
            else:
                draw.line([x, y, x + w, y + h], fill=color, width=max(1, scale // 10))

    img = img.filter(ImageFilter.GaussianBlur(radius=0.6))
    arr = np.asarray(img).astype(np.float32)
    arr += np.random.default_rng(rng.randint(0, 2**31)).normal(0, 6, arr.shape)
    return Image.fromarray(np.clip(arr, 0, 255).astype(np.uint8))


def make_bare(size, rng):
    """Near-uniform surface for deliberately testing VSLAM degradation."""
    base = _rand_color(rng, (200, 200, 200), spread=30)
    arr = np.full((size, size, 3), base, dtype=np.float32)
    arr += np.random.default_rng(rng.randint(0, 2**31)).normal(0, 2, arr.shape)
    return Image.fromarray(np.clip(arr, 0, 255).astype(np.uint8))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--count", type=int, default=8, help="number of textured surfaces")
    ap.add_argument("--size", type=int, default=1024)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--out", type=Path, default=DEFAULT_OUT)
    args = ap.parse_args()

    rng = random.Random(args.seed)
    args.out.mkdir(parents=True, exist_ok=True)
    for i in range(args.count):
        make_textured(args.size, rng).save(args.out / f"textured_{i}.png")
    make_textured(args.size * 2, rng).save(args.out / "floor.png")
    make_bare(256, rng).save(args.out / "bare.png")
    print(f"wrote {args.count + 2} textures to {args.out}")


if __name__ == "__main__":
    main()
