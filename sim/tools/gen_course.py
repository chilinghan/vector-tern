#!/usr/bin/env python3
"""Procedurally generate an obstacle-course world for Gazebo Harmonic + PX4.

Each seed gives a different layout (hoops, interior walls, pillars, lighting)
so SLAM and planning can be scored on held-out courses the stack never saw.
Alongside the .sdf, a .json sidecar records ground-truth hoop poses for
evaluating hoop detection and mapping.

The world contains the systems PX4 needs in standalone mode and spawns the
drone as model `vt_drone`. Its <world name> is always `course`, which
sim.launch.py passes to PX4 as PX4_GZ_WORLD.

Usage:
    python3 sim/tools/gen_textures.py            # once
    python3 sim/tools/gen_course.py --seed 0     # -> sim/worlds/generated/course_0.sdf
"""
import argparse
import json
import math
import random
from pathlib import Path

SIM_DIR = Path(__file__).resolve().parents[1]
DEFAULT_TEXTURES = SIM_DIR / "models" / "vt_textures" / "materials" / "textures"
DEFAULT_OUT_DIR = SIM_DIR / "worlds" / "generated"

WALL_HEIGHT = 3.0
WALL_THICKNESS = 0.1
HOOP_COLORS = [(1.0, 0.4, 0.0), (0.9, 0.1, 0.1), (0.1, 0.3, 0.9), (0.1, 0.8, 0.2), (0.9, 0.9, 0.1)]
START = (0.0, 0.0)


def textured_material(texture_path):
    return (
        "<material><diffuse>1 1 1 1</diffuse><specular>0.1 0.1 0.1 1</specular>"
        f"<pbr><metal><albedo_map>{texture_path}</albedo_map><roughness>0.9</roughness><metalness>0</metalness></metal></pbr>"
        "</material>"
    )


def color_material(rgb):
    r, g, b = rgb
    return f"<material><ambient>{r} {g} {b} 1</ambient><diffuse>{r} {g} {b} 1</diffuse><specular>0.2 0.2 0.2 1</specular></material>"


def box_model(name, pose, size, material):
    x, y, z, roll, pitch, yaw = pose
    sx, sy, sz = size
    geom = f"<geometry><box><size>{sx:.3f} {sy:.3f} {sz:.3f}</size></box></geometry>"
    return f"""
    <model name="{name}">
      <static>true</static>
      <pose>{x:.3f} {y:.3f} {z:.3f} {roll:.4f} {pitch:.4f} {yaw:.4f}</pose>
      <link name="link">
        <collision name="collision">{geom}</collision>
        <visual name="visual">{geom}{material}</visual>
      </link>
    </model>"""


def cylinder_model(name, x, y, radius, height, material):
    geom = f"<geometry><cylinder><radius>{radius:.3f}</radius><length>{height:.3f}</length></cylinder></geometry>"
    return f"""
    <model name="{name}">
      <static>true</static>
      <pose>{x:.3f} {y:.3f} {height / 2:.3f} 0 0 0</pose>
      <link name="link">
        <collision name="collision">{geom}</collision>
        <visual name="visual">{geom}{material}</visual>
      </link>
    </model>"""


def hoop_model(name, x, y, z, yaw, radius, tube, rgb, segments=24):
    """Ring of boxes in the model's local YZ plane; the fly-through normal is local +x."""
    seg_len = 2 * math.pi * radius / segments * 1.08
    links = []
    for i in range(segments):
        theta = 2 * math.pi * i / segments
        geom = f"<geometry><box><size>{tube:.3f} {tube:.3f} {seg_len:.3f}</size></box></geometry>"
        links.append(
            f"""
        <collision name="c{i}"><pose>0 {radius * math.cos(theta):.4f} {radius * math.sin(theta):.4f} {theta:.4f} 0 0</pose>{geom}</collision>
        <visual name="v{i}"><pose>0 {radius * math.cos(theta):.4f} {radius * math.sin(theta):.4f} {theta:.4f} 0 0</pose>{geom}{color_material(rgb)}</visual>"""
        )
    post_len = z - radius
    post = ""
    if post_len > 0.05:
        post_geom = f"<geometry><cylinder><radius>0.02</radius><length>{post_len:.3f}</length></cylinder></geometry>"
        post_pose = f"<pose>0 0 {-(radius + post_len / 2):.3f} 0 0 0</pose>"
        post = (
            f'<collision name="post_c">{post_pose}{post_geom}</collision>'
            f'<visual name="post_v">{post_pose}{post_geom}{color_material((0.3, 0.3, 0.3))}</visual>'
        )
    return f"""
    <model name="{name}">
      <static>true</static>
      <pose>{x:.3f} {y:.3f} {z:.3f} 0 0 {yaw:.4f}</pose>
      <link name="link">{''.join(links)}
        {post}
      </link>
    </model>"""


def world_header(rng, light_scale):
    sun_dir = (rng.uniform(-0.6, 0.6), rng.uniform(-0.6, 0.6), -1.0)
    d = rng.uniform(0.5, 1.0) * light_scale
    a = rng.uniform(0.25, 0.5) * light_scale
    return f"""<?xml version="1.0"?>
<sdf version="1.9">
  <world name="course">
    <physics type="ode">
      <max_step_size>0.004</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>250</real_time_update_rate>
    </physics>
    <gravity>0 0 -9.8</gravity>
    <magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field>
    <atmosphere type="adiabatic"/>

    <!-- Systems required by PX4 SITL (mirrors PX4's default.sdf) -->
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-contact-system" name="gz::sim::systems::Contact"/>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
    <plugin filename="gz-sim-air-pressure-system" name="gz::sim::systems::AirPressure"/>
    <plugin filename="gz-sim-apply-link-wrench-system" name="gz::sim::systems::ApplyLinkWrench"/>
    <plugin filename="gz-sim-navsat-system" name="gz::sim::systems::NavSat"/>
    <plugin filename="gz-sim-magnetometer-system" name="gz::sim::systems::Magnetometer"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>

    <scene>
      <grid>false</grid>
      <ambient>{a:.2f} {a:.2f} {a:.2f} 1</ambient>
      <background>0.6 0.6 0.6 1</background>
      <shadows>true</shadows>
    </scene>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>{d:.2f} {d:.2f} {d:.2f} 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>{sun_dir[0]:.2f} {sun_dir[1]:.2f} {sun_dir[2]:.2f}</direction>
    </light>

    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <world_frame_orientation>ENU</world_frame_orientation>
      <latitude_deg>47.397971057728974</latitude_deg>
      <longitude_deg>8.546163739800146</longitude_deg>
      <elevation>0</elevation>
    </spherical_coordinates>
"""


def far_enough(x, y, placed, min_dist):
    if math.hypot(x - START[0], y - START[1]) < 1.5:
        return False
    return all(math.hypot(x - px, y - py) >= min_dist for px, py in placed)


def sample_position(rng, half_l, half_w, margin, placed, min_dist, tries=200):
    for _ in range(tries):
        x = rng.uniform(-half_l + margin, half_l - margin)
        y = rng.uniform(-half_w + margin, half_w - margin)
        if far_enough(x, y, placed, min_dist):
            return x, y
    return None


def generate(args):
    rng = random.Random(args.seed)
    tex_dir = args.textures.resolve()
    textured = sorted(tex_dir.glob("textured_*.png"))
    if not textured or not (tex_dir / "floor.png").exists():
        raise SystemExit(f"no textures in {tex_dir}; run sim/tools/gen_textures.py first")

    def wall_tex():
        if rng.random() < args.bare_fraction:
            return textured_material(tex_dir / "bare.png")
        return textured_material(rng.choice(textured))

    length, width = args.room
    half_l, half_w = length / 2, width / 2
    parts = [world_header(rng, args.light_scale)]
    meta = {"seed": args.seed, "room": [length, width], "hoops": [], "obstacles": [], "walls": []}

    # Floor (thin box so it can carry a texture) + perimeter walls.
    parts.append(box_model("floor", (0, 0, -0.05, 0, 0, 0), (length + 1, width + 1, 0.1),
                           textured_material(tex_dir / "floor.png")))
    perimeter = [
        ("wall_n", (0, half_w, WALL_HEIGHT / 2, 0, 0, 0), (length, WALL_THICKNESS, WALL_HEIGHT)),
        ("wall_s", (0, -half_w, WALL_HEIGHT / 2, 0, 0, 0), (length, WALL_THICKNESS, WALL_HEIGHT)),
        ("wall_e", (half_l, 0, WALL_HEIGHT / 2, 0, 0, 0), (WALL_THICKNESS, width, WALL_HEIGHT)),
        ("wall_w", (-half_l, 0, WALL_HEIGHT / 2, 0, 0, 0), (WALL_THICKNESS, width, WALL_HEIGHT)),
    ]
    for name, pose, size in perimeter:
        parts.append(box_model(name, pose, size, wall_tex()))

    placed = []

    # Interior wall segments.
    for i in range(args.walls):
        pos = sample_position(rng, half_l, half_w, 2.0, placed, 2.5)
        if pos is None:
            break
        seg_len = rng.uniform(2.0, 4.0)
        yaw = rng.choice([0.0, math.pi / 2]) + rng.uniform(-0.2, 0.2)
        parts.append(box_model(f"wall_{i}", (pos[0], pos[1], WALL_HEIGHT / 2, 0, 0, yaw),
                               (seg_len, WALL_THICKNESS, WALL_HEIGHT), wall_tex()))
        placed.append(pos)
        meta["walls"].append({"x": pos[0], "y": pos[1], "yaw": yaw, "length": seg_len})

    # Hoops.
    for i in range(args.hoops):
        pos = sample_position(rng, half_l, half_w, 1.5, placed, 2.0)
        if pos is None:
            break
        radius = rng.uniform(*args.hoop_radius)
        z = rng.uniform(max(radius + 0.3, 1.0), 2.2)
        yaw = rng.uniform(-math.pi, math.pi)
        rgb = rng.choice(HOOP_COLORS)
        parts.append(hoop_model(f"hoop_{i}", pos[0], pos[1], z, yaw, radius, 0.04, rgb))
        placed.append(pos)
        meta["hoops"].append({"index": i, "x": pos[0], "y": pos[1], "z": z, "yaw": yaw, "radius": radius})

    # Pillar / box obstacles.
    for i in range(args.obstacles):
        pos = sample_position(rng, half_l, half_w, 1.0, placed, 1.5)
        if pos is None:
            break
        mat = textured_material(rng.choice(textured))
        if rng.random() < 0.5:
            r = rng.uniform(0.1, 0.35)
            parts.append(cylinder_model(f"obstacle_{i}", pos[0], pos[1], r, WALL_HEIGHT, mat))
            meta["obstacles"].append({"type": "cylinder", "x": pos[0], "y": pos[1], "radius": r})
        else:
            s = rng.uniform(0.4, 1.0)
            h = rng.uniform(0.5, WALL_HEIGHT)
            parts.append(box_model(f"obstacle_{i}", (pos[0], pos[1], h / 2, 0, 0, rng.uniform(0, math.pi)),
                                   (s, s, h), mat))
            meta["obstacles"].append({"type": "box", "x": pos[0], "y": pos[1], "size": s, "height": h})
        placed.append(pos)

    parts.append(f"""
    <include>
      <uri>model://vt_x500_oakd</uri>
      <name>vt_drone</name>
      <pose>{START[0]} {START[1]} 0.1 0 0 0</pose>
    </include>
  </world>
</sdf>
""")

    args.out_dir.mkdir(parents=True, exist_ok=True)
    sdf_path = args.out_dir / f"course_{args.seed}.sdf"
    sdf_path.write_text("".join(parts))
    (args.out_dir / f"course_{args.seed}.json").write_text(json.dumps(meta, indent=2))
    print(f"wrote {sdf_path} ({len(meta['hoops'])} hoops, {len(meta['walls'])} walls, "
          f"{len(meta['obstacles'])} obstacles)")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--room", type=float, nargs=2, default=[16.0, 10.0], metavar=("LENGTH", "WIDTH"))
    ap.add_argument("--hoops", type=int, default=5)
    ap.add_argument("--hoop-radius", type=float, nargs=2, default=[0.4, 0.6], metavar=("MIN", "MAX"))
    ap.add_argument("--walls", type=int, default=2, help="interior wall segments")
    ap.add_argument("--obstacles", type=int, default=6)
    ap.add_argument("--bare-fraction", type=float, default=0.1, help="chance a wall is featureless")
    ap.add_argument("--light-scale", type=float, default=1.0, help="<1 for dim-lighting variants")
    ap.add_argument("--textures", type=Path, default=DEFAULT_TEXTURES)
    ap.add_argument("--out-dir", type=Path, default=DEFAULT_OUT_DIR)
    generate(ap.parse_args())


if __name__ == "__main__":
    main()
