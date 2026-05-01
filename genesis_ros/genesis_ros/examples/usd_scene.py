"""Standalone USD scene demo.

Loads one of the bundled .usdz scenery files into a Genesis scene and opens
the viewer. Assets ship with the genesis-world-assets .deb under
/opt/genesis/assets/usd, with a fallback to the in-repo copy for dev
checkouts.

Run via either of:

* ``ros2 run genesis_ros usd_scene -a falcon9``
* ``python3 -m genesis_ros.examples.usd_scene -a train_station``

List bundled assets with ``--list``.
"""
from __future__ import annotations

import argparse
import os
import sys

import genesis as gs


# Short alias -> filename in assets/usd. Keep keys lowercase + underscored.
_USD_EXTS = (".usd", ".usda", ".usdc", ".usdz")
# .usdz scenery from Sketchfab is visual-only (no UsdPhysics schemas) —
# add_stage will report "No entities found". Listed so we can warn instead
# of trying to load them.
_VISUAL_ONLY_NAMES = {
    "abandoned_station", "train_ride", "falcon9",
    "minecraft_station", "launch_pad",
}

_ASSET_ROOTS = (
    "/opt/genesis/assets/usd",
    os.path.join(os.path.dirname(__file__), "..", "..", "..",
                 "Genesis", "genesis", "assets", "usd"),
)


def _alias_from_relpath(rel: str) -> str:
    """robots/g1.usd -> 'g1', envs/hospital.usd -> 'hospital',
    Falcon_9_-_SpaceX.usdz -> 'falcon9' (lowercased, punctuation stripped)."""
    base = os.path.splitext(os.path.basename(rel))[0]
    out = []
    for ch in base.lower():
        if ch.isalnum():
            out.append(ch)
        elif ch in "._- ":
            out.append("_")
    alias = "".join(out).strip("_")
    while "__" in alias:
        alias = alias.replace("__", "_")
    return alias


def _discover_assets() -> dict[str, str]:
    """Walk every _ASSET_ROOT and map alias -> absolute path.
    First root wins on collisions (so /opt/genesis takes priority over the
    in-repo dev copy)."""
    found: dict[str, str] = {}
    for root in _ASSET_ROOTS:
        if not os.path.isdir(root):
            continue
        for dirpath, _, files in os.walk(root):
            for f in files:
                if not f.lower().endswith(_USD_EXTS):
                    continue
                full = os.path.abspath(os.path.join(dirpath, f))
                rel = os.path.relpath(full, root)
                alias = _alias_from_relpath(rel)
                if alias in found:
                    continue  # earlier root wins
                found[alias] = full
    return found


_ASSETS = _discover_assets()


def _resolve_asset(name: str) -> str:
    if name not in _ASSETS:
        raise FileNotFoundError(
            f"USD asset '{name}' not found. Run --list to see available aliases."
        )
    return _ASSETS[name]


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("-a", "--asset", choices=sorted(_ASSETS.keys()),
                        default="cartpole",
                        help="Bundled physics-ready USD asset to load (default: cartpole).")
    parser.add_argument("--list", action="store_true",
                        help="List bundled assets and exit.")
    parser.add_argument("--scale", type=float, default=1.0)
    parser.add_argument("--pos", type=float, nargs=3, default=(0.0, 0.0, 0.0),
                        metavar=("X", "Y", "Z"))
    parser.add_argument("--euler", type=float, nargs=3, default=(0.0, 0.0, 0.0),
                        metavar=("RX", "RY", "RZ"))
    parser.add_argument("--no_plane", action="store_true",
                        help="Skip ground plane (useful for sky-box scenes).")
    parser.add_argument("--headless", action="store_true",
                        help="No viewer (useful for asset-load smoke tests).")
    parser.add_argument("-n", "--num_steps", type=int, default=0,
                        help="If >0, step the sim N times then exit. "
                             "0 keeps the viewer open until closed.")
    args = parser.parse_args(argv)

    if args.list:
        physics, visual = {}, {}
        for k, v in _ASSETS.items():
            (visual if k in _VISUAL_ONLY_NAMES else physics)[k] = v
        if physics:
            print("Physics-ready USD assets (work with add_stage):")
            width = max(len(k) for k in physics)
            for k in sorted(physics):
                print(f"  {k:<{width}}  ->  {physics[k]}")
        if visual:
            print("\nVisual-only USDZ scenes (need GLB conversion to load):")
            width = max(len(k) for k in visual)
            for k in sorted(visual):
                print(f"  {k:<{width}}  ->  {visual[k]}")
        if not _ASSETS:
            print(f"No USD assets found under: {', '.join(_ASSET_ROOTS)}")
        return 0

    asset_path = _resolve_asset(args.asset)
    print(f"[usd_scene] loading {asset_path}")

    try:
        gs.init()
    except Exception:
        pass

    show_viewer = not args.headless
    scene = gs.Scene(
        show_viewer=show_viewer,
        viewer_options=gs.options.ViewerOptions(
            camera_pos=(6.0, 4.0, 3.0),
            camera_lookat=(0.0, 0.0, 1.0),
            camera_fov=45,
        ),
    )

    if not args.no_plane:
        scene.add_entity(gs.morphs.Plane())

    scene.add_stage(
        morph=gs.morphs.USD(
            file=asset_path,
            scale=args.scale,
            pos=tuple(args.pos),
            euler=tuple(args.euler),
        ),
    )

    scene.build()

    if args.num_steps > 0:
        for _ in range(args.num_steps):
            scene.step()
    else:
        # Idle-step until viewer is closed.
        try:
            while True:
                scene.step()
        except KeyboardInterrupt:
            pass

    return 0


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main(sys.argv[1:]))
