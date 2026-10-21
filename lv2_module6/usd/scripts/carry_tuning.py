#!/usr/bin/env python3
"""Carry hand-tuned values from the previous USD (in git) into a freshly re-imported one.

Run right after the URDF re-import overwrites ../beebot/, before committing it:

    python carry_tuning.py            # old = HEAD
    python carry_tuning.py --ref abc1 # old = another commit
    python carry_tuning.py --dry-run  # report only, write nothing

Rule: the importer's values (from URDF) win. From the old layers we copy only what the
importer did not author -- drive K/D/type, armature, maxJointVelocity, jointEnabled,
material bindings, applied API schemas, and whole hand-made prims (cameras, physics
materials, ground plane). Exceptions in OVERWRITE are copied even if the importer
authored them. Renamed prims cannot be matched and are reported as SKIP.

Needs pxr: Isaac Sim's python, or `pip install usd-core`.
"""
import argparse
import subprocess
from pathlib import Path

from pxr import Sdf, Usd

ASSET_DIR = Path(__file__).resolve().parent.parent / "beebot"
LAYERS = ["beebot.usda", "payloads/Physics/physics.usda", "payloads/Physics/physx.usda"]
# (layer, property name): hand-tuned although the importer also authors it
OVERWRITE = {
    ("beebot.usda", "height"),  # wheel collider width 0.015 (URDF says 0.0182)
    ("beebot.usda", "extent"),
}


def skip_prop(name):
    # sim leftovers (poses, joint states) must not override the new URDF geometry
    return name.startswith(("xformOp", "state:"))


def has_payload(spec):
    if isinstance(spec, Sdf.RelationshipSpec):
        return bool(spec.targetPathList.GetAddedOrExplicitItems())
    return spec.HasDefaultValue() or bool(spec.connectionPathList.GetAddedOrExplicitItems())


def merge_api_schemas(old, new, log):
    old_items = old.GetInfo("apiSchemas").GetAddedOrExplicitItems() if old.HasInfo("apiSchemas") else []
    if not new.HasInfo("apiSchemas"):
        new_op = Sdf.TokenListOp()
    else:
        new_op = new.GetInfo("apiSchemas")
    have = set(new_op.GetAddedOrExplicitItems())
    missing = [s for s in old_items if s not in have]
    if not missing:
        return
    if new_op.isExplicit:
        new_op.explicitItems = list(new_op.explicitItems) + missing
    else:
        new_op.prependedItems = list(new_op.prependedItems) + missing
    new.SetInfo("apiSchemas", new_op)
    log(f"api   {new.path} += {missing}")


def merge_prim(old_layer, new_layer, stage, path, layer_name, log):
    old = old_layer.GetPrimAtPath(path)
    new = new_layer.GetPrimAtPath(path)
    if not new:
        exists = stage.GetPrimAtPath(path).IsValid()
        parent_ok = stage.GetPrimAtPath(path.GetParentPath()).IsValid() or new_layer.GetPrimAtPath(path.GetParentPath())
        if old.specifier == Sdf.SpecifierDef and not exists and parent_ok:
            Sdf.CopySpec(old_layer, path, new_layer, path)
            log(f"prim  {path}")
            return
        if not exists:
            log(f"SKIP  {path} (not in new asset: removed or renamed?)")
            return
        new = Sdf.CreatePrimInLayer(new_layer, path)

    merge_api_schemas(old, new, log)
    for prop in old.properties:
        name = prop.name
        if skip_prop(name) or not has_payload(prop):
            continue
        cur = new.properties.get(name)
        if cur:
            if (layer_name, name) not in OVERWRITE:
                continue
            if isinstance(prop, Sdf.AttributeSpec) and cur.default == prop.default:
                continue
        Sdf.CopySpec(old_layer, prop.path, new_layer, prop.path)
        log(f"prop  {prop.path} = {getattr(prop, 'default', None)}")
    for child in old.nameChildren:
        merge_prim(old_layer, new_layer, stage, child.path, layer_name, log)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--ref", default="HEAD", help="git commit holding the tuned USD (default HEAD)")
    ap.add_argument("--dry-run", action="store_true")
    args = ap.parse_args()

    stage = Usd.Stage.Open(str(ASSET_DIR / "beebot.usda"))
    stage.SetEditTarget(stage.GetSessionLayer())  # variant pick must not be saved
    stage.GetPrimAtPath(stage.GetDefaultPrim().GetPath()).GetVariantSet("Physics").SetVariantSelection("physx")

    for name in LAYERS:
        text = subprocess.run(["git", "show", f"{args.ref}:./{name}"], cwd=ASSET_DIR,
                              check=True, capture_output=True, text=True).stdout
        old_layer = Sdf.Layer.CreateAnonymous(".usda")
        old_layer.ImportFromString(text)
        new_layer = Sdf.Layer.FindOrOpen(str(ASSET_DIR / name))
        lines = []
        for root in old_layer.rootPrims:
            merge_prim(old_layer, new_layer, stage, root.path, name, lines.append)
        print(f"== {name}: {len(lines)} change(s)")
        for line in lines:
            print("  " + line)
        if lines and not args.dry_run:
            new_layer.Save()
    if args.dry_run:
        print("dry run: nothing written")


if __name__ == "__main__":
    main()
