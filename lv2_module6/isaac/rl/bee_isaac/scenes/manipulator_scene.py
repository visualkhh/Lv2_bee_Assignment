"""Manipulator demo stage built in code (no scene_sim.usda): ground, one start box, two standing pucks, robot. Layout: constants.py."""

from pxr import Gf, PhysxSchema, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics, UsdShade

from bee_isaac.constants import BOX_X, SOURCE_DIR, USD_DIR


VIEW_CAMERA = "/World/Camera"


def view_quat(x, y, z):
    """Kit Property window Orient (deg) -> quaternion: the window shows it as Rx * Ry * Rz."""
    axes = (Gf.Vec3d.XAxis(), Gf.Vec3d.YAxis(), Gf.Vec3d.ZAxis())
    return (Gf.Rotation(axes[2], z) * Gf.Rotation(axes[1], y) * Gf.Rotation(axes[0], x)).GetQuat()  # Gf multiplies row vectors


def place(stage, path, asset, x, y=0.0, z=0.0, rot=(0.0, 0.0, 0.0)):
    prim = stage.DefinePrim(path)
    prim.GetPayloads().AddPayload(str(USD_DIR / asset))
    xf = UsdGeom.Xformable(prim)
    xf.ClearXformOpOrder()
    xf.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(x, y, z))
    xf.AddRotateXYZOp().Set(Gf.Vec3f(*rot))
    return prim


def build(stage, pucks, box=(BOX_X, 0.0, 0.0), trial=None):
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    stage.SetDefaultPrim(UsdGeom.Xform.Define(stage, "/World").GetPrim())

    # CPU PhysX — 로봇 하나 + 퍽 몇 개에는 GPU 동역학의 동기화 비용이 더 큼 (isaac_startup.py)
    scene = PhysxSchema.PhysxSceneAPI.Apply(UsdPhysics.Scene.Define(stage, "/PhysicsScene").GetPrim())
    scene.CreateEnableGPUDynamicsAttr().Set(False)
    scene.CreateBroadphaseTypeAttr().Set("MBP")

    dome = UsdLux.DomeLight.Define(stage, "/World/DomeLight")  # scene_sim과 같은 조명 (인지 조건)
    dome.CreateIntensityAttr(1000.0)
    dome.CreateExposureAttr(1.25)
    dome.CreateTextureFileAttr(str(SOURCE_DIR / "HDRI" / "class_room_03_4k.exr"))
    dome.CreateTextureFormatAttr("latlong")

    floor = UsdGeom.Mesh.Define(stage, "/World/Ground")
    floor.CreatePointsAttr([(-25, -25, 0), (25, -25, 0), (25, 25, 0), (-25, 25, 0)])
    floor.CreateFaceVertexCountsAttr([4])
    floor.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    floor.CreateDisplayColorAttr([(0.45, 0.45, 0.45)])
    plane = UsdGeom.Plane.Define(stage, "/World/Ground/CollisionPlane")
    plane.CreateAxisAttr("Z")
    plane.CreatePurposeAttr("guide")
    UsdPhysics.CollisionAPI.Apply(plane.GetPrim())
    material = UsdShade.Material.Define(stage, "/World/Ground/PhysicsMaterial")
    friction = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
    friction.CreateStaticFrictionAttr(0.85)  # scene_sim floor_PhysicsMaterial
    friction.CreateDynamicFrictionAttr(0.7)
    friction.CreateRestitutionAttr(0.02)
    UsdShade.MaterialBindingAPI.Apply(plane.GetPrim()).Bind(material, UsdShade.Tokens.weakerThanDescendants, "physics")

    camera = UsdGeom.Camera.Define(stage, VIEW_CAMERA)  # 보기용 — 상자 쪽 비스듬히 위
    camera.CreateFocalLengthAttr(18.147562)
    camera.CreateClippingRangeAttr(Gf.Vec2f(0.01, 1e7))
    camera.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(0.4434, 0.70614, 0.42829))
    camera.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(view_quat(-72.259, 24.084, 172.562))

    robot = place(stage, "/World/beebot", "beebot/beebot.usda", 0.0)
    place(stage, "/World/start_box", "shelf/shelf.usda", box[0], y=box[1], rot=(0.0, 0.0, box[2]))
    with Usd.EditContext(stage, stage.GetSessionLayer()):
        robot.GetVariantSet("Physics").SetVariantSelection("physx")
    put_pucks(stage, pucks, trial)


def put_pucks(stage, pucks, trial=None):
    """(Re)create the pucks: remove the old ones, place the new ones, AprilTag texture, and for a random trial mass · friction."""
    for old in [c for c in stage.GetPrimAtPath("/World").GetChildren() if c.GetName().startswith("puck_")]:
        stage.RemovePrim(old.GetPath())
    for k, p in enumerate(pucks):
        place(stage, f"/World/{p.name}", p.asset, p.pos[0], y=p.pos[1], z=p.pos[2], rot=p.rot)
        texture = stage.GetPrimAtPath(f"/World/{p.name}/{p.shape}/Looks/mtlximage5")  # 윗면 AprilTag 번호 = 텍스처 (scene_sim과 같은 override)
        if not texture:
            raise RuntimeError(f"{p.name}: 텍스처 prim 없음")
        texture.GetAttribute("inputs:file").Set(Sdf.AssetPath(str(SOURCE_DIR / "texture" / "puck" / f"puck_{p.shape}_basecolor_id{p.tag}.png")))
        if trial:
            mass = stage.GetPrimAtPath(f"/World/{p.name}").GetAttribute("physics:mass")
            mass.Set(mass.Get() * trial.mass[k])
            pla = stage.GetPrimAtPath(f"/World/{p.name}/PhysicsMaterials/PLA")
            for attr in ("physics:dynamicFriction", "physics:staticFriction"):
                pla.GetAttribute(attr).Set(pla.GetAttribute(attr).Get() * trial.friction[k])


def move(stage, path, x, y=0.0, z=0.0, rot=(0.0, 0.0, 0.0)):
    """Set the translate / rotate ops that place() made."""
    ops = UsdGeom.Xformable(stage.GetPrimAtPath(path)).GetOrderedXformOps()  # 실행 중엔 물리가 op를 바꿔 놓을 수 있어 종류로 찾음
    by = {o.GetOpType(): o for o in ops}
    by[UsdGeom.XformOp.TypeTranslate].Set(Gf.Vec3d(x, y, z))
    if UsdGeom.XformOp.TypeRotateXYZ in by:
        by[UsdGeom.XformOp.TypeRotateXYZ].Set(Gf.Vec3f(*rot))
    elif UsdGeom.XformOp.TypeOrient in by:  # 회전이 쿼터니언 op로 바뀐 경우
        q = (Gf.Rotation(Gf.Vec3d.ZAxis(), rot[2]) * Gf.Rotation(Gf.Vec3d.YAxis(), rot[1]) * Gf.Rotation(Gf.Vec3d.XAxis(), rot[0])).GetQuat()
        op = by[UsdGeom.XformOp.TypeOrient]
        op.Set(Gf.Quatf(q) if op.GetPrecision() == UsdGeom.XformOp.PrecisionFloat else q)
    else:
        raise RuntimeError(f"{path}: 회전 op 없음 {[o.GetOpName() for o in ops]}")