#!/usr/bin/env python3
"""Build the AgroBot Blender scenes from the CAD export.

Run with Blender's Python (either works):
    blender --background --python blender/build_scene.py -- studio --render
    python  blender/build_scene.py field --render          # with the `bpy` pip module

Scenes
    studio     robot on a studio backdrop (hero still)
    field      robot working in a crop row (still)
    exploded   every printed part pulled apart, for the build guide
    anim       the simulated mission replayed from simulation/output/row_mission_traj.npz

Each scene is saved to blender/scenes/<name>.blend; --render also writes
blender/renders/<name>.png (or an .mp4 for `anim`).
"""
import json
import math
import os
import sys

import bpy
import bmesh
import numpy as np
from mathutils import Matrix, Quaternion, Vector

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
EXPORT = os.path.join(ROOT, "cad", "export")
SCENES = os.path.join(HERE, "scenes")
RENDERS = os.path.join(HERE, "renders")
MM = 0.001

# crop row layout - keep identical to simulation/gen_mjcf.py
ROW_Y = 0.335
PLANT_XS = [0.70, 1.30, 1.90]
FRUIT_PLANT = 2
FRUIT_R = 0.021
FRUIT_OFFSET = (0.100, -0.030, 0.290)


# ----------------------------------------------------------------------------
# helpers
# ----------------------------------------------------------------------------
def reset():
    bpy.ops.wm.read_factory_settings(use_empty=True)
    scene = bpy.context.scene
    scene.unit_settings.system = "METRIC"
    return scene


def load_manifest():
    with open(os.path.join(EXPORT, "manifest.json")) as fh:
        return json.load(fh)


def principled(name, color, rough=0.5, metallic=0.0, transmission=0.0, coat=0.0, alpha=1.0, emission=None):
    mat = bpy.data.materials.new(name)
    mat.use_nodes = True
    b = mat.node_tree.nodes.get("Principled BSDF")
    b.inputs["Base Color"].default_value = (*color, 1.0)
    b.inputs["Roughness"].default_value = rough
    b.inputs["Metallic"].default_value = metallic

    def set_input(names, value):
        for n in names:
            if n in b.inputs:
                b.inputs[n].default_value = value
                return

    set_input(["Transmission Weight", "Transmission"], transmission)
    set_input(["Coat Weight", "Clearcoat"], coat)
    if alpha < 1.0:
        b.inputs["Alpha"].default_value = alpha
    if emission:
        set_input(["Emission Color", "Emission"], (*emission[:3], 1.0))
        set_input(["Emission Strength"], emission[3])
    return mat


def srgb_to_linear(c):
    """Manifest colours are display (sRGB) values; Blender shaders want scene-linear."""
    return tuple(v / 12.92 if v <= 0.04045 else ((v + 0.055) / 1.055) ** 2.4 for v in c)


def part_material(part):
    n, c = part["name"], srgb_to_linear(part["color"])
    if n == "bottle_500ml":
        m = principled("m_" + n, (0.75, 0.90, 1.0), rough=0.04, transmission=1.0)
        m.node_tree.nodes["Principled BSDF"].inputs["IOR"].default_value = 1.33
        return m
    if n in ("gearmotor", "hex_coupler", "race_balls"):
        return principled("m_" + n, c, rough=0.32, metallic=1.0)
    if part["material"].startswith("TPU"):
        return principled("m_" + n, c, rough=0.85)
    if not part["printed"]:
        return principled("m_" + n, c, rough=0.45)
    return principled("m_" + n, c, rough=0.42, coat=0.15)      # printed PETG: slightly glossy


def import_stl(path, name):
    bpy.ops.wm.stl_import(filepath=path, global_scale=1.0)
    obj = bpy.context.selected_objects[0]
    obj.name = name
    me = obj.data
    bm = bmesh.new()
    bm.from_mesh(me)
    bmesh.ops.remove_doubles(bm, verts=bm.verts, dist=1e-3)
    bm.to_mesh(me)
    bm.free()
    me.transform(Matrix.Diagonal((MM, MM, MM, 1.0)))      # bake millimetres -> metres into the mesh
    obj.matrix_world = Matrix.Identity(4)
    for p in me.polygons:
        p.use_smooth = True
    if hasattr(me, "set_sharp_from_angle"):
        me.set_sharp_from_angle(angle=math.radians(32))
    return obj


def link_matrices(man, pose):
    """World matrix of every link frame for the given joint angles (degrees)."""
    out = {}

    def solve(name):
        if name in out:
            return out[name]
        L = man["links"][name]
        if L["parent"] is None:
            out[name] = Matrix.Identity(4)
            return out[name]
        w, x, y, z = L["quat"]
        m = solve(L["parent"]) @ Matrix.Translation(Vector(L["pos"]) * MM) @ Quaternion((w, x, y, z)).to_matrix().to_4x4()
        j = "grip" if L["joint"] == "grip_mimic" else L["joint"]
        q = pose.get(j, 0.0)
        if q:
            m = m @ Matrix.Rotation(math.radians(q), 4, Vector(L["axis"]))
        out[name] = m
        return m

    for n in man["links"]:
        solve(n)
    return out


def build_robot(man, pose=None, printed_only=False, collection_name="AgroBot"):
    """Import every mesh.  Returns ({link: empty}, [(obj, part, link)])."""
    pose = pose if pose is not None else man["pose_default"]
    coll = bpy.data.collections.new(collection_name)
    bpy.context.scene.collection.children.link(coll)
    mats = link_matrices(man, pose)
    empties = {}
    for name in man["links"]:
        e = bpy.data.objects.new("link_" + name, None)
        e.empty_display_size = 0.03
        e.matrix_world = mats[name]
        coll.objects.link(e)
        empties[name] = e
    objs = []
    for part in man["parts"]:
        if printed_only and not part["printed"]:
            continue
        mat = part_material(part)
        base = None
        for i, pl in enumerate(part["placements"]):
            nm = part["name"] if len(part["placements"]) == 1 else f'{part["name"]}.{i + 1}'
            if base is None:
                obj = import_stl(os.path.join(EXPORT, part["mesh"]), nm)
                for c in obj.users_collection:
                    c.objects.unlink(obj)
                obj.data.materials.append(mat)
                base = obj
            else:
                obj = bpy.data.objects.new(nm, base.data)      # instances share the mesh
            coll.objects.link(obj)
            w, x, y, z = pl["quat"]
            local = Matrix.Translation(Vector(pl["pos"]) * MM) @ Quaternion((w, x, y, z)).to_matrix().to_4x4()
            obj.parent = empties[pl["link"]]
            obj.matrix_parent_inverse = Matrix.Identity(4)
            obj.matrix_basis = local
            objs.append((obj, part, pl["link"]))
    return empties, objs


def add_camera(scene, loc, target, lens=50):
    cam = bpy.data.cameras.new("Camera")
    cam.lens = lens
    obj = bpy.data.objects.new("Camera", cam)
    scene.collection.objects.link(obj)
    obj.location = loc
    tgt = bpy.data.objects.new("CameraTarget", None)
    scene.collection.objects.link(tgt)
    tgt.location = target
    c = obj.constraints.new("TRACK_TO")
    c.target = tgt
    c.track_axis = "TRACK_NEGATIVE_Z"
    c.up_axis = "UP_Y"
    scene.camera = obj
    return obj, tgt


def area_light(scene, name, loc, target, power, size, color=(1, 1, 1)):
    L = bpy.data.lights.new(name, "AREA")
    L.energy = power
    L.size = size
    L.color = color
    obj = bpy.data.objects.new(name, L)
    scene.collection.objects.link(obj)
    obj.location = loc
    d = Vector(target) - Vector(loc)
    obj.rotation_euler = d.to_track_quat("-Z", "Y").to_euler()
    return obj


def setup_cycles(scene, samples=64, res=(1920, 1080)):
    scene.render.engine = "CYCLES"
    scene.cycles.device = "CPU"
    scene.cycles.samples = samples
    scene.cycles.use_denoising = True
    scene.cycles.use_adaptive_sampling = True
    scene.cycles.adaptive_threshold = 0.03
    scene.cycles.max_bounces = 6
    scene.cycles.transparent_max_bounces = 8
    scene.cycles.transmission_bounces = 6
    scene.render.resolution_x, scene.render.resolution_y = res
    scene.render.resolution_percentage = 100
    scene.render.film_transparent = False
    try:
        scene.view_settings.view_transform = "AgX"
        scene.view_settings.look = "AgX - Medium High Contrast"
    except Exception:      # noqa: BLE001 - older Blender: keep Filmic
        pass


def world_color(scene, color, strength=1.0):
    w = bpy.data.worlds.new("World")
    w.use_nodes = True
    bg = w.node_tree.nodes["Background"]
    bg.inputs[0].default_value = (*color, 1.0)
    bg.inputs[1].default_value = strength
    scene.world = w
    return w


def world_sky(scene, sun_elev=38.0, sun_rot=140.0, strength=0.11):
    """Physical sky if this Blender has one, otherwise a plain blue world + sun lamp."""
    w = bpy.data.worlds.new("Sky")
    w.use_nodes = True
    nt = w.node_tree
    bg = nt.nodes["Background"]
    ok = False
    try:
        sky = nt.nodes.new("ShaderNodeTexSky")
        types = [i.identifier for i in sky.bl_rna.properties["sky_type"].enum_items]
        for t in ("MULTIPLE_SCATTERING", "NISHITA", "SINGLE_SCATTERING", "HOSEK_WILKIE"):
            if t in types:
                sky.sky_type = t
                break
        if hasattr(sky, "sun_elevation"):
            sky.sun_elevation = math.radians(sun_elev)
            sky.sun_rotation = math.radians(sun_rot)
        nt.links.new(sky.outputs[0], bg.inputs[0])
        bg.inputs[1].default_value = strength
        ok = sky.sky_type in ("MULTIPLE_SCATTERING", "NISHITA", "SINGLE_SCATTERING")
    except Exception as e:      # noqa: BLE001
        print("sky texture unavailable:", e)
    if not ok:
        bg.inputs[0].default_value = (0.45, 0.65, 0.95, 1.0)
        bg.inputs[1].default_value = 1.0
        sun = bpy.data.lights.new("Sun", "SUN")
        sun.energy = 4.0
        sun.angle = math.radians(1.5)
        o = bpy.data.objects.new("Sun", sun)
        scene.collection.objects.link(o)
        o.rotation_euler = (math.radians(90 - sun_elev), 0, math.radians(sun_rot + 180))
    scene.world = w


def save_and_render(scene, name, render, animation=False):
    os.makedirs(SCENES, exist_ok=True)
    os.makedirs(RENDERS, exist_ok=True)
    blend = os.path.join(SCENES, name + ".blend")
    bpy.ops.wm.save_as_mainfile(filepath=blend, compress=True, relative_remap=False)
    print("saved", blend)
    if not render:
        return
    if animation:
        frames = os.path.join(RENDERS, name + "_frames")
        os.makedirs(frames, exist_ok=True)
        scene.render.image_settings.file_format = "PNG"
        scene.render.filepath = os.path.join(frames, "f_")
        bpy.ops.render.render(animation=True)
        import shutil
        import subprocess
        if shutil.which("ffmpeg"):
            mp4 = os.path.join(RENDERS, name + ".mp4")
            subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-framerate", str(scene.render.fps), "-i",
                            os.path.join(frames, "f_%04d.png"), "-c:v", "libx264", "-crf", "20", "-pix_fmt", "yuv420p",
                            "-movflags", "+faststart", mp4], check=True)
            print("encoded", mp4)
    else:
        scene.render.image_settings.file_format = "PNG"
        scene.render.filepath = os.path.join(RENDERS, name + ".png")
        bpy.ops.render.render(write_still=True)
        print("rendered", scene.render.filepath)


# ----------------------------------------------------------------------------
# environment pieces
# ----------------------------------------------------------------------------
def studio_backdrop(scene):
    """Seamless cyclorama: floor sweeping up into a back wall."""
    prof = [(-3.0, 0.0), (1.2, 0.0)]
    r, n = 1.2, 16
    for i in range(1, n + 1):
        a = math.pi / 2 * i / n
        prof.append((1.2 + r * math.sin(a), r - r * math.cos(a)))
    prof.append((2.4, 4.0))
    verts, faces = [], []
    for (x, z) in prof:
        verts += [(x, -4.0, z), (x, 4.0, z)]
    for i in range(len(prof) - 1):
        faces.append((2 * i, 2 * i + 1, 2 * i + 3, 2 * i + 2))
    me = bpy.data.meshes.new("Backdrop")
    me.from_pydata(verts, [], faces)
    for p in me.polygons:
        p.use_smooth = True
    obj = bpy.data.objects.new("Backdrop", me)
    scene.collection.objects.link(obj)
    obj.rotation_euler = (0, 0, math.radians(128))       # wall behind the robot as seen from the camera
    obj.data.materials.append(principled("backdrop", (0.55, 0.58, 0.57), rough=0.7))
    return obj


def soil_material():
    mat = bpy.data.materials.new("Soil")
    mat.use_nodes = True
    nt = mat.node_tree
    b = nt.nodes["Principled BSDF"]
    b.inputs["Roughness"].default_value = 0.95
    coord = nt.nodes.new("ShaderNodeTexCoord")
    noise = nt.nodes.new("ShaderNodeTexNoise")
    noise.inputs["Scale"].default_value = 18.0
    noise.inputs["Detail"].default_value = 12.0
    noise.inputs["Roughness"].default_value = 0.7
    fine = nt.nodes.new("ShaderNodeTexNoise")
    fine.inputs["Scale"].default_value = 260.0
    fine.inputs["Detail"].default_value = 6.0
    ramp = nt.nodes.new("ShaderNodeValToRGB")
    ramp.color_ramp.elements[0].position = 0.30
    ramp.color_ramp.elements[0].color = (0.035, 0.020, 0.011, 1)
    ramp.color_ramp.elements[1].position = 0.75
    ramp.color_ramp.elements[1].color = (0.150, 0.085, 0.045, 1)
    mix = nt.nodes.new("ShaderNodeMath")
    mix.operation = "ADD"
    scale = nt.nodes.new("ShaderNodeMath")
    scale.operation = "MULTIPLY"
    scale.inputs[1].default_value = 0.35
    bump = nt.nodes.new("ShaderNodeBump")
    bump.inputs["Strength"].default_value = 0.9
    bump.inputs["Distance"].default_value = 0.02
    nt.links.new(coord.outputs["Object"], noise.inputs["Vector"])
    nt.links.new(coord.outputs["Object"], fine.inputs["Vector"])
    nt.links.new(noise.outputs[0], ramp.inputs[0])
    nt.links.new(ramp.outputs[0], b.inputs["Base Color"])
    nt.links.new(fine.outputs[0], scale.inputs[0])
    nt.links.new(noise.outputs[0], mix.inputs[0])
    nt.links.new(scale.outputs[0], mix.inputs[1])
    nt.links.new(mix.outputs[0], bump.inputs["Height"])
    nt.links.new(bump.outputs[0], b.inputs["Normal"])
    return mat


def field_ground(scene, size=14.0, n=220, flat_band=0.34):
    """Tilled soil: raised beds under the crop rows, flat wheel tracks in between.
    The strip the robot drives on is exactly z = 0 so it matches the simulation."""
    rng = np.random.default_rng(4)
    xs = np.linspace(-size / 2 + 1.5, size / 2 + 1.5, n)
    ys = np.linspace(-size / 2, size / 2, n)
    verts, faces = [], []
    row_ys = [ROW_Y, ROW_Y + 0.75, ROW_Y + 1.5, ROW_Y + 2.25, -0.62, -0.62 - 0.75, -0.62 - 1.5, -0.62 - 2.25]
    for y in ys:
        for x in xs:
            bed = 0.0
            for ry in row_ys:
                bed = max(bed, 0.030 * math.exp(-((y - ry) / 0.11) ** 2))
            rough = 0.006 * math.sin(x * 23.0 + y * 7.0) * math.sin(y * 31.0) + rng.normal(0, 0.0025)
            w = min(1.0, max(0.0, (abs(y) - flat_band * 0.55) / 0.08))     # 0 on the robot's lane
            verts.append((x, y, bed * w + rough * w - 0.0005))
    for j in range(n - 1):
        for i in range(n - 1):
            a = j * n + i
            faces.append((a, a + 1, a + n + 1, a + n))
    me = bpy.data.meshes.new("Field")
    me.from_pydata(verts, [], faces)
    for p in me.polygons:
        p.use_smooth = True
    obj = bpy.data.objects.new("Field", me)
    scene.collection.objects.link(obj)
    obj.data.materials.append(soil_material())
    return obj


_leaf_mesh = None


def leaf_mesh():
    """One leaf: a pointed oval with a centre fold and a droop, base at the origin."""
    global _leaf_mesh
    if _leaf_mesh:
        return _leaf_mesh
    nu, nv = 9, 5
    verts, faces = [], []
    for i in range(nu):
        u = i / (nu - 1)
        half_w = 0.30 * math.sin(math.pi * u ** 0.75) * (1 - 0.25 * u)
        for j in range(nv):
            v = j / (nv - 1) * 2 - 1
            x = u
            y = v * half_w
            z = -0.28 * u * u + 0.10 * abs(v) * half_w * 2.0
            verts.append((x, y, z))
    for i in range(nu - 1):
        for j in range(nv - 1):
            a = i * nv + j
            faces.append((a, a + nv, a + nv + 1, a + 1))
    me = bpy.data.meshes.new("Leaf")
    me.from_pydata(verts, [], faces)
    for p in me.polygons:
        p.use_smooth = True
    _leaf_mesh = me
    return me


_plant_mats = {}


def plant_materials():
    if not _plant_mats:
        _plant_mats["stem"] = principled("stem", (0.05, 0.13, 0.025), rough=0.6)
        _plant_mats["leaf0"] = principled("leaf0", (0.020, 0.115, 0.018), rough=0.5)
        _plant_mats["leaf1"] = principled("leaf1", (0.035, 0.160, 0.025), rough=0.5)
        _plant_mats["fruit"] = principled("fruit", (0.62, 0.035, 0.02), rough=0.18, coat=0.5)
        _plant_mats["green"] = principled("fruit_green", (0.25, 0.38, 0.06), rough=0.25)
    return _plant_mats


def add_cylinder(coll, name, p0, p1, radius, mat, verts=10):
    p0, p1 = Vector(p0), Vector(p1)
    d = p1 - p0
    bm = bmesh.new()
    bmesh.ops.create_cone(bm, cap_ends=True, segments=verts, radius1=radius, radius2=radius * 0.8, depth=d.length)
    me = bpy.data.meshes.new(name)
    bm.to_mesh(me)
    bm.free()
    for p in me.polygons:
        p.use_smooth = True
    obj = bpy.data.objects.new(name, me)
    coll.objects.link(obj)
    obj.location = (p0 + p1) / 2
    obj.rotation_euler = d.to_track_quat("Z", "Y").to_euler()
    obj.data.materials.append(mat)
    return obj


def add_sphere(coll, name, loc, r, mat, segs=20):
    bm = bmesh.new()
    bmesh.ops.create_uvsphere(bm, u_segments=segs, v_segments=segs // 2, radius=r)
    me = bpy.data.meshes.new(name)
    bm.to_mesh(me)
    bm.free()
    for p in me.polygons:
        p.use_smooth = True
    obj = bpy.data.objects.new(name, me)
    coll.objects.link(obj)
    obj.location = loc
    obj.data.materials.append(mat)
    return obj


def add_plant(coll, name, x, y, h=0.36, seed=0, fruit_at=None, extra_fruit=True, clear_dir=None):
    rng = np.random.default_rng(seed)
    m = plant_materials()
    add_cylinder(coll, name + "_stem", (x, y, 0), (x, y, h), 0.006, m["stem"])
    n_leaf = 11
    for i in range(n_leaf):
        z = h * (0.22 + 0.78 * i / (n_leaf - 1))
        a = i * 2.39996 + rng.uniform(-0.3, 0.3)
        length = 0.17 * (1.0 - 0.45 * i / n_leaf) * rng.uniform(0.85, 1.1)
        # keep foliage out of the corridor the gripper uses
        if fruit_at is not None:
            fa = math.atan2(fruit_at[1], fruit_at[0])
            if abs((a - fa + math.pi) % (2 * math.pi) - math.pi) < 1.0 and abs(z - fruit_at[2]) < 0.14:
                a += math.pi
        # treated plants: no low leaves on the side the arm works from
        if clear_dir is not None and z < 0.27 and abs((a - clear_dir + math.pi) % (2 * math.pi) - math.pi) < 1.3:
            a += math.pi
        leaf = bpy.data.objects.new(f"{name}_leaf{i}", leaf_mesh())
        coll.objects.link(leaf)
        leaf.location = (x, y, z)
        leaf.scale = (length, length, length)
        leaf.rotation_euler = (rng.uniform(-0.3, 0.3), rng.uniform(-0.35, 0.05), a)
        if not leaf.data.materials:
            leaf.data.materials.append(m["leaf0"])
        leaf.material_slots[0].link = "OBJECT"
        leaf.material_slots[0].material = m["leaf0"] if i % 2 else m["leaf1"]
    if fruit_at is not None:
        fx, fy, fz = fruit_at
        add_cylinder(coll, name + "_truss", (x, y, fz - 0.05), (x + fx, y + fy, fz - 0.034), 0.004, m["stem"])
        add_cylinder(coll, name + "_cup", (x + fx, y + fy, fz - 0.034), (x + fx, y + fy, fz - 0.026), 0.011, m["stem"])
    elif extra_fruit:
        # a few unripe / ripe fruit on the far side for colour
        for k in range(int(rng.integers(1, 4))):
            a = rng.uniform(0.3, 2.8)
            r = rng.uniform(0.03, 0.06)
            z = h * rng.uniform(0.35, 0.7)
            ripe = rng.random() < 0.4
            add_sphere(coll, f"{name}_f{k}", (x + r * math.cos(a), y + r * math.sin(a), z), rng.uniform(0.014, 0.02),
                       m["fruit"] if ripe else m["green"], segs=14)


def crop_rows(scene, with_fruit_body=True, x_range=(-1.1, 4.4)):
    coll = bpy.data.collections.new("Crop")
    scene.collection.children.link(coll)
    k = 0
    for i, x in enumerate(PLANT_XS):
        add_plant(coll, f"plant{i}", x, ROW_Y, h=0.36, seed=10 + i, fruit_at=FRUIT_OFFSET if i == FRUIT_PLANT else None,
                  extra_fruit=False, clear_dir=-math.pi / 2)
    for row_y in (ROW_Y, ROW_Y + 0.75, ROW_Y + 1.5, ROW_Y + 2.25, -0.62, -0.62 - 0.75, -0.62 - 1.5, -0.62 - 2.25):
        for x in np.arange(x_range[0], x_range[1], 0.6):
            if row_y == ROW_Y and any(abs(x + 0.1 - px) < 0.05 for px in PLANT_XS):
                continue
            k += 1
            add_plant(coll, f"bg{k}", float(x) + 0.1, row_y, h=0.30 + 0.08 * ((k * 7) % 5) / 5, seed=100 + k)
    fruit = None
    if with_fruit_body:
        fx = PLANT_XS[FRUIT_PLANT] + FRUIT_OFFSET[0]
        fruit = add_sphere(coll, "fruit", (fx, ROW_Y + FRUIT_OFFSET[1], FRUIT_OFFSET[2]), FRUIT_R, plant_materials()["fruit"], segs=28)
    return coll, fruit


# ----------------------------------------------------------------------------
# scenes
# ----------------------------------------------------------------------------
def scene_studio(render):
    scene = reset()
    man = load_manifest()
    build_robot(man)
    studio_backdrop(scene)
    world_color(scene, (0.9, 0.92, 0.95), 0.12)
    area_light(scene, "Key", (1.9, -0.6, 2.0), (0.1, 0, 0.2), 150, 1.4, (1.0, 0.96, 0.9))
    area_light(scene, "Fill", (-0.6, -2.2, 1.0), (0.1, 0, 0.2), 45, 2.2, (0.9, 0.95, 1.0))
    area_light(scene, "Rim", (-1.6, 1.4, 1.8), (0.1, 0, 0.25), 110, 1.0)
    add_camera(scene, (0.98, -1.12, 0.62), (0.10, 0.0, 0.20), lens=58)
    setup_cycles(scene, samples=72, res=(1600, 900))
    save_and_render(scene, "agrobot_studio", render)


def scene_field(render):
    scene = reset()
    man = load_manifest()
    # robot stopped at the second plant with the probe in the soil
    pose = dict(yaw=97.4, shoulder=13.7, elbow=-88.2, wrist=74.5, grip=10.0)
    empties, _ = build_robot(man, pose)
    root = empties["chassis"]
    move = Matrix.Translation((PLANT_XS[1] - 0.09, 0, 0))
    for e in empties.values():
        e.matrix_world = move @ e.matrix_world
    field_ground(scene)
    crop_rows(scene)
    world_sky(scene)
    add_camera(scene, (2.12, -0.10, 0.56), (1.24, 0.19, 0.14), lens=33)
    setup_cycles(scene, samples=72, res=(1600, 900))
    scene.view_settings.exposure = -0.5
    save_and_render(scene, "agrobot_field", render)


def scene_exploded(render):
    scene = reset()
    man = load_manifest()
    pose = dict(yaw=0, shoulder=0, elbow=0, wrist=0, grip=0)
    empties, objs = build_robot(man, pose, printed_only=True)
    # push parts apart: along Z by height in the stack, wheels outwards, arm forwards
    lift = dict(chassis_tray=0.0, electronics_plate=0.07, battery_clamp=0.10, top_deck=0.20, race_ring=0.26,
                turntable_clip=0.30, harvest_basket=0.30, camera_mast=0.30, camera_holder=0.36, bottle_holder=0.30,
                pump_saddle=0.27, front_bumper=0.0)
    for obj, part, link in objs:
        n = part["name"]
        mw = obj.matrix_world.copy()
        d = Vector((0, 0, 0))
        if link == "chassis":
            d.z = lift.get(n, 0.2)
            if n == "front_bumper":
                d.x = 0.10
        elif link.startswith("wheel"):
            side = 1 if mw.translation.y > 0 else -1
            d.y = side * (0.10 if n == "wheel_rim" else 0.20)
        else:
            order = dict(turntable=0.34, upper_arm=0.42, forearm=0.50, tool=0.56, finger_drive=0.60, finger_idle=0.60)
            d.z = order[link]
            d.x = dict(turntable=0.0, upper_arm=0.02, forearm=0.06, tool=0.10, finger_drive=0.16, finger_idle=0.16)[link]
            if n == "upper_arm_drive_plate":
                d.y = 0.06
            if n == "upper_arm_idler_plate":
                d.y = -0.06
            if n == "probe_clip":
                d.y = -0.06
            if n == "nozzle_clip":
                d.y = 0.06
            if n == "gripper_palm":
                d.x += 0.04
            if n == "finger_pad":
                d.x += 0.05
        obj.parent = None
        obj.matrix_world = Matrix.Translation(d) @ mw
    world_color(scene, (1, 1, 1), 0.35)
    floor = bpy.data.meshes.new("Floor")
    floor.from_pydata([(-6, -6, 0), (6, -6, 0), (6, 6, 0), (-6, 6, 0)], [], [(0, 1, 2, 3)])
    fo = bpy.data.objects.new("Floor", floor)
    scene.collection.objects.link(fo)
    fo.location.z = -0.002
    fo.data.materials.append(principled("floor", (0.93, 0.93, 0.93), rough=0.7))
    area_light(scene, "Key", (-1.5, -2.0, 3.0), (0.2, 0, 0.4), 260, 2.5)
    area_light(scene, "Fill", (2.0, -1.0, 1.5), (0.2, 0, 0.4), 90, 2.5)
    add_camera(scene, (1.45, -1.75, 1.25), (0.18, 0.0, 0.36), lens=50)
    setup_cycles(scene, samples=64, res=(1600, 900))
    save_and_render(scene, "agrobot_exploded", render)


def scene_anim(render, speed=4.0, fps=24, res=(960, 540), samples=24, t0=None, t1=None):
    """Replay the simulated row mission (time-lapse)."""
    traj_path = os.path.join(ROOT, "simulation", "output", "row_mission_traj.npz")
    tr = np.load(traj_path, allow_pickle=False)
    t, poses, bodies, pump = tr["t"], tr["poses"], [str(b) for b in tr["bodies"]], tr["pump"]
    nb = len(bodies)
    pos = poses[:, :3 * nb].reshape(-1, nb, 3)
    quat = poses[:, 3 * nb:].reshape(-1, nb, 4)

    scene = reset()
    man = load_manifest()
    empties, _ = build_robot(man, dict())
    field_ground(scene)
    _, fruit = crop_rows(scene)
    world_sky(scene)

    # spray cone under the nozzle
    bm = bmesh.new()
    bmesh.ops.create_cone(bm, cap_ends=False, segments=24, radius1=0.004, radius2=0.05, depth=0.30)
    me = bpy.data.meshes.new("Spray")
    bm.to_mesh(me)
    bm.free()
    spray = bpy.data.objects.new("Spray", me)
    scene.collection.objects.link(spray)
    spray.parent = empties["tool"]
    tcp = man["tcp"]["nozzle"]
    spray.location = (tcp[0] * MM, tcp[1] * MM, tcp[2] * MM - 0.15)
    spray.rotation_euler = (math.pi, 0, 0)
    sm = principled("spray", (0.55, 0.80, 1.0), rough=0.3, alpha=0.10, emission=(0.5, 0.8, 1.0, 1.5))
    try:
        sm.blend_method = "BLEND"
    except Exception:       # noqa: BLE001
        pass
    spray.data.materials.append(sm)

    sel = np.arange(len(t))
    if t0 is not None:
        sel = sel[(t[sel] >= t0)]
    if t1 is not None:
        sel = sel[(t[sel] <= t1)]
    t_start = t[sel[0]]
    n_frames = int((t[sel[-1]] - t_start) / speed * fps)
    scene.render.fps = fps
    scene.frame_start, scene.frame_end = 1, n_frames
    animated = {name: empties[name] for name in empties}
    idx = {b: i for i, b in enumerate(bodies)}
    for e in animated.values():
        e.rotation_mode = "QUATERNION"
    if fruit:
        fruit.rotation_mode = "QUATERNION"
    cam, tgt = add_camera(scene, (0, 0, 1), (0, 0, 0.2), lens=35)
    # camera path in the robot frame: (sim time, offset from chassis, look-at offset)
    cam_keys = [(0, (-1.2, -0.9, 0.80), (0.25, 0.10, 0.15)), (12, (1.25, -0.45, 0.65), (0.1, 0.2, 0.16)),
                (30, (1.30, -0.20, 0.70), (0.1, 0.2, 0.16)), (48, (1.20, -0.50, 0.65), (0.1, 0.2, 0.18)),
                (64, (1.25, -0.30, 0.70), (0.12, 0.2, 0.22)), (80, (1.10, -0.60, 0.80), (0.0, 0.1, 0.25)),
                (99, (1.40, -0.90, 0.75), (0.0, 0.0, 0.2))]

    def cam_at(time):
        for a, b in zip(cam_keys, cam_keys[1:]):
            if a[0] <= time <= b[0]:
                s = (time - a[0]) / (b[0] - a[0])
                s = s * s * (3 - 2 * s)
                return (Vector(a[1]).lerp(Vector(b[1]), s), Vector(a[2]).lerp(Vector(b[2]), s))
        k = cam_keys[0] if time < cam_keys[0][0] else cam_keys[-1]
        return Vector(k[1]), Vector(k[2])

    prev_q = {}
    for f in range(1, n_frames + 1):
        time = t_start + (f - 1) * speed / fps
        k = int(np.searchsorted(t, time))
        k = min(max(k, 0), len(t) - 1)
        for name, e in animated.items():
            i = idx[name]
            q = Quaternion(quat[k, i].tolist())
            if name in prev_q and prev_q[name].dot(q) < 0:
                q.negate()
            prev_q[name] = q
            e.location = pos[k, i].tolist()
            e.rotation_quaternion = q
            e.keyframe_insert("location", frame=f)
            e.keyframe_insert("rotation_quaternion", frame=f)
        if fruit:
            i = idx["fruit"]
            fruit.location = pos[k, i].tolist()
            fruit.rotation_quaternion = Quaternion(quat[k, i].tolist())
            fruit.keyframe_insert("location", frame=f)
            fruit.keyframe_insert("rotation_quaternion", frame=f)
        on = bool(pump[k])
        spray.hide_render = not on
        spray.hide_viewport = not on
        spray.keyframe_insert("hide_render", frame=f)
        spray.keyframe_insert("hide_viewport", frame=f)
        c = Vector(pos[k, idx["chassis"]].tolist())
        off, look = cam_at(time)
        cam.location = Vector((c.x, 0, 0)) + off
        tgt.location = Vector((c.x, 0, 0)) + look
        cam.keyframe_insert("location", frame=f)
        tgt.keyframe_insert("location", frame=f)
    setup_cycles(scene, samples=samples, res=res)
    scene.cycles.adaptive_threshold = 0.06
    scene.render.use_persistent_data = True
    scene.view_settings.exposure = -0.5
    save_and_render(scene, "agrobot_mission_anim", render, animation=True)


def main():
    argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else sys.argv[1:]
    render = "--render" in argv
    names = [a for a in argv if not a.startswith("--")] or ["studio"]
    opts = {}
    for a in argv:
        if a.startswith("--") and "=" in a:
            k, v = a[2:].split("=", 1)
            opts[k] = float(v)
    for n in names:
        if n == "studio":
            scene_studio(render)
        elif n == "field":
            scene_field(render)
        elif n == "exploded":
            scene_exploded(render)
        elif n == "anim":
            kw = {}
            if "speed" in opts:
                kw["speed"] = opts["speed"]
            if "samples" in opts:
                kw["samples"] = int(opts["samples"])
            if "t0" in opts:
                kw["t0"] = opts["t0"]
            if "t1" in opts:
                kw["t1"] = opts["t1"]
            if "fps" in opts:
                kw["fps"] = int(opts["fps"])
            if "resx" in opts and "resy" in opts:
                kw["res"] = (int(opts["resx"]), int(opts["resy"]))
            scene_anim(render, **kw)
        else:
            raise SystemExit(f"unknown scene '{n}'")


if __name__ == "__main__":
    main()
