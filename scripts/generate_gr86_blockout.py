"""Generate an editable, approximate GR86-style blockout in Blender.

Run with:
  blender --background --python scripts/generate_gr86_blockout.py -- --output assets/gr86_blockout.blend

This is a procedural first-pass blockout, not a production-accurate replica. Dimensions
are approximate and should be refined against verified references before final modeling.
Coordinate convention: X = vehicle length (front is -X), Y = width, Z = up.
"""
from __future__ import annotations

import argparse
import math
import os
import sys

import bpy
from mathutils import Vector

# Approximate second-generation GR86 overall dimensions in metres.
LENGTH = 4.265
WIDTH = 1.775
HEIGHT = 1.310
WHEELBASE = 2.575
WHEEL_RADIUS = 0.337
WHEEL_WIDTH = 0.225


def cli_output_path() -> str:
    args = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", default="assets/gr86_blockout.blend")
    return os.path.abspath(parser.parse_args(args).output)


def material(name: str, color, metallic=0.0, roughness=0.35):
    mat = bpy.data.materials.new(name)
    mat.diffuse_color = (*color, 1.0)
    mat.use_nodes = True
    bsdf = mat.node_tree.nodes.get("Principled BSDF")
    bsdf.inputs["Base Color"].default_value = (*color, 1.0)
    bsdf.inputs["Metallic"].default_value = metallic
    bsdf.inputs["Roughness"].default_value = roughness
    return mat


def mesh_object(name, verts, faces, mat):
    mesh = bpy.data.meshes.new(name + "Mesh")
    mesh.from_pydata(verts, [], faces)
    mesh.materials.append(mat)
    mesh.update()
    obj = bpy.data.objects.new(name, mesh)
    bpy.context.collection.objects.link(obj)
    return obj


def loft_body(mat):
    # Cross-sections from nose to tail; each ring traces the lower edge, sides,
    # shoulders, windscreen/roof line, opposite shoulder and underside.
    # This intentionally prioritizes silhouette over production topology.
    stations = [
        (-2.132, .58, .49, .76),
        (-1.88, .72, .70, .88),
        (-1.42, .80, .78, .93),
        (-.90, .78, .80, .96),
        (-.38, .69, .82, 1.02),
        (.30, .69, .82, 1.02),
        (.88, .78, .80, .97),
        (1.45, .80, .76, .91),
        (1.90, .74, .70, .86),
        (2.132, .60, .48, .76),
    ]
    # Each section is ordered around the Y-Z perimeter. Cabin/roof is a separate mesh.
    profile = [
        (-.78, .43), (-.82, .69), (-.72, .88), (-.46, .94),
        (.46, .94), (.72, .88), (.82, .69), (.78, .43),
    ]
    verts = []
    for x, lower_w, shoulder_w, belt_z in stations:
        for y_factor, z in profile:
            width_factor = lower_w if z < .60 else shoulder_w
            verts.append((x, y_factor * width_factor, min(z, belt_z)))
    faces = []
    n = len(profile)
    for i in range(len(stations) - 1):
        for j in range(n):
            a = i * n + j
            b = i * n + (j + 1) % n
            faces.append((a, b, b + n, a + n))
    faces.extend([tuple(reversed(range(n))), tuple((len(stations)-1)*n + j for j in range(n))])
    obj = mesh_object("Body_Blockout", verts, faces, mat)
    bevel = obj.modifiers.new("Soft blockout edges", "BEVEL")
    bevel.width = .025
    bevel.segments = 2
    obj.modifiers.new("Weighted normals", "WEIGHTED_NORMAL")
    return obj


def loft_cabin(mat):
    # Trapezoidal coupe greenhouse, separate so it can be replaced with refined panels.
    rings = [(-.72, .65, .91), (-.22, .61, 1.295), (.49, .60, 1.295), (1.06, .67, .91)]
    verts = []
    for x, half_w, z_top in rings:
        verts.extend([(x, -half_w, .91), (x, half_w, .91),
                      (x, half_w * .78, z_top), (x, -half_w * .78, z_top)])
    faces = []
    for i in range(len(rings)-1):
        for j in range(4):
            a, b = i*4+j, i*4+(j+1)%4
            faces.append((a, b, b+4, a+4))
    faces.extend([(0, 1, 2, 3), tuple(range(12, 16))])
    return mesh_object("Cabin_Blockout", verts, faces, mat)


def add_box(name, loc, scale, mat, bevel=0.0):
    bpy.ops.mesh.primitive_cube_add(size=1, location=loc)
    obj = bpy.context.object
    obj.name = name
    obj.dimensions = scale
    bpy.ops.object.transform_apply(location=False, rotation=False, scale=True)
    obj.data.materials.append(mat)
    if bevel:
        mod = obj.modifiers.new("Edge bevel", "BEVEL")
        mod.width = bevel
        mod.segments = 2
        obj.modifiers.new("Weighted normals", "WEIGHTED_NORMAL")
    return obj


def add_wheel(x, y, side, rubber, alloy, brake):
    z = WHEEL_RADIUS
    bpy.ops.mesh.primitive_torus_add(major_radius=WHEEL_RADIUS*.72,
                                     minor_radius=WHEEL_RADIUS*.28,
                                     major_segments=32, minor_segments=12,
                                     location=(x, y, z),
                                     rotation=(math.pi/2, 0, 0))
    tire = bpy.context.object
    tire.name = f"Tire_{'L' if side < 0 else 'R'}_{'Front' if x < 0 else 'Rear'}"
    tire.data.materials.append(rubber)

    # Hub and brake cylinders have their local Z axis rotated onto vehicle Y.
    for name, radius, depth, mat, offset in [
        ("Brake", WHEEL_RADIUS*.48, .035, brake, 0.0),
        ("Rim", WHEEL_RADIUS*.64, .045, alloy, side*(WHEEL_WIDTH*.10)),
        ("Hub", WHEEL_RADIUS*.13, .055, alloy, side*(WHEEL_WIDTH*.18)),
    ]:
        bpy.ops.mesh.primitive_cylinder_add(vertices=32, radius=radius, depth=depth,
                                             location=(x, y+offset, z),
                                             rotation=(math.pi/2, 0, 0))
        obj = bpy.context.object
        obj.name = f"{name}_{'L' if side < 0 else 'R'}_{'Front' if x < 0 else 'Rear'}"
        obj.data.materials.append(mat)
        for p in obj.data.polygons:
            p.use_smooth = True

    # Simple five-spoke hint; replace with modeled wheel design in detail pass.
    for i in range(5):
        angle = 2*math.pi*i/5
        bpy.ops.mesh.primitive_cube_add(size=1, location=(x, y+side*.027, z))
        spoke = bpy.context.object
        spoke.name = f"Spoke_{'L' if side < 0 else 'R'}_{'Front' if x < 0 else 'Rear'}_{i+1}"
        spoke.dimensions = (WHEEL_RADIUS*.075, .035, WHEEL_RADIUS*.92)
        spoke.rotation_euler[1] = angle
        bpy.ops.object.transform_apply(location=False, rotation=False, scale=True)
        spoke.data.materials.append(alloy)


def build(output_path):
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete(use_global=False)
    for datablocks in (bpy.data.materials, bpy.data.meshes, bpy.data.curves):
        for block in list(datablocks):
            if block.users == 0:
                datablocks.remove(block)

    paint = material("Paint_Pearl_White_Blockout", (.82, .84, .86), .55, .24)
    glass = material("Glass_Dark_Blockout", (.025, .045, .060), .25, .16)
    rubber = material("Rubber_Blockout", (.018, .020, .023), .0, .82)
    alloy = material("Wheel_Alloy_Graphite", (.12, .14, .16), .72, .27)
    brake = material("Brake_Rotor", (.20, .20, .19), .6, .4)
    lamp = material("Lamp_Lens", (.88, .88, .78), .15, .15)
    red = material("Tail_Lamp", (.55, .018, .012), .2, .2)

    loft_body(paint)
    loft_cabin(glass)

    # Approximate glass planes and a roof panel create readable coupe glazing.
    add_box("Roof_Panel_Blockout", (.17, 0, 1.296), (.56, .78, .025), paint, .015)
    # Front/rear screen planes slant in X; side-window panels are an early silhouette cue.
    for side in (-1, 1):
        add_box(f"Side_Glass_{'L' if side < 0 else 'R'}", (.20, side*.626, 1.105),
                (.78, .018, .30), glass, .018)
        add_box(f"Mirror_{'L' if side < 0 else 'R'}", (-.63, side*.91, .94),
                (.19, .14, .10), paint, .035)

    # Front/rear fascia cues, lamps and grille.
    add_box("Front_Grille_Blockout", (-2.105, 0, .65), (.045, .72, .21), rubber, .035)
    for side in (-1, 1):
        add_box(f"Headlamp_{side}", (-2.09, side*.52, .79), (.07, .25, .10), lamp, .03)
        add_box(f"TailLamp_{side}", (2.08, side*.52, .80), (.06, .24, .105), red, .025)

    front_x, rear_x = -WHEELBASE/2, WHEELBASE/2
    for x in (front_x, rear_x):
        for side in (-1, 1):
            add_wheel(x, side*(WIDTH/2-.025), side, rubber, alloy, brake)

    # Approximate collision proxy and wheelbase markers, hidden from render.
    col = add_box("UCX_Body_Simple", (0, 0, .57), (3.55, 1.48, .72), rubber)
    col.display_type = "WIRE"
    col.hide_render = True
    col.hide_viewport = True

    scene = bpy.context.scene
    scene.unit_settings.system = "METRIC"
    scene.unit_settings.scale_length = 1.0
    scene.render.engine = "BLENDER_EEVEE_NEXT" if bpy.app.version >= (4, 2, 0) else "BLENDER_EEVEE"
    scene.render.resolution_x = 1280
    scene.render.resolution_y = 720
    scene.render.resolution_percentage = 100

    # Organize generated parts for follow-up editing.
    collection = bpy.data.collections.new("GR86_Blockout")
    scene.collection.children.link(collection)
    for obj in list(scene.objects):
        if obj.name != "Camera" and obj.name != "Light":
            for old in list(obj.users_collection):
                old.objects.unlink(obj)
            collection.objects.link(obj)

    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    bpy.ops.wm.save_as_mainfile(filepath=output_path)
    print(f"Saved GR86 blockout: {output_path}")


if __name__ == "__main__":
    build(cli_output_path())
