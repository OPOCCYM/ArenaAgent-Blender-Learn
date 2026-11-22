#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
qa.py — техническая проверка геометрии сцены car_realistic.blend

    blender -b -noaudio car_realistic.blend -P qa.py

Проверяет каждый меш на:
  * дубли вершин (remove_doubles, dist = 5e-5) — проверяется на отдельной копии,
    чтобы не искажать счётчики исходного меша;
  * вырожденные грани (площадь < 1e-12 или меньше 3 вершин);
  * неманифолдные рёбра;
  * n-gon (грани с более чем 4 вершинами);
  * «висячие» вершины/рёбра без граней (loose) и пустые меши;
  * для объектов коллекции Car: наличие UV-канала и отсутствие точек «.» в имени
    (защита от коллизий вроде Wheel_1.001).

Ожидаемый результат: все счётчики ошибок — нули.
При любом ненулевом счётчике скрипт завершается с кодом 1.
"""
import os
import sys

import bpy
import bmesh

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE and HERE not in sys.path:
    sys.path.insert(0, HERE)

from car_geometry import CAR_COLLECTION  # noqa: E402

DIST = 5e-5


def audit(ob, is_car_mesh=False):
    if ob.type != 'MESH':
        return None
    me = ob.data
    bm = bmesh.new()
    bm.from_mesh(me)

    nv = len(bm.verts)
    nf = len(bm.faces)
    empty = 1 if nf == 0 else 0
    deg = sum(1 for f in bm.faces if len(f.verts) < 3 or f.calc_area() < 1e-12)
    nonman = sum(1 for e in bm.edges if not e.is_manifold)
    ngon = sum(1 for f in bm.faces if len(f.verts) > 4)
    loose = (
        sum(1 for v in bm.verts if not v.link_faces)
        + sum(1 for e in bm.edges if not e.link_faces)
    )
    tris = sum(max(0, len(f.verts) - 2) for f in bm.faces)

    # Проверяем дубли после снятия метрик исходного меша
    bmesh.ops.remove_doubles(bm, verts=bm.verts, dist=DIST)
    dups = nv - len(bm.verts)
    bm.free()

    no_uv = 1 if (is_car_mesh and not me.uv_layers) else 0
    dot_name = 1 if (is_car_mesh and '.' in ob.name) else 0

    return {
        'name': ob.name,
        'verts': nv,
        'faces': nf,
        'tris': tris,
        'dups': dups,
        'deg': deg,
        'nonman': nonman,
        'ngon': ngon,
        'loose': loose,
        'empty': empty,
        'no_uv': no_uv,
        'dot_name': dot_name,
    }


def run_qa():
    """Выполняет аудит сцены и возвращает (ok, totals, rows)."""
    car_col = bpy.data.collections.get(CAR_COLLECTION)
    car_names = {ob.name for ob in car_col.objects} if car_col is not None else set()

    totals = {
        'verts': 0, 'faces': 0, 'tris': 0,
        'dups': 0, 'deg': 0, 'nonman': 0, 'ngon': 0,
        'loose': 0, 'empty': 0, 'no_uv': 0, 'dot_name': 0,
    }
    rows = []
    n_obj = 0
    for ob in bpy.data.objects:
        if ob.type != 'MESH':
            continue
        n_obj += 1
        is_car = ob.name in car_names if car_names else (ob.name != 'Cyclorama')
        r = audit(ob, is_car_mesh=is_car)
        rows.append(r)
        for k in totals:
            totals[k] += r[k]

    error_keys = ('dups', 'deg', 'nonman', 'ngon', 'loose', 'empty', 'no_uv', 'dot_name')
    bad = sum(totals[k] for k in error_keys)
    print(
        f'qa.py: {n_obj} объектов, '
        f'{totals["verts"]:,} вершин, {totals["faces"]:,} грани, '
        f'{totals["tris"]:,} трис')
    print(
        f'       дублей {totals["dups"]}, '
        f'вырожденных граней {totals["deg"]}, '
        f'неманифолдных рёбер {totals["nonman"]}, '
        f'n-gon {totals["ngon"]}, '
        f'loose {totals["loose"]}, '
        f'пустых {totals["empty"]}, '
        f'без UV {totals["no_uv"]}, '
        f'точек в имени {totals["dot_name"]}')
    if n_obj == 0 or bad:
        print('GEOMETRY QA: FAIL')
        for r in rows:
            if any(r[k] for k in error_keys):
                print(
                    f"  {r['name']}: dups={r['dups']} deg={r['deg']} "
                    f"nonman={r['nonman']} ngon={r['ngon']} loose={r['loose']} "
                    f"empty={r['empty']} no_uv={r['no_uv']} dot_name={r['dot_name']}"
                )
        return False, totals, rows
    print('GEOMETRY QA: OK (все счётчики нулевые)')
    return True, totals, rows


def main():
    ok, _totals, _rows = run_qa()
    if not ok:
        sys.exit(1)


if __name__ == '__main__':
    main()
