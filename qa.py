#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
qa.py — техническая проверка геометрии сцены car_realistic.blend

    blender -b -noaudio car_realistic.blend -P qa.py

Проверяет каждый меш на:
  * дубли вершин (remove_doubles, dist = 5e-5),
  * вырожденные грани (площадь < 1e-12 или меньше 3 вершин),
  * неманифолдные рёбра,
  * n-gon (грани с более чем 4 вершинами).

Ожидаемый результат (эталон из PROJECT_MEMORY.md): все счётчики — нули.
При любом ненулеом значении скрипт завершается с кодом 1.
"""
import sys

import bpy
import bmesh

DIST = 5e-5


def audit(ob):
    if ob.type != 'MESH':
        return None
    me = ob.data
    bm = bmesh.new()
    bm.from_mesh(me)
    nv0 = len(bm.verts)
    bmesh.ops.remove_doubles(bm, verts=bm.verts, dist=DIST)
    dups = nv0 - len(bm.verts)
    deg = sum(1 for f in bm.faces
              if len(f.verts) < 3 or f.calc_area() < 1e-12)
    nonman = sum(1 for e in bm.edges if not e.is_manifold)
    ngon = sum(1 for f in bm.faces if len(f.verts) > 4)
    tris = sum(len(f.verts) - 2 for f in bm.faces)
    nf = len(bm.faces)
    bm.free()
    return {
        'name': ob.name,
        'verts': nv0 - dups,
        'faces': nf,
        'tris': tris,
        'dups': dups,
        'deg': deg,
        'nonman': nonman,
        'ngon': ngon,
    }


def main():
    totals = {'verts': 0, 'faces': 0, 'tris': 0,
              'dups': 0, 'deg': 0, 'nonman': 0, 'ngon': 0}
    rows = []
    n_obj = 0
    for ob in bpy.data.objects:
        if ob.type != 'MESH':
            continue
        n_obj += 1
        r = audit(ob)
        rows.append(r)
        for k in totals:
            totals[k] += r[k]

    bad = (totals['dups'] + totals['deg'] +
           totals['nonman'] + totals['ngon'])
    print(
        f'qa.py: {n_obj} объектов, '
        f'{totals["verts"]:,} вершин, {totals["faces"]:,} грани, '
        f'{totals["tris"]:,} трис')
    print(
        f'       дублей {totals["dups"]}, '
        f'вырожденных граней {totals["deg"]}, '
        f'неманифолдных рёбер {totals["nonman"]}, '
        f'n-gon {totals["ngon"]}')
    if bad:
        print('GEOMETRY QA: FAIL')
        for r in rows:
            if r['dups'] or r['deg'] or r['nonman'] or r['ngon']:
                print(f"  {r['name']}: dups={r['dups']} deg={r['deg']} "
                      f"nonman={r['nonman']} ngon={r['ngon']}")
        sys.exit(1)
    print('GEOMETRY QA: OK (все счётчики нулевые)')


if __name__ == '__main__':
    main()
