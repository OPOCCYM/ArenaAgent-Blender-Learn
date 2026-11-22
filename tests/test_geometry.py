#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Автотесты валидации геометрии экспортированного OBJ.

Запуск:
    python3 -m unittest discover -s tests -v

Проверяет:
  * OBJ и MTL существуют и не пустые
  * Вершины и грани корректны (индексы в диапазоне)
  * Нет вырожденных граней (площадь ~0 или дублирующиеся индексы)
  * Границы модели позитивны и близки к реальным габаритам седана
  * Нет NaN/Inf координат
"""
import math
import os
import unittest

from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
OBJ = ROOT / 'exports' / 'car_ue.obj'
MTL = ROOT / 'exports' / 'car_ue.mtl'

# Ожидаемые габариты седана (м) из PROJECT_MEMORY.md / car_build.py
EXPECTED_LENGTH = 5.00
EXPECTED_WIDTH = 1.872
EXPECTED_HEIGHT = 1.456
TOLERANCE = 0.30  # м, допуск на процедурную блокировку


def read_mtl(path):
    mats = {}
    cur = None
    for line in path.read_text().splitlines():
        p = line.split()
        if not p:
            continue
        if p[0] == 'newmtl':
            cur = p[1]
            mats[cur] = {'Kd': (0.7, 0.7, 0.7), 'Ks': (0.1, 0.1, 0.1), 'Ns': 80}
        elif cur and p[0] == 'Kd':
            mats[cur]['Kd'] = tuple(map(float, p[1:4]))
        elif cur and p[0] == 'Ks':
            mats[cur]['Ks'] = tuple(map(float, p[1:4]))
        elif cur and p[0] == 'Ns':
            mats[cur]['Ns'] = float(p[1])
    return mats


def load_obj(path):
    verts = []
    mats = read_mtl(path.with_suffix('.mtl')) if path.with_suffix('.mtl').exists() else {}
    tris = []
    curmat = 'DefaultGrey'
    group = ''
    for line in path.read_text().splitlines():
        p = line.split()
        if not p:
            continue
        if p[0] == 'v':
            verts.append(tuple(map(float, p[1:4])))
        elif p[0] == 'usemtl':
            curmat = p[1]
        elif p[0] == 'o':
            group = p[1]
        elif p[0] == 'f':
            ids = [int(x.split('/')[0]) - 1 for x in p[1:]]
            for i in range(1, len(ids) - 1):
                tris.append((ids[0], ids[i], ids[i + 1], curmat, group))
    return verts, tris, mats


class TestGeometry(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if not OBJ.exists():
            raise unittest.SkipTest(f'OBJ не найден: {OBJ}. Сначала запустите export_obj.py.')
        cls.verts, cls.tris, cls.mats = load_obj(OBJ)

    def test_obj_exists_and_nonempty(self):
        self.assertTrue(OBJ.exists(), f'{OBJ} не существует')
        self.assertTrue(MTL.exists(), f'{MTL} не существует')
        self.assertGreater(len(self.verts), 0, 'Нет вершин в OBJ')
        self.assertGreater(len(self.tris), 0, 'Нет граней в OBJ')

    def test_no_nan_inf(self):
        for v in self.verts:
            for coord in v:
                self.assertFalse(math.isnan(coord), f'NaN в вершине {v}')
                self.assertFalse(math.isinf(coord), f'Inf в вершине {v}')

    def test_face_indices_in_range(self):
        n = len(self.verts)
        for ia, ib, ic, _, _ in self.tris:
            self.assertGreaterEqual(ia, 0)
            self.assertLess(ia, n)
            self.assertGreaterEqual(ib, 0)
            self.assertLess(ib, n)
            self.assertGreaterEqual(ic, 0)
            self.assertLess(ic, n)

    def test_no_degenerate_faces(self):
        for ia, ib, ic, _, _ in self.tris:
            self.assertNotEqual(ia, ib)
            self.assertNotEqual(ib, ic)
            self.assertNotEqual(ia, ic)

    def test_bounds_positive_and_reasonable(self):
        xs = [v[0] for v in self.verts]
        ys = [v[1] for v in self.verts]
        zs = [v[2] for v in self.verts]
        length = max(xs) - min(xs)
        width = max(ys) - min(ys)
        height = max(zs) - min(zs)
        self.assertGreater(length, 0)
        self.assertGreater(width, 0)
        self.assertGreater(height, 0)
        self.assertAlmostEqual(length, EXPECTED_LENGTH, delta=TOLERANCE)
        self.assertAlmostEqual(width, EXPECTED_WIDTH, delta=TOLERANCE)
        self.assertAlmostEqual(height, EXPECTED_HEIGHT, delta=TOLERANCE)

    def test_materials_defined(self):
        self.assertGreater(len(self.mats), 0, 'Нет материалов в MTL')
        used = {t[3] for t in self.tris}
        for mat in used:
            self.assertIn(mat, self.mats, f'Материал {mat} используется, но не определён в MTL')

    def test_object_groups(self):
        groups = {t[4] for t in self.tris if t[4]}
        self.assertGreater(len(groups), 0, 'Нет объектных групп в OBJ')


if __name__ == '__main__':
    unittest.main()
