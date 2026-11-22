"""Structural acceptance tests for the generated OBJ blockout.
Run from repository root: python -m unittest discover -s tests -v
"""
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1]
OBJ = ROOT / "assets" / "gr86_blockout.obj"

class ObjBlockoutTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.lines = OBJ.read_text().splitlines()
        cls.vertices = [tuple(map(float, line.split()[1:])) for line in cls.lines if line.startswith("v ")]
        cls.faces = [line.split()[1:] for line in cls.lines if line.startswith("f ")]

    def test_export_files_exist(self):
        self.assertTrue(OBJ.is_file())
        self.assertTrue(OBJ.with_suffix(".mtl").is_file())

    def test_mesh_is_nontrivial(self):
        self.assertGreater(len(self.vertices), 1000)
        self.assertGreater(len(self.faces), 1000)

    def test_vehicle_object_groups_exist(self):
        text = "\n".join(self.lines)
        for token in ("Body_Blockout", "Cabin_Glazing_Blockout", "Tire_Front_-1", "Tire_Rear_1", "Fender_Arch", "Headlamp", "Front_Badge", "Grille_Slat", "Rear_Plate", "Rear_Diffuser", "TailLamp", "Exhaust_Tip"):
            self.assertIn(token, text)
        tires = [line for line in self.lines if line.startswith("o Tire_")]
        self.assertEqual(len(tires), 4)

    def test_face_indices_are_valid(self):
        for face in self.faces:
            self.assertGreaterEqual(len(face), 3)
            for token in face:
                index = int(token.split("/")[0])
                self.assertGreaterEqual(index, 1)
                self.assertLessEqual(index, len(self.vertices))

    def test_approximate_vehicle_bounds(self):
        mins = [min(v[i] for v in self.vertices) for i in range(3)]
        maxs = [max(v[i] for v in self.vertices) for i in range(3)]
        dims = [maxs[i] - mins[i] for i in range(3)]
        # Body plus grille badge at the front and correctly tucked rear exhausts.
        self.assertAlmostEqual(dims[0], 4.329, delta=.01)
        self.assertGreater(dims[1], 1.7)
        self.assertLess(dims[1], 2.1)
        self.assertAlmostEqual(dims[2], 1.3025, delta=.10)

if __name__ == "__main__":
    unittest.main()
