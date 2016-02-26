import unittest
from math import pi
from deck.deck_module import Coordinate


class TestCoordinateMethods(unittest.TestCase):
    def test_constructor(self):
        c = Coordinate(1, 2, 3)
        self.assertEqual(c.coor_x, 1)
        self.assertEqual(c.coor_y, 2)
        self.assertEqual(c.coor_z, 3)

    def test_rotate_z(self):
        c = Coordinate(2, 0, 0)
        c.rotate_z(pi / 2, Coordinate(1, 0, 0))
        self.assertEqual(c.coor_x, 1)
        self.assertEqual(c.coor_y, 1)
        self.assertEqual(c.coor_z, 0)

        c.rotate_z(-pi / 2, Coordinate(1, 0, 0))
        self.assertEqual(c.coor_x, 2)
        self.assertEqual(c.coor_y, 0)
        self.assertEqual(c.coor_z, 0)

        c.rotate_z(-pi / 2, Coordinate(1, 0, 0))
        self.assertEqual(c.coor_x, 1)
        self.assertEqual(c.coor_y, -1)
        self.assertEqual(c.coor_z, 0)

    def test_translate_x(self):
        c = Coordinate(2, 0, 0)
        c.translate_x(3)
        self.assertEqual(c.coor_y, 0)
        self.assertEqual(c.coor_z, 0)
        self.assertEqual(c.coor_x, 5)

        c.translate_x(-1)
        self.assertEqual(c.coor_x, 4)
        self.assertEqual(c.coor_y, 0)
        self.assertEqual(c.coor_z, 0)

    def test_translate_y(self):
        c = Coordinate(2, 0, 0)
        c.translate_y(3)
        self.assertEqual(c.coor_y, 3)
        self.assertEqual(c.coor_x, 2)
        self.assertEqual(c.coor_z, 0)

        c.translate_y(-1)
        self.assertEqual(c.coor_y, 2)
        self.assertEqual(c.coor_x, 2)
        self.assertEqual(c.coor_z, 0)


if __name__ == '__main__':
    unittest.main()
