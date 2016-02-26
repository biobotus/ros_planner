import unittest

from deck.deck_module import DeckModule, Coordinate, ModuleParam


class TestDeckModuleMethods(unittest.TestCase):

    def test_constructor(self):
        m = DeckModule("test", Coordinate(1, 2, 3))
        self.assertEqual(m.coor.coor_x, 1)
        self.assertEqual(m.coor.coor_y, 2)
        self.assertEqual(m.coor.coor_z, 3)
        self.assertEqual(m.name, "test")
        
    def test_str(self):
        m = DeckModule("test", Coordinate(1, 2, 3))
        self.assertEqual(str(m), "test")
        
    def test_add_parameter(self):
        m = DeckModule("test", Coordinate(1, 2, 3))
        p = ModuleParam("testParam", True, True, 6, 4)
        m.add_parameter(p)
        self.assertEqual(len(m.params), 1)
        self.assertEqual(m.params[0].name, "testParam")
        self.assertTrue(m.params[0].p_in)
        self.assertTrue(m.params[0].p_out)
        
    def test_set_well_layout(self):
        m = DeckModule("test", Coordinate(10, 10, 0))
        m.set_well_layout(8, 12, Coordinate(2, 2, 0), Coordinate(5, 3, 0))
        self.assertEqual(m.nb_line, 8)
        self.assertEqual(m.nb_column, 12)
        self.assertEqual(m.well1_offset, Coordinate(2, 2, 0))
        self.assertEqual(m.well_offset, Coordinate(5, 3, 0))
        
        

if __name__ == '__main__':
    unittest.main()
