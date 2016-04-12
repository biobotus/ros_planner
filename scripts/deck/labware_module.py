#!usr/bin/python
from deck.deck_module import Coordinate, DeckModule


class Trash_bin(DeckModule):
    """Trash bin tip collector"""
    def __init__(self, name, coord):
        super(Trash_bin, self).__init__(name, coord)

class Small_Tip_Holder(DeckModule):
    """Small tip holder """
    def __init__(self, name, coord):
        super(Small_Tip_Holder, self).__init__(name, coord)
        self.set_well_layout(8, 12, Coordinate(0, 0, 0), Coordinate(9, 9, 0))

class Medium_Tip_Holder(DeckModule):
    """Medium tip holder """
    def __init__(self, name, coord):
        super(Medium_Tip_Holder, self).__init__(name, coord)
        self.set_well_layout(8, 12, Coordinate(0, 0, 0), Coordinate(9.2, 9.2, 0))

class Large_Tip_Holder(DeckModule):
    """Large tip holder """
    def __init__(self, name, coord):
        super(Large_Tip_Holder, self).__init__(name, coord)
        self.set_well_layout(8, 12, Coordinate(0, 0, 0), Coordinate(9, 9, 0))

class Centrifuge_Vial_Holder(DeckModule):
    """Holder for centrifugial vials """
    def __init__(self, name, coord):
        super(Centrifuge_Vial_Holder, self).__init__(name, coord)
        self.set_well_layout(8, 12, Coordinate(0, 0, 0), Coordinate(9, 9, 0))

class Multiwell_Plate(DeckModule):
    """Multiwell plate"""
    def __init__(self, name, coord):
        super(Multiwell_Plate, self).__init__(name, coord)
        self.set_well_layout(8, 12, Coordinate(0, 0, 0), Coordinate(9, 9, 0))
