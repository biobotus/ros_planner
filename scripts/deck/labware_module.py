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
        self.xS = 0
        self.yS = 0
        self.max_column = 8
        self.max_row = 11

    def get_tip_pos(self, m_type):
        x = self.xS
        y = self.yS



        if m_type == "pipette_m":
            if x > 0:
                y = y+1
                self.yS=self.yS+1
            self.yS=self.yS+1
            self.xS=0
            x = 0

        else:
            self.xS = self.xS + 1
            if self.xS>self.max_column:
                self.yS=self.yS+1
                if self.yS>self.max_row:
                    return None
                else:
                    self.xS=0
        return x,y

class Medium_Tip_Holder(DeckModule): # TODO add better get tip pos 8 / 11
    """Medium tip holder """
    def __init__(self, name, coord):
        super(Medium_Tip_Holder, self).__init__(name, coord)
        self.set_well_layout(8, 12, Coordinate(0, 0, 0), Coordinate(9.2, 9.2, 0))
        self.xM = 0
        self.yM = 0
        self.max_column = 8
        self.max_row = 11


    def get_tip_pos(self, m_type):
        x = self.xM
        y = self.yM

        if m_type == "pipette_m":
            if x > 0:
                y = y+1
                self.yM=self.yM+1
            self.yM=self.yM+1
            self.xM=0
            x = 0
        else:
            self.xM = self.xM + 1
            if self.xM>self.max_column:
                self.yM=self.yM+1
                if self.yM>self.max_row:
                    return None
                else:
                    self.xM=0
        return x,y

class Large_Tip_Holder(DeckModule):
    """Large tip holder """
    def __init__(self, name, coord):
        super(Large_Tip_Holder, self).__init__(name, coord)
        self.set_well_layout(8, 12, Coordinate(0, 0, 0), Coordinate(9, 9, 0))
        self.xL = 0
        self.yL = 0
        self.max_column = 8
        self.max_row = 11

    def get_tip_pos(self, m_type):
        x = self.xL
        y = self.yL

        if m_type == "pipette_m":

            if x > 0:
                y = y+1
                self.yL=self.yL+1

            self.yL=self.yL+1
            self.xL=0
            x = 0

        else:
            self.xL = self.xL + 1
            if self.xL>self.max_column:
                self.yL=self.yL+1
                if self.yL>self.max_row:
                    self.yL=0
                else:
                    self.xL=0
        return x,y


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

class Large_Container(DeckModule):
    """Large_Container"""
    def __init__(self, name, coord):
        super(Large_Container, self).__init__(name, coord)
        self.set_well_layout(8, 1, Coordinate(0, 0, 0), Coordinate(0, 0, 0))


