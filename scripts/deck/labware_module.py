#!usr/bin/python
from deck.deck_module import Coordinate, DeckModule


class Rect4Container(DeckModule):

    def __init__(self, name, coord):

        super(Rect4Container,self).__init__(name, coord)
        self.set_well_layout(4, 4, Coordinate(10, 10, 0), Coordinate(10, 10, 0))

class Trash_bin(DeckModule):

    def __init__(self, name, coord):

        super(Trash_bin, self).__init__(name, coord)
