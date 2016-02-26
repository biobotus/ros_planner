__author__ = 'Do'

from deck.deck_module import DeckModule
from deck.deck_module import Coordinate


class RectContainerLabware(DeckModule):

    def __init__(self, name, coor=Coordinate(0, 0, 0)):
        super(RectContainerLabware, self).__init__(name, coor)
        # Ce contenant imaginaire a un seul puit et on va pipette au milieu a 5 mm 5 mm.
        self.set_well_layout(1, 1, Coordinate(100, 100, 0), Coordinate(0, 0, 0))
