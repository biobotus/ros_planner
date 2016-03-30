from deck.deck_module import DeckModule
from deck.deck_module import Coordinate


class Rect4Container(DeckModule):

    def __init__(self, name, coor):

        super(Rect4Container,self).__init__(name, coor)
        self.set_well_layout(1, 4, Coordinate(10, 15, 0), Coordinate(20, 0, 0))
        
