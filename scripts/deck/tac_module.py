from deck.deck_module import DeckModule
from deck.deck_module import Coordinate
from deck.deck_module import ModuleParam

class TacModule(DeckModule):

    def __init__(self, name, coord):

            super(TacModule,self).__init__( name, coord)
            self.add_parameter(ModuleParam(name="temperature", p_in=True,p_out=True, vmax=40, vmin=4))
            self.add_parameter(ModuleParam(name="density", p_in=False, p_out=True))
            self.add_parameter(ModuleParam(name="spin", p_in=True, p_out=False, vmax=100, vmin=0))
            print "Tac Module Initialized"
