__author__ = 'Jie Fu, jief@seas.upenn.edu'

from types import *

class ExceptionFSM(Exception):

    """This is the FSM Exception class."""

    def __init__(self, value):
        self.value = value

    def __str__(self):
        return `self.value`

class DFA:

    """This is a deterministic Finite State Automaton (NFA).
    """

    def __init__(self, initial_state=None, alphabet=None, transitions= dict([]),final_states=None, memory=None):
        self.state_transitions = {}
        self.final_states = set([])
        self.state_transitions=transitions
        if alphabet == None:
            self.alphabet=[]
        else:
            self.alphabet=alphabet
        self.initial_state = initial_state
        self.states =[ initial_state ] # the list of states in the machine.
        
    def reset (self):

        """This sets the current_state to the initial_state and sets
        input_symbol to None. The initial state was set by the constructor
         __init__(). """

        self.current_state = self.initial_state
        self.input_symbol = None

    def add_transition(self,input_symbol,state,next_state=None):
        if next_state is None:
            next_state = state
        else:
            self.state_transitions[(input_symbol, state)] = next_state
        if next_state in self.states:
            pass
        else:
            self.states.append(next_state)
        if state in self.states:
            pass
        else:
            self.states.append(state)
        
        if input_symbol in self.alphabet:
            pass
        else:
            self.alphabet.append(input_symbol)
                
                
    def get_transition (self, input_symbol, state):

        """This returns a list of next states given an input_symbol and state.
        """

        if self.state_transitions.has_key((input_symbol, state)):
            return self.state_transitions[(input_symbol, state)]
        else:
            return None

class DRA(DFA,object):
    """A child class of DFA --- determinisitic Rabin automaton
    """
    def __init__(self, initial_state=None, alphabet=None, transitions=dict([]),rabin_acc= None, memory=None):
        # The rabin_acc is a list of rabin pairs rabin_acc=[(J_i, K_i), i =0,...,N]
        # Each K_i, J_i is a set of states.
        # J_i is visited only finitely often
        # K_i has to be visited infinitely often.
        super(DRA, self).__init__(initial_state, alphabet, transitions)
        self.acc=rabin_acc
        
    def add_rabin_acc(self, rabin_acc):
        self.acc=rabin_acc





if __name__=='__main__':    
    dra=DRA(0,['W','C','H','E'])#,'-1','-2','-3','-4']) # we use 'E' to stand for everything else other than W or C.
    #add_transition(input label, current state, next state)
    dra.add_transition('W',0,'Crash')
    dra.add_transition('W','Win','Win')
    dra.add_transition('W','Home','Win')
    dra.add_transition('W','Crash','Crash')
    
    dra.add_transition('H',0,'Home')
    dra.add_transition('H','Home','Crash')
    dra.add_transition('H','Win','Home')
    dra.add_transition('H','Crash','Crash')
    
    
    dra.add_transition('E',0,0)
    dra.add_transition('E','Win',0)
    dra.add_transition('E','Home','Home')
    dra.add_transition('E','Crash','Crash')
    
    
    dra.add_transition('C',0,'Crash')
    dra.add_transition('C','Win','Crash')
    dra.add_transition('C','Crash','Crash')
    dra.add_transition('C','Home','Crash')
    
    J0={'Crash',0}
    K0={}
    rabin_acc=[(J0,K0)]
    dra.add_rabin_acc(rabin_acc)

    


