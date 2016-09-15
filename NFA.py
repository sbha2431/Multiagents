__author__ = 'Jie Fu, jief@seas.upenn.edu'

from types import *

class ExceptionFSM(Exception):

    """This is the FSM Exception class."""

    def __init__(self, value):
        self.value = value

    def __str__(self):
        return `self.value`

class NFA(object):

    """This is a Nondeterministic Finite State Automaton (NFA).
    """

    def __init__(self, initial_state=None, alphabet=None ,final_states=None, memory=None):
        self.state_transitions = {}
        self.final_states = set([])
        self.state_transitions=dict()
        self._pre_cache  = dict()
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

    def add_transition(self,input_symbol,state,next_state_list):
        if state not in self.states:
            self.states.append(state)
        for each_next in next_state_list:
            if each_next not in self.states:
                self.states.append(each_next)
        if input_symbol not in self.alphabet:
            self.alphabet.append(input_symbol)
        #if self.state_transitions.has_key((input_symbol,state)):
        #    pass
        #else:
        self.state_transitions[input_symbol,state]=next_state_list
        return
    
    def get_transition (self, input_symbol, state):

        """This returns a list of next states given an input_symbol and state.
        """

        if self.state_transitions.has_key((input_symbol, state)):
            return self.state_transitions[(input_symbol, state)]
        else:
            return None
    
    
    def get_Pre (self, state):

        """This returns a list of next states given an input_symbol and state.
        """
        assert state in self.states
        pre = []
        for s in self.states:
            for a in self.alphabet:
                if state in self.get_transition(a,s):
                    pre.append((a,s))
        return pre