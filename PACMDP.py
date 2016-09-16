from __future__ import division
from MDP import *
import math
import time
from Policy import *
from gridworld import *
# from SymbolicSCC import *
import copy
import random
from gridworldLearner import *

def reg_nextStates(a):
    if a == 'N':
        nextStates=['north','northeast','northwest']
    if a == 'S':
        nextStates=['south','southeast','southwest']
    if a == 'W':
        nextStates=['west','southwest','northwest']
    if a == 'E':
        nextStates=['east','northeast', 'southeast']
    return nextStates


def exploit_explore(gwg,mdp,dra,initKnownRegions,T):

    start = time.clock()

    gwl = GridworldLearner(gwg.current,['gravel'],10,0.05,0.9,10,gwg.nrows, gwg.ncols, gwg.nagents, gwg.targets, [],gwg.regions )


    V = [None]*gwg.nagents
    trueProdMDP = [None]*gwg.nagents
    current_s = [None]*gwg.nagents
    HS = [None]*gwg.nagents
    Hpre = [None]*gwg.nagents
    for n in range(gwg.nagents):
        V.append(dict([])) #initialize the value function.
        trueProdMDP[n] = copy.deepcopy(mdp[n].productMDP(dra[n]))
        current_s[n]=trueProdMDP[n].initial_state
        HS[n]= set([(h,s) for h in gwl.H[n] for s in dra[n].states])
        Hpre[n]=gwl.H[n].copy() # record the set of known states. 

    iter_count = [0]*gwg.nagents # the number of actions taken from the begining. We limit the number of iter_counts by placing an upper bound limit.
    limit = 200000
    
    knownProdMDP = [None]*gwg.nagents
    subknownProdMDP = [None]*gwg.nagents
    Win = [None]*gwg.nagents
    policyAEC = [None]*gwg.nagents
    policy_init = [None]*gwg.nagents
    policyT = [None]*gwg.nagents
    V = [None]*gwg.nagents
    value = [None]*gwg.nagents
    commagent = 0
    collagent = 0
    
    exploreRegions = [set()]*gwg.nagents
    avoidRegions = [set()]* gwg.nagents

    for n in range(gwg.nagents):
        gwl.knownGWMDP[n].L = mdp[n].L.copy()
        knownProdMDP[n]=gwl.knownGWMDP[n].productMDP(dra[n])
        if n > 0:
            avoidRegions[n] = avoidRegions[n].union(exploreRegions[n-1])

        exploreRegions[n] = set(random.sample((set(gwg.regions.keys()) - set(gwl.knownRegions[n]) - avoidRegions[n]),1))
        exploreStates = set()
        for reg in exploreRegions[n]:
            exploreStates = exploreStates.union(set([(h,s) for h in gwg.regions[reg] for s in dra[n].states]))
        subknownProdMDP[n]=knownProdMDP[n].sub_MDP(HS[n],exploreStates)
    
        Win[n]= {-1}
        policyAEC[n] = {-1: set(gwg.actlist)}
        V[n], policy_init[n] = T_step_value_iter(subknownProdMDP[n],T, (Win[n],policyAEC[n]))
        policyT[n]=dict([])
        for s in knownProdMDP[n].states:
            if s in HS[n]:
                policyT[n][s]=policy_init[n][s]
            elif s == -1:
                policyT[n][s]=policy_init[n][-1]
            elif s == -4:
                policyT[n][s]=policy_init[n][-4] 
    for n in range(gwg.nagents):
        print "agent ", n, "exploring ", exploreRegions[n]

    resets = [None]*2
    resets[0] = 0
    resets[1] = 0
    steps = [0]*gwg.nagents
    learnsteps = [0]*gwg.nagents
    finished = [0]*gwg.nagents
    unknowncommagent = 0
    policyagentshare = [None]*gwg.nagents