from __future__ import division
from MDP import *
import math
import time
from Policy import *
from gridworld import *
# from SymbolicSCC import *
import copy
import random


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

    count = [dict() for x in range(2)]
    count_sum = [dict() for x in range(2)]
    sharedcount = [dict() for x in range(1)]
    sharedcount_sum = [dict() for x in range(1)]
    
    regStates=['origin','north','south', 'west','east','northeast','northwest', 'southeast','southwest']
    for n in range(gwg.nagents):
        count[n] ={(s,a,next_s,regionName) : 0 for s in regStates for a in mdp[n].alphabet for next_s in regStates for regionName in gwg.regions.keys() }
        count_sum[n]= {(s,a,regionName) : 0 for s in regStates for a in mdp[n].alphabet for regionName in gwg.regions.keys()}
    
    for n in range(gwg.nagents-1):
        sharedcount[n] = {(s,a,next_s,regionName) : 0 for s in regStates for a in mdp[n].alphabet for next_s in regStates for regionName in gwg.regions.keys() }
        sharedcount_sum[n]= {(s,a,regionName) : 0 for s in regStates for a in mdp[n].alphabet for regionName in gwg.regions.keys()}
    
    policyHistory=[None]*gwg.nagents
    log = open(str(gwg.nrows)+'x'+str(gwg.ncols)+str(gwg.nagents)+'agentsResults.txt','w')
    #Initialize the learned gridworld MDP.
    aregionMDPstates = ['origin','south','north','east','west','southwest','southeast','northeast','northwest']
    aregionMDPprob = {a: np.zeros((len(aregionMDPstates),len(aregionMDPstates))) for a in mdp[0].alphabet} #initialize the transition probability
    aregionMDP= MDP('origin', list(mdp[0].alphabet), aregionMDPstates,aregionMDPprob)
    regionMDP = [None]*gwg.nagents
    knownRegions = [set()]*gwg.nagents
    knownRegionsPre = [set()]*gwg.nagents
    for n in range(gwg.nagents):
        regionMDP[n]={regionName: aregionMDP for regionName in gwg.regions.keys()}
        for regs in initKnownRegions:
            knownRegions[n].add(regs)
            knownRegionsPre[n].add(regs)
            i = regionMDP[n][regs].states.index('origin')
            for a in regionMDP[n][regs].alphabet:
                next_s_list = []
                for nextStates in reg_nextStates(a):
                    j = regionMDP[n][regs].states.index(nextStates)
                    regionMDP[n][regs].prob[a][i,j] = gwg.getDirnProbs(regs,nextStates,a)
                    next_s_list.append(nextStates)
                regionMDP[n][regs].add_transition(a,'origin',next_s_list)



    knownGWMDP = [None]*gwg.nagents
    V = [None]*gwg.nagents
    trueProdMDP = [None]*gwg.nagents
    current_s = [None]*gwg.nagents
    H = [None]*gwg.nagents
    HS = [None]*gwg.nagents
    Hpre = [None]*gwg.nagents
    for n in range(gwg.nagents):
        knownGWMDP[n] = copy.deepcopy(MDP(int(gwg.current[n]), list(mdp[n].alphabet), list(mdp[n].states), {a: np.zeros((gwg.nstates,gwg.nstates)) for a in mdp[n].alphabet})) # initialize the known gridworld MDP.
        knownGWMDP[n].L = mdp[n].L.copy()
        for regs in knownRegions[n]:
            for s in gwg.regions[regs]:
                for a in gwg.actlist:
                    knownGWMDP[n].prob[a][s] = copy.deepcopy(mdp[n].prob[a][s])
        knownGWMDP[n].setPost(knownGWMDP[n].prob)
        
        V.append(dict([])) #initialize the value function.
        H[n] = set()
        for kr in knownRegions[n]:
            H[n] = H[n].union(gwg.regions[kr])
        trueProdMDP[n] = copy.deepcopy(mdp[n].productMDP(dra[n]))
        current_s[n]=trueProdMDP[n].initial_state
        HS[n]= set([(h,s) for h in H[n] for s in dra[n].states])
        Hpre[n]=H[n].copy() # record the set of known states. 

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
        knownProdMDP[n]=knownGWMDP[n].productMDP(dra[n])
        if n > 0:
            avoidRegions[n] = avoidRegions[n].union(exploreRegions[n-1])

        exploreRegions[n] = set(random.sample((set(gwg.regions.keys()) - set(knownRegions[n]) - avoidRegions[n]),1))
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