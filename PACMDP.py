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


def exploit_explore(gwg,gwl,mdp,dra,initKnownRegions,T):

    start = time.clock()



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
    
    commagent = 0
    collagent = 0
    resets = [None]*2
    resets[0] = 0
    resets[1] = 0
    steps = [0]*gwg.nagents
    learnsteps = [0]*gwg.nagents
    finished = [0]*gwg.nagents
    unknowncommagent = 0
    policyagentshare = [None]*gwg.nagents
    
    
    while True:
        for n in range(gwg.nagents):
            for m in range(n+1,gwg.nagents):
                if gwl.check_comm(n,m):
                    gwl.share_count()
                    commagent += 0
                    if gwg.getStateRegion(current_s[n][0]) == gwg.getStateRegion(current_s[k][0]) and (gwg.getStateRegion(current_s[k][0]) not in knownRegions[n] or gwg.getStateRegion(current_s[k][0]) not in knownRegions[k]):
                        unknowncommagent = 0
                    for a in gwl.actlist:
                        gwl.update_region_mdp(gwl.getStateRegion(current_s[n][0]),a,n)
                        gwl.update_region_mdp(gwl.getStateRegion(current_s[m][0]),a,m)
                    gwl.knownGWMDP[l]=gwl.update_knownGWMDP(n)
                    gwl.knownGWMDP[l]=gwl.update_knownGWMDP(m)
                    if current_s[n][1] != 'C':
                        current_s[n] = (current_s[n][0],'Home')
                    if current_s[m][1] != 'C':
                        current_s[m] = (current_s[n][0],'Home')
                            
                            
        for n in range(gwg.nagents):
            if exploreRegions[n] in gwl.knownRegions[n]:
                print('Agent {} finished exploring {}'.format(n,exploreRegions[n]))
                print('Agent {} knows {}'.format(n,gwl.knownRegions[n]))
                exploreRegions[n] = set()
                avoidRegions[n] = set()
                if n > 0:
                    avoidRegions[n] = avoidRegions[n].union(exploreRegions[n-1])
                    
                exploreRegions[n] = set(random.sample((set(gwg.regions.keys()) - set(gwl.knownRegions[n]) - avoidRegions[n]),1))
                print ('Agent {} now exploring {}'.format(n,exploreRegions[n]))
                exploreStates = set()
                for reg in exploreRegions[n]:
                    exploreStates = exploreStates.union(set([(h,s) for h in gwg.regions[reg] for s in dra[n].states]))
                
                HS[n]= set([(h,s) for h in gwl.H[n] for s in dra[n].states])
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
        if gwl.knownRegions == [set(gwl.regions.keys())]*gwl.nagents:
            break
        
        for n in range(gwg.nagents):
            iter_count[n]=iter_count[n]+1    
            act = random.sample(policyT[n][current_s[n]], 1)[0]
            next_s = trueProdMDP[n].sample(current_s[n],act)
            regionName= gwg.getStateRegion(current_s[n][0])
            if regionName not in gwl.knownRegions[n]:
                dirn = gwl.checkdirn(current_s[n][0],next_s[0])
                gwl.update_count(regionName,n, act, dirn)
                gwl.update_region_mdp(regionName, act, n)
                gwl.update_knownGWMDP(n)
                if gwl.knownGWMDP[n].P(current_s[n][0],a,next_s[0]) == 1:
                    current_s[agentnum] = trueProdMDP[agentnum].init
                    resets[n][0]+=1
                    if current_s[agentnum] in Win[agentnum]:
                        resets[agentnum][1]+=1
                else:
                    current_s[n] = next_s
            gwg.current[n] = current_s[n][0]
            gwl.current[n] = current_s[n][0]
            gwg.render()
            steps[n] += 1
            if gwg.getStateRegion(current_s[n][0]) in gwl.knownRegions[n]:
                learnsteps[n]+= 1
    