__author__ = 'Jie Fu, jief@seas.upenn.edu'

from MDP import *
import numpy as np
import random
from SymbolicSCC import *

def T_step_value_iter(mdp,T,AEC=None):
    """
    T- finite time horizon
    Value iteration: Vstate[s] the maximal probability of hitting the target AEC within T steps. 
    """
    policyT=dict([])
    Vstate1=dict([])
    if AEC == None:
        Win,policy= win_lose(mdp)
    else:
        Win=AEC[0]
        policy=AEC[1]
    policyT.update(policy)
    NAEC=set(mdp.states)-Win
    Vstate1.update({s: 1 for s in list(Win)})
    Vstate1.update({s: 0 for s in list(NAEC)})

    if NAEC ==  set([]):
        return Vstate1, policyT
    t=T
    Q=dict([])
    while t>0:
        Vstate = Vstate1.copy()
        for s in set(mdp.states)-Win:
            acts=mdp.actions(s)
            optimal=0
            act=None
            for a in mdp.actions(s):
                Q[(s,a)]= sum([mdp.P(s,a,next_s)* Vstate[next_s] for next_s in mdp.states])
                #Q[(s,a)]= np.inner(mdp.T(s,a), Vstate)
                if Q[(s,a)] >=optimal:
                    optimal=Q[(s,a)]
                    act=a
                else:
                    pass
            
            acts=set([])
            for act in mdp.actions(s):
                if Q[(s,act)] == optimal:
                    acts.add(act)
            Vstate1[s]=optimal
            policyT[s]= acts
            #        print "iteration: {} and the state value is {}".format(t, Vstate1)
        t=t-1
    return Vstate1,policyT

def value_iter_ent(gwg,mdp,regions,motioncost,count,count_sum,beliefprobs,targchoice,C):
    """
    T- finite time horizon
    Value iteration: Vstate[s] the maximal probability of hitting the target AEC within T steps. 
    """
    
    
    policyT=dict([])
    Vstate1=dict([])
    if AEC == None:
        Win,policy= win_lose(mdp)
    else:
        Win=AEC[0]
        policy=AEC[1]
    policyT.update(policy)
    NAEC=set(mdp.states)-Win
    Vstate1.update({s: 1 for s in list(Win)})
    Vstate1.update({s: 0 for s in list(NAEC)})
    t = 300
    Q=dict([])
    Vstate = dict([])
    Vstate.update({s:0 for s in mdp.states})
    Vstate.update({s : 0 for s in [gwg.targets[0][targchoice]]})
    Vstate.update({s : 0 for s in gwg.walls})
    diff = 10
    while t>0 and diff > 1:
        Vstateold = copy.deepcopy(Vstate)
        for s in set(mdp.states):
            acts=mdp.actions(s)
            optimal=-100
            act=None
            for a in mdp.actions(s):
                Q[(s,a)] = 0
                if beliefprobs[a][s][s] == 1:
                        R = 0 # If it's a permanent self transition there is no entropy
                else:
                    R = calcEntropyReward(gwg,count,count_sum,s,a,[],beliefprobs)
                for next_s in mdp.states:
                    ns = checkdirn(s,next_s,gwg.ncols)
                    # if ns == 0 or beliefprobs[a][s][s] == 1:
                    #     R = 0 # If it's a permanent self transition there is no entropy
                    # else:
                    #     R = calcEntropyReward(gwg,count,count_sum,s,a,ns,beliefprobs) 
                    
                    Q[(s,a)]= Q[(s,a)] + (beliefprobs[a][s][next_s]*(Vstate1[s] + C*R + 0.9*Vstate[next_s]))
                #Q[(s,a)]= np.inner(mdp.T(s,a), Vstate)
                if Q[(s,a)] >=optimal:
                    optimal=Q[(s,a)]
                    act=a
                else:
                    pass
            Vstate[s] = optimal
            
            acts=set([])
            for act in mdp.actions(s):
                if Q[(s,act)] == optimal:
                    acts.add(act)
            policyT[s]= acts
            #        print "iteration: {} and the state value is {}".format(t, Vstate1)
        print(t)            
        t=t-1
        #diff = np.abs(np.linalg.norm(Vstate.values()) - np.linalg.norm(Vstateold.values()))
        diff = Counter(Vstate) - Counter(Vstateold)
        diff = np.linalg.norm(diff.values())
        print(diff)
        
    return Vstate,policyT


def E_state_value_iter(mdp,epsilon, AEC):
    """
    Value iteration: Vstate[s] the maximal probability of hitting the
    target AEC.  Iteration terminates when the change in the state
    value cannot be improved by at least epsilon.  return: state
    values, policy and the number of iterations.
    """
    policyE=dict([])
    N=len(mdp.states)
    t=0
    Win=AEC[0]
    policy=AEC[1]
    Vstate1=dict([])
    policyE =dict([])
    policyE.update(policy)
    Vstate1.update({s: 1 for s in list(Win)})
    NAEC=set(mdp.states)-Win
    if NAEC ==  set([]):
        return Vstate1, policy
    Vstate1.update({s: 0 for s in list(NAEC)})

    Q=dict([])
    e=1.0
    while e > epsilon:
        t=t+1
        Vstate = Vstate1.copy()
        for s in set(mdp.states)-Win:
            acts=mdp.actions(s)
            optimal=0
            act=None
            for a in mdp.actlist:
                Q[(s,a)]= sum([mdp.P(s,a,next_s)* Vstate[next_s] for next_s in mdp.states])
                #                Q[(s,a)]= np.inner(mdp.T(s,a), Vstate)
                if Q[(s,a)] >=optimal:
                    optimal=Q[(s,a)]
                    act=a
                else:
                    pass
            for act in mdp.actions(s):
                if Q[(s,act)] == optimal:
                    acts.add(act)
            Vstate1[s]=optimal
            policyE[s]= acts
            #        print "iteration: {} and the state value is {}".format(t, Vstate1)
            e= abs( max([Vstate1[s] - Vstate[s] for s in mdp.states])) # the absolute error
    return Vstate1,policyE,t

def evaluate_policy2(mdp,policy,AEC):
    Win=AEC[0]
    policyAEC=AEC[1]
    U=dict([]) # define a map from a set of states into a set of variables.
    problem=LpProblem("MDP SOLVER", LpMinimize)
    #firstly we need to define a set of variables, which is the state value U(q)
    for state in mdp.states:
        U[state] = LpVariable("U"+str(state),0) # for each state, assign a state value variable.
    problem += LpAffineExpression([(U[state], 1) for state in mdp.states])
    # The state-value of a state in the AEC is 1.
    policyE=dict([])
    policyE.update(policy)
    for state in Win.intersection(mdp.states):
        problem += U[state]==1
    for state in set(mdp.states)-Win:
        act=random.choice(list(policyE[state]))
        C=LpAffineExpression([(U[next_state], mdp.P(state,act,next_state))  for next_state in mdp.states])
        problem+= U[state]-C ==0
    status = problem.solve()
    value={state: U[state].varValue for state in mdp.states}
    return value


def evaluate_policy_T(mdp,policyT,T,AEC):
    V=dict([])
    Win=AEC[0]
    policy=dict([])
    policy.update(policyT)
    for s in Win:
        V[s]=1
    for s in set(mdp.states)-Win:
        V[s]=0
    t=T
    while t>0:
        for s in set(mdp.states)-Win:
            if type(policy[s]) == set:
                a= random.choice(list(policy[s]))
            else:
                a=policy[s]
            V[s]= sum([mdp.P(s,a,next_s)* V[next_s] for next_s in mdp.states])  
        t=t-1
    return V
    
def balanced_wandering(mdp,s):
    act=random.choice(list(mdp.actions(s))) # randomly choose an action from the set of enabled actions.
    next_s=mdp.sample(s,act)
    return next_s,act

def exploit(mdp,s,policyT):
    acts=policyT[s]
    action=random.choice(list(acts))
    next_s=mdp.sample(s,action)
    return next_s,action


def getPdist(s,policyT,knownProdMDP,T):
    x = np.zeros((1,len(knownProdMDP.states)))
    x[0][knownProdMDP.states.index(s)] = 1
    Ptrans = np.zeros((len(knownProdMDP.states),len(knownProdMDP.states)))
    for srow in range(len(knownProdMDP.states)):
        act = policyT[knownProdMDP.states[srow]]
        for a in act:
            Ptrans[srow] += 1.0/len(act)*knownProdMDP.prob[a][srow]
    Pdist = np.mat(x)*np.mat(Ptrans)**T
    Pdist = np.array(Pdist)[0]
    return Pdist

"""
Test the code for computing the accepting end component.

if __name__=='__main__':    
    nfa = NFA()
    nfa.add_transition('a', 1, [2])
    nfa.add_transition('b', 1, [3])
    nfa.add_transition('b', 2, [2,3])
    nfa.add_transition('a', 3, [2,3])
    nfa.add_transition('a',2,[1,3])
    nfa.add_transition('b', 3, [3])
    F=set([3])
    AEC,policy=get_AEC(nfa, F)

"""

"""
Test the code for MDP policy computation.
"""
if __name__=='__main__':
    ex=read_from_file_MDP(ex.txt)
    ex.init=0
    translist=dict([])
    translist[(0,a)]=[(1,1)]
    

