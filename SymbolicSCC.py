__author__ = 'Jie Fu, jief@seas.upenn.edu'
"""
Generate the set of SCC components in the graph of MDP.
"""

import os
import random
from SCCtarjan import *
#from SCC2 import *
import networkx as nx
import copy
import numpy as np
def get_BDD(mdp):
    S=[]
    S1=set([]) # the set of player 1's states
    Sr=set([]) # the set of random states (player2)
    edges=set([]) # the set of edges
    G=set([]) 
    goals=set([])
    dictG = dict([])
    for (J,K) in mdp.acc:
        G=G.union(K) # G is the union of all states that we aim to visit infinitely often.

    for s in mdp.states:
        dictG[s] = set()
        S1.add(s)
        if s in G:
            goals.add(s)

        for a in mdp.actlist:
            for next_s in mdp.states:
                if mdp.P(s,a,next_s) != 0:
                    Sr.add((s,a)) # the state s is player1's state and the state (s,a) is player 2's state.
                    edges.add((s, (s,a)))
                    edges.add(((s,a), next_s))
                    # edges.add((s,next_s))
                    dictG[s].add(next_s)
    S=list(S1)+list(Sr)
    #edges_indx= {(S.index(s),S.index(t)) for (s,t) in edges}
    #goals_indx={S.index(s) for s in goals}
    return S1, Sr,S, dictG, edges, goals

        
def generate_input_mdp(S1,Sr,S,edges,goals, filename):
    inputf= open(filename,'w+')
    inputf.write('{}\n'.format(len(S1)))
    inputf.write('{}\n'.format(len(Sr)))

    for e in edges:
        inputf.write('{} {}\n'.format(e[0], e[1]))
    inputf.write('-1')

    for g in goals:
        inputf.write('{}\n'.format(g))
    inputf.write('-1')

def is_accepting(mdp,A):
    for (J,K) in mdp.acc:
        if A.intersection(J) == set([]) and A.intersection(K) != set([]):
            return True
        else:
            return False

def win_lose(mdp):

    S1, Sr,S,dictG, edges, goals =get_BDD(mdp)
    G=nx.DiGraph()
    G.add_edges_from(edges)

    W1=set([])
    W2=set([])
    W=W1.union(W2)
        
    while True:
        inducedSet=set(S)-W
        #print "The remainning set is {}".format(len(inducedSet))
        inducedG=G.subgraph(list(inducedSet))
        SCCs=scc_decompose(inducedG,S)
        for C in SCCs:
            #print "the SCC is {}".format(C)
            if Post(G,C).issubset(C.union(W)): # to check if C is a bottom SCC in the graph induced by S\W.
                if Post(G,C).intersection(W1) !=set([])  or C.intersection(goals) != set([]):
                    W1=W1.union(C)
                else:
                    W2=W2.union(C)
        W1=Attr1(G,S1,Sr,W1)
        W2=AttrR(G,S1,Sr,W2)
        W=W1.union(W2)
        if W == set(S):
            break
    print "The length of W1 is {}".format(len(W1))
    print "The length of W2 is {}".format(len(W2))

    Win,Policy=get_AEC(G, S1,Sr, W1,W2)
    return Win, Policy

def get_AEC(G, S1,Sr, W1, W2):
    
    subG=G.subgraph(list(W1-W2))
    Win=set([])
    Policy=dict([])

    for (s,t) in subG.edges():
        if s in S1:
            Win.add(s)
            if Policy.has_key(s):
                Policy[s].add(t[1])
            else:
                Policy[s]=set([t[1]])

    return Win, Policy
            
            

def Post(G,C):
    Post=set([])
    for (x,y) in G.edges():
        if x in C:
            Post.add(y)
    return Post
        
def Pre(G,U):
    Pre=set([])
    for (x,y) in G.edges():
        if y in U:
            Pre.add(x)
    return Pre
        
def Attr1(G,S1,Sr,U):        
    X=U.copy()
    print "The size of X is {}".format(len(X))
    new_X =set([])
    while True:
        new_X=X.copy()
        for s in Sr:
            Y=set([])
            for (x,y) in G.edges():
                if x == s:
                    Y.add(y)
            if Y.issubset(X):
                new_X.add(s)
        for s in S1:
            for (x,y) in G.edges():
                if x==s and y in X:
                    new_X.add(s)
                    break
        if new_X != X:
            X=new_X.copy()
        else:
            break
    print "The attractor of X is {}".format(len(new_X))
    return new_X

def AttrR(G,S1,Sr,U):        
    return Attr1(G,Sr,S1,U)

def generate_input(G,S,filename):
    inputf= open(filename,'w+')
    inputf.write('{}\n'.format(len(S)))

    for (s,t) in G.edges():
        inputf.write('{} {}\n'.format(S.index(s), S.index(t)))
    inputf.write('-1')
    inputf.close()
    return
    

def scc_decompose(G,S):
    generate_input(G,S,"input.txt")
    os.system("scc/./scc < input.txt")
    outputf=open("SCC_Decomposition.txt", 'r')
    lines=[line.rstrip('\n') for line in outputf]
    indx_sccs= [l.split() for l in lines]
    SCCs=[]
    for scc in indx_sccs:
        A=set([S[int(i)] for i in scc])
        SCCs.append(A)
    return SCCs

def MEC(mdp):
    S1, Sr,S, dictG, edges, goals = get_BDD(mdp)
    SCCs = tarjan(dictG)
    C = []
    for T in SCCs:
        temp = []
        for t in T:
            temp.append((t,mdp.actlist))        
        C.append(temp)    
    while True:
        C2 = []
        for T in C:
            temp = []
            for (t,acts) in T: 
                acttemp = copy.deepcopy(acts)
                for a in acts:
                    if np.sum(mdp.P(t,a,T[u][0]) for u in range(len(T))) < 1:
                        acttemp.remove(a)
                temp.append((t,acttemp))           
            C2.append(temp)
        if C2 == C:
            break        
        dictG2 = dict([])
        allowedactlist = dict([])
        for T in C2:
            for (t,act) in T:
                dictG2[t] = set()
                allowedactlist[t] = set()
                for a in act:
                    for next_s in mdp.states:
                        if mdp.P(t,a,next_s) != 0:
                            dictG2[t].add(next_s)
                            allowedactlist[t].add(a)
        dictG = copy.deepcopy(dictG2)
        SCCs = tarjan(dictG)
        C = []
        for T in SCCs:
            temp = []
            for t in T:
                temp.append((t,allowedactlist[t]))
            
            C.append(temp)        
    return C2, allowedactlist
    
def getAMEC(MEC,AEC):
    AMEC = []
    for T in MEC:
        if len(set(T).intersection(AEC)) > 0:
            AMEC.append(T)
    return AMEC


