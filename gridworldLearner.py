from __future__ import division
from gridworld import *
from MDP import *
from operator import itemgetter

confidence_parameters = [(0.9, 1.645),
                         (0.95, 1.96),
                         (0.98, 2.326),
                         (0.99, 2.576)]
n_states = 0
T = 0
e = 0.01
d = 0.95
count = dict()
count_sum = dict()
unknown_trans = set()
steps_before_known = dict()

class GridworldLearner(Gridworld):
    def __init__(self, initial,knownRegion,T,epsilon,delta,N,nrows= 8, ncols= 8, nagents = 1, targets=[], obstacles=[],regions = dict()):
        super(GridworldLearner, self).__init__(initial, nrows, ncols, nagents, [], [],regions)
        self.k = min([(x, y) for (x, y) in confidence_parameters if x >= d],
            key=itemgetter(0))[1]
        self.count = [dict() for x in range(nagents)]
        self.count_sum = [dict() for x in range(nagents)]
        self.count_home = dict()
        self.count_home_sum = dict()
        self.sharedcount = [dict() for x in range(nagents)]
        self.sharedcount_sum = [dict() for x in range(nagents)]
        self.sharedcount_home = [dict() for x in range(nagents)]
        self.sharedcount_home_sum = [dict() for x in range(nagents)]
        self.states = range(self.nstates)
        self.knownRegions = [set(knownRegion) for n in range(self.nagents)]
        self.H = [set() for n in range(self.nagents)]        
        
        self.count_home ={(s,a,next_s) : 0 for s in self.states for a in self.actlist for next_s in self.states }
        count_home_sum= {(s,a) : 0 for s in self.states for a in self.actlist}
        for n in range(self.nagents):
            self.count[n] ={(s,a,next_s) : 0 for s in self.states for a in self.actlist for next_s in self.states}
            self.count_sum[n]= {(s,a) : 0 for s in self.states for a in self.actlist}
            self.sharedcount_home[n] = {(s,a,next_s) : 0 for s in self.states for a in self.actlist for next_s in self.states}
            self.sharedcount_home_sum[n]= {(s,a) : 0 for s in self.states for a in self.actlist}
            
        for n in range(self.nagents-1):
            self.sharedcount[n] = {(s,a,next_s) : 0 for s in self.states for a in self.actlist for next_s in self.states}
            self.sharedcount_sum[n]= {(s,a) : 0 for s in self.states for a in self.actlist}
            
        regStates=['origin','north','south', 'west','east','northeast','northwest', 'southeast','southwest']
        aregionMDPprob = {a: np.zeros((len(regStates),len(regStates))) for a in self.actlist} #initialize the transition probability
        aregionMDP= MDP('origin', list(self.actlist), regStates,aregionMDPprob)
        self.regionMDP = [None]*self.nagents
        for n in range(self.nagents):
            self.regionMDP[n]={regionName: copy.deepcopy(aregionMDP) for regionName in self.regions.keys()}
            for regs in knownRegion:
                self.H[n] = self.H[n].union(add(self.regions[regs]))
                for a in self.regionMDP[n][regs].alphabet:
                    self.update_region_mdp(regs,a,n)
                    next_s_list = []
                    for nextStates in self.nextDirns(a):
                        next_s_list.append(nextStates)
                    self.regionMDP[n][regs].add_transition(a,'origin',next_s_list)
        self.knownGWMDP = [None]*self.nagents
        for n in range(self.nagents):
            self.knownGWMDP[n] = copy.deepcopy(MDP(self.current[n], list(self.actlist), list(self.states), {a: np.zeros((self.nstates,self.nstates)) for a in self.actlist})) # initialize the known gridworld MDP.
            self.update_knownGWMDP(n)       
            self.knownGWMDP[n].setPost(self.knownGWMDP[n].prob)
            
            
    def known_trans(self,s,a,next_s, T,agent):
        # this uses center limit theorem, which requires the sample is sufficiently large.
        # hence, return false if the sample is too small.
        if count[agent][(s,a,next_s)] < 50*self.N or count_sum[agent][(s,a)] == 0:
            return False
        # else, we need to compute the mean and var.
        mean= self.count[agent][(s,a,next_s)]/count_sum[agent][(s,a)]
        var= self.count[agent][(s,a,next_s)]*(count_sum[agent][(s,a)]-count[agent][(s,a,next_s)])/(math.pow(count_sum[agent][(s,a)],2)*(count_sum[agent][(s,a)]+1))
        N=100
        if var*k <= self.epsilon/(1.0*N*self.T): #e/(N*math.pow(T,2)):
            print "known act {} with next {} in {}".format(a,next_s,regionName)
            return True
        else:
            return False
    
    def known_state(self,s,agent):
        reg=self.getStateRegion(s)
        if self.known_region(reg,agent):
            return True
        else:
            return False
    
    def known_region(self,regionName,agent):
        for a in self.actlist:
            for nextState in self.next_Dirns(a):
                if not self.known_trans(regionName, a, nextState,agent):
                    return False
        return True
    
    def update_knownGWMDP(self,agent):
        probOfSuccess=dict()
        for reg in self.regions.keys():
            probOfSuccess[(reg,'N')]=[self.regionMDP[agent][reg].P('origin','N','north'), self.regionMDP[agent][reg].P('origin','N','northwest'), self.regionMDP[agent][reg].P('origin','N','northeast')]
            probOfSuccess[(reg,'S')]=[self.regionMDP[agent][reg].P('origin','S','south'), self.regionMDP[agent][reg].P('origin','S','southwest'), self.regionMDP[agent][reg].P('origin','S','southeast')]
            probOfSuccess[(reg,'W')]=[self.regionMDP[agent][reg].P('origin','W','west'), self.regionMDP[agent][reg].P('origin','W','southwest'), self.regionMDP[agent][reg].P('origin','W','northwest')]
            probOfSuccess[(reg,'E')]=[self.regionMDP[agent][reg].P('origin','E','east'), self.regionMDP[agent][reg].P('origin','E','southeast'), self.regionMDP[agent][reg].P('origin','E','northeast')]
        prob={ a:np.zeros((self.nstates, self.nstates)) for a in self.actlist}
        for s in self.knownGWMDP[agent].states:
            for a in self.knownGWMDP[agent].alphabet:
                prob=self.updateGWProbs(prob,s, a, probOfSuccess)
        self.knownGWMDP[agent].prob=prob
        return 
    
    def get_belief_prob(self,agent,s,a,next_s):
        prob = self.count[agent][(s,a,nextState)]/(self.count_sum[agent][(s,a)])
        return prob
                
    def update_region_mdp(self,regionName,a,agent):
        """
        This function update the learned regionMDP, not the product MDP.
        """
        i=self.regionMDP[agent][regionName].states.index('origin')
        if regionName in self.knownRegions[agent]:
            for nextState in self.nextDirns(a):
                j=self.regionMDP[agent][regionName].states.index(nextState)
                self.regionMDP[agent][regionName].prob[a][i,j] = self.getDirnProbs(regionName,nextState,a)
            return
        else:
            for nextState in self.regionMDP[agent][regionName].states:
                
                j=self.regionMDP[agent][regionName].states.index(nextState)
                if self.count_sum[agent][('origin',a,regionName)] !=0:
                    self.regionMDP[agent][regionName].prob[a][i,j] = self.get_belief_prob(agent,'origin',a,nextState)
                else:
                    self.regionMDP[agent][regionName].prob[a][i,j]=0
            if self.known_region(regionName,self.T,agent):
                self.knownRegions[agent].add(regionName)
        return
    
        
    def share_count(self):
        for s in self.states:
            for act in self.alphabet:
                for dirn in self.states:
                    for agent1 in range(self.nagents):
                        for agent2 in range(n+1,self.nagents):
                            self.count[agent1][s,act,dirn] += (self.count[agent2][s,act,dirn] - self.sharedcount[0][s,act,dirn])
                            self.count_sum[agent1][(s,act)]+= (self.count_sum[agent2][(s,act)] - self.sharedcount_sum[0][(s,act)])
        
        for agent1 in range(self.nagents):
            for agent2 in range(n+1,self.nagents):
                self.count[agent2] = self.count[agent1]
                self.count_sum[agent2] = self.count_sum[agent1]
                
        for n in range(len(sharedcount)):
            self.sharedcount[n] = count[agent1]
            self.sharedcount_sum[n] = count_sum[agent1]
    
    
    def updateGWProbs(self,prob,state,action,probOfSuccess):
        successors = [] 
            
        if state in self.walls:
            successors =[(state, 1)]
            for (next_state, p) in successors:
                prob[action][state,next_state]=p
                return prob
            
        northState = (self.isAllowed(state-self.ncols) and state-self.ncols) or state
        northwestState = (self.isAllowed(state-1-self.ncols) and state-1-self.ncols) or state
        northeastState = (self.isAllowed(state+1-self.ncols) and state -self.ncols+1) or state
            
        southState = (self.isAllowed(state+self.ncols) and state+self.ncols) or state
        southeastState = (self.isAllowed(state+1+self.ncols) and state+1+self.ncols) or state
        southwestState = (self.isAllowed(state-1+self.ncols) and state-1+self.ncols) or state
            
        westState = (self.isAllowed(state-1) and state-1) or state
        eastState = (self.isAllowed(state+1) and state+1) or state
    
    
        reg = self.getStateRegion(state)
        if action == 'N':   
            [p0,p1,p2] = probOfSuccess[(reg,'N')]
            successors.append((northState,p0))
            successors.append((northwestState,p1))
            successors.append((northeastState,p2))
    
            
        if action == 'S':
            [p0,p1,p2] = probOfSuccess[(reg,'S')]
            successors.append((southState,p0))
            successors.append((southwestState,p1))
            successors.append((southeastState,p2))
            
        if action == 'W':
            [p0,p1,p2] = probOfSuccess[(reg,'W')]
            successors.append((westState,p0))
            successors.append((southwestState,p1))
            successors.append((northwestState,p2))
    
        if action == 'E':
            [p0,p1,p2] = probOfSuccess[(reg,'W')]
            successors.append((eastState,p0))
            successors.append((southeastState,p1))
            successors.append((northeastState,p2))
            
        for (next_state,p) in successors:
            prob[action][state,next_state]+=p
        return prob
    
    