import sys
import random
sys.path.append('/home/sudab/anaconda/lib/python2.7/site-packages')
from DFA import *
from gridworld import *
import copy
from MDP import *
from gridworldLearner import *
##### Create environment 20x20
obstacles=[368,348,328,308,288,268,188,168,148,361,362,363,364,365,366,367,181,182,183,184,185,186,187,332,312,292,272,192,172,152,72,52,32,333,334,335,336,337,338,197,198,222,265,283,247,255,296,278,103,23,101,146,36,137,115,76,118,92,112,132]
initial= [350,378]
nagents=2
commrange = 3
targets = [None]*nagents
ncols=20
nrows=20
nagents=2
commrange = 2
targets = [None]*nagents
targets[0]=[26,25,356]
targets[1] = [34,35,356]

## Set up regions
rooms = {'pavement':[],'grass':[],'sand':[]}
rooms['pavement'] = [range(21,32),range(41,52),range(61,72),range(81,92),range(101,112),range(121,132),range(141,152),range(161,172)]
rooms['grass'] = [range(201,209),range(221,229),range(241,249),range(261,268),range(281,288),range(301,308),range(321,328),range(341,348)]
rooms['sand'] = [range(32,39),range(52,59),range(72,79),range(92,99),range(112,119),range(132,139),range(152,159),range(172,179),range(192,199),range(212,219),range(232,239),range(252,259),range(273,279),range(293,299),range(313,319)]

rooms['pavement'] = sum(rooms['pavement'],[])
rooms['grass'] = sum(rooms['grass'],[])
rooms['sand'] = sum(rooms['sand'],[])
regions={'pavement': set(rooms['pavement']), 'grass': set(rooms['grass']), 'sand':set(rooms['sand']), 'gravel': {}}
regions['gravel']=set(range(0,nrows*ncols))- regions['pavement']-regions['grass']-regions['sand']


# gwg=Gridworld(initial, ncols, nrows, nagents, targets, obstacles,regions)
gwg = GridworldGUI(initial, ncols, nrows, nagents,targets, obstacles,regions)
gwg.render()
gwg.draw_state_labels()

mdp = MDP(initial[0],gwg.actlist,range(nrows*ncols),gwg.prob)

execfile('DFA.py')
region_map={targets[0][0]:'1', targets[0][1]: '2',targets[0][2]: '3'}
for w in obstacles:
    region_map[w] = '4' # never hitting the walls
for s in mdp.states:
    if s in region_map.keys():
        mdp.labeling(s,region_map[s])
    else:
        mdp.labeling(s,'E') 



# prodmdp = mdp.productMDP(dra)
# HS = set([(s,q) for s in gwg.regions['gravel'] for q in dra.states])
# sinkstates = set([(s,q) for s in gwg.regions['pavement'] for q in dra.states])
# submdp = prodmdp.sub_MDP(HS)
from PACMDP import *
epsilon = 0.05
delta = 0.9
T = 10
N = 1
commrange = 2

mdp1 = [mdp,mdp]
gwl = GridworldLearner(initial,['gravel'],T,epsilon,delta,N,commrange,gwg.nrows, gwg.ncols, gwg.nagents, gwg.targets, obstacles,gwg.regions )
exploit_explore(gwg,gwl,mdp1,[dra,dra],['gravel'],10)