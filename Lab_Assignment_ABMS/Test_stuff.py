import random
import os
import pandas as pd
import networkx as nx
import matplotlib.pyplot as plt
import time as timer
import pygame as pg
from single_agent_planner import calc_heuristics
from visualization import map_initialization, map_running
from Aircraft import Aircraft
from independent import run_independent_planner
from prioritized import run_prioritized_planner
from cbs import run_CBS
# aircraft_list = [<Aircraft.Aircraft object at 0x7fb73cd4ea30>, <Aircraft.Aircraft object at 0x7fb73cd4ea00>, <Aircraft.Aircraft object at 0x7fb73cd4ea90>, <Aircraft.Aircraft object at 0x7fb73cd4eaf0>, <Aircraft.Aircraft object at 0x7fb73cd4ea60>, <Aircraft.Aircraft object at 0x7fb73cd4eb20>, <Aircraft.Aircraft object at 0x7fb73cd4ebb0>, <Aircraft.Aircraft object at 0x7fb73cd4ec10>, <Aircraft.Aircraft object at 0x7fb73cd4eb80>]

def seq(start, stop, step=1):
    n = int(round((stop - start)/float(step)))
    if n > 1:
        return([start + step*i for i in range(n+1)])
    elif n == 1:
        return([start])
    else:
        return([])

print(seq(1,12,1/2))