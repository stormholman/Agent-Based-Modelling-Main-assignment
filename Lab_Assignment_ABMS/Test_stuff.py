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






def prioritized_aircraft_list(aircraft_list):
    prioritized_aircraft_list = []
    for ac in aircraft_list:
        if ac.type == 'a_d':
            prioritized_aircraft_list.append(ac)
    for ac in aircraft_list:
        if ac.type == 'a_a':
            prioritized_aircraft_list.append(ac)
    return prioritized_aircraft_list