""" A plugin to spawn traffic through a circular area.
The inputs are the desired traffic numbers within the circle, the circle position (center, in lat/lon) and the circle radius (in meters), as well as the aircraft type.
The destination waypoints can be set, but those (for good practice) should be far outside the experiment area for more 
The headings are randomized, as are the spawn locations along the borders of the spawn zone.

 """
import numpy as np
import pandas as pd
import datetime

# Import the global bluesky objects. Uncomment the ones you need
from bluesky import traf, sim  # , settings, navdb, traf, sim, scr, tools
from bluesky.tools import datalog, areafilter
from bluesky.core import Entity, timed_function
from bluesky.tools.aero import ft, kts, nm, fpm
from bluesky.tools.geo import *
from bluesky import stack
from bluesky import core
import time
import datetime

import os

def init_plugin():

    # Addtional initilisation code
    loglossev = LogLOSSev()

    # Configuration parameters
    config = {
        "plugin_name": "LogLOSSev",
        # This is a sim plugin
        "plugin_type": "sim",
    }

    return config


class LogLOSSev(Entity):
    def __init__(self):
        # log_header = "id,latitude,longitude,true_positive,false_positive,false_negative"
        self.log_header = "simt,acid1,acid2,lat1,lon1,lat2,lon2,cpalat,cpalon"
        self.var_initiated = False

        super().__init__()

    def dep_var_comp(self):
        dep_var_1 = 0
        dep_var_2 = 0
        dep_var_3 = 0

        return dep_var_1, dep_var_2, dep_var_3
    
    @stack.command(name="LogLOSSev")
    def log_adsl(self, lookahead: int, rpz: float):
        self.log_folder = "/Users/sryhandiniputeri/bluesky/log/"
        self.str_lookahead = "DT_" + str(lookahead)
        self.str_rpz = "_RPZ_" + str(int(rpz*nm))

        self.log_dir = (self.log_folder+self.str_lookahead+self.str_rpz)
        folder_exist = os.path.isdir(self.log_dir)

        if not folder_exist:
            os.makedirs(self.log_dir)

        self.lossev = datalog.crelog(self.log_dir + "/LogLOSSev", None, self.log_header)
        self.lossev.start()
        self.var_initiated= True

        stack.stack(f'ECHO LOG CREATED')
        

    @core.timed_function(name="statcomp_lossev", dt=1)
    def statcomp_here(self):

        # if(self.var_initiated):

        #     if(traf.cd.lospairs):
        #         if(self.los_sev > traf.cd.dist[0]):
        #             self.los_sev = traf.cd.dist[0]
                
        #         print(self.los_sev)
            # self.lossev.log()

            # self.lossev.log(
            #     # traf.id,
            #     # traf.lat,
            #     # traf.lon,
            #     traf.cd.nb_true_positive,
            #     traf.cd.nb_false_positive,
            #     traf.cd.nb_false_negative,
            #     traf.cd.rpz[0],
            #     traf.cd.dtlookahead[0],
            #     traf.adsb.hpos_noise_m,
            #     len(traf.cd.confpairs_all),
            #     len(traf.cd.lospairs_all)
            # )

        return
