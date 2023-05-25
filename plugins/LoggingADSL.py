""" A plugin to spawn traffic through a circular area.
The inputs are the desired traffic numbers within the circle, the circle position (center, in lat/lon) and the circle radius (in meters), as well as the aircraft type.
The destination waypoints can be set, but those (for good practice) should be far outside the experiment area for more 
The headings are randomized, as are the spawn locations along the borders of the spawn zone.

 """
import numpy as np
import pandas as pd

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
    loggingADSL = LoggingADSL()

    # Configuration parameters
    config = {
        "plugin_name": "LoggingADSL",
        # This is a sim plugin
        "plugin_type": "sim",
    }

    return config


class LoggingADSL(Entity):
    def __init__(self):
        # log_header = "id,latitude,longitude,true_positive,false_positive,false_negative"
        self.log_header = "simt,true_positive,false_positive,false_negative,rpz,dtlookahead,hpos_noise_m,nb_conf_total,nb_los_total"
        self.var_initiated = False

        super().__init__()

    def dep_var_comp(self):
        dep_var_1 = 0
        dep_var_2 = 0
        dep_var_3 = 0

        return dep_var_1, dep_var_2, dep_var_3
    
    @stack.command(name="LOGADSL")
    def log_adsl(self, lookahead: int, rpz: float, hpos: float, use_adsl: bool, delay_stdev: float):
        log_folder = "/Users/sryhandiniputeri/bluesky/log/"
        str_lookahead = "DT_" + str(lookahead)
        str_rpz = "_RPZ_" + str(int(rpz*nm))
        str_hpos = "_hpos_" + str(hpos)
        str_use_adsl = "_adsl_" + str(use_adsl)
        str_adsl_stdev = "_stdev_" + str(delay_stdev)

        self.log_dir = (log_folder + str_lookahead + str_rpz+str_hpos + str_use_adsl + str_adsl_stdev)
        folder_exist = os.path.isdir(self.log_dir)

        if not folder_exist:
            os.makedirs(self.log_dir)

        self.logADSL = datalog.crelog(self.log_dir + "/LoggingADSL", None, self.log_header)
        self.logADSL.start()
        self.var_initiated= True

        stack.stack(f'ECHO LOG CREATED')


    @core.timed_function(name="statcomp", dt=1)
    def statcomp(self):
        if(self.var_initiated):
            self.logADSL.log(
                # traf.id,
                # traf.lat,
                # traf.lon,
                traf.cd.nb_true_positive,
                traf.cd.nb_false_positive,
                traf.cd.nb_false_negative,
                traf.cd.rpz[0],
                traf.cd.dtlookahead[0],
                traf.adsb.hpos_noise_m,
                len(traf.cd.confpairs_all),
                len(traf.cd.lospairs_all)
            )

        return
    
    @stack.command(name="LogCPA")
    def log_cpa(self, scenario_name, rpz: float):
        cpa_los_sev = []

        df_1 = pd.DataFrame(traf.cd.cpa_closest, index=traf.id, columns=traf.id)

        current_datetime = datetime.datetime.now()
        formatted_datetime = current_datetime.strftime("%Y_%m_%d_%H_%M_%S")

        dictionary_cpa = {(index, column): (rpz-value)/rpz*100 for index, row in df_1.iterrows() for column, value in row.items() if value < 100}
        cpa_los_sev.extend(list(dictionary_cpa.values()))

        df = pd.DataFrame({'cpa_los_sev': cpa_los_sev})
        df.to_csv(f'{self.log_dir}/cpa_{scenario_name}_{formatted_datetime}.log')

        stack.stack("ECHO LOGCPA CREATED")
