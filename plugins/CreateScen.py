import numpy as np

# Import the global bluesky objects. Uncomment the ones you need
from bluesky import traf, sim  # , settings, navdb, traf, sim, scr, tools
from bluesky.tools import datalog, areafilter
from bluesky.core import Entity, timed_function
from bluesky.tools.aero import ft, kts, nm, fpm
from bluesky.tools.geo import *
from bluesky import stack
from bluesky import core

import os

def init_plugin():

    # Addtional initilisation code
    createscen = CreateScen()

    # Configuration parameters
    config = {
        "plugin_name": "CreateScen",
        # This is a sim plugin
        "plugin_type": "sim",
    }

    return config

class CreateScen(Entity):
    def __init__(self):
        super().__init__()

    # @stack.command(name = "ECHO_CREATE")
    # def cre_here(self):
    #     stack.stack(f'ECHO HERE')

    @stack.command(name = "Create_Scen")
    def cre_scen(self, dt_lookahead: int, rpz: float, lat: float = 52.5, lon: float = 5.3):
        box_size_in_dt_lookahead = dt_lookahead if dt_lookahead > 20 else 20

        rpz_in_nm = round(rpz / nm, 4)

        str_1 = f'00:00:00.00>NOISE ON'
        str_2 = f'00:00:00.00>IMPL ADSB ADSL'
        str_3 = f'00:00:00.00>ASAS DETECTADSL'
        str_4 = f'# 00:00:00.00>RESO RESOADSL'
        str_5 = f'00:00:00.00>PAN {lat} {lon}'
        str_6 = f'00:00:00.00>++++++++++++++'
        str_7 = f'00:00:00.00>DTLOOK {dt_lookahead}'
        str_8 = f'00:00:00.00>RPZ {rpz_in_nm}'
        str_9 = f'00:00:00.00>LOGADSL {dt_lookahead} {rpz_in_nm}'
        str_10 = f'00:00:00.00>SPAWN_CONF_LOOKAHEAD {lat} {lon} {box_size_in_dt_lookahead}'
        str_11 = f'00:00:00.00>FF {dt_lookahead * 2}'

        scenario_path = "/Users/sryhandiniputeri/bluesky/scenario/"
        filename = f'spawn_conflict_dt_{dt_lookahead}_rpz_{int(rpz)}.scn'
        fullname = f'{scenario_path}{filename}'

        if os.path.exists(fullname):
            os.remove(fullname)
            
        f = open(fullname, "a")
        f.write(f'{str_1}\n{str_2}\n{str_3}\n{str_4}\n{str_5}\n{str_6}\n{str_7}\n{str_8}\n{str_9}\n{str_10}\n{str_11}\n')
        f.close()

        stack.stack(f'ECHO Scenario Created {filename}')

        return filename

    @stack.command(name = "Create_Batch")
    def cre_batch(self,
                  dt_lookahead: int,
                  rpz: float, 
                  duration: float,
                  nb_of_scen: int = 10,
                  lat: float = 52.5,
                  lon: float = 5.3):

        created_scen = self.cre_scen(dt_lookahead, rpz, lat, lon)
        minute, second = self.seconds_to_minutes(duration)

        scenario_path = "/Users/sryhandiniputeri/bluesky/scenario/"
        filename = f'batch_spawn_conflict_dt_{dt_lookahead}_rpz_{int(rpz)}.scn'
        fullname = f'{scenario_path}{filename}'

        if os.path.exists(fullname):
            os.remove(fullname)
        
        f = open(fullname, "a")
        

        for i in range(nb_of_scen):
            scen_name = f'Spawn_conflict_{i+1}'
            str_1 = f'0:00:00.00>SCEN {scen_name}'
            str_2 = f'00:00:00.00>PCALL {created_scen}'
            str_3 = f'00:{minute}:{second}.0>HOLD'

            f.write(f'{str_1}\n{str_2}\n{str_3}\n\n')

        f.close()

    def seconds_to_minutes(self, seconds):
        minutes = seconds // 60   # Integer division to get the number of full minutes
        remaining_seconds = seconds % 60   # Modulo operator to get the number of remaining seconds
        return '{:02d}'.format(int(minutes)), '{:02d}'.format(int(remaining_seconds))


                