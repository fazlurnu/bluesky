import bluesky as bs
from bluesky.core import Entity, timed_function
from bluesky.stack import command
from bluesky import stack
from bluesky.tools import datalog
from bluesky.tools.aero import ft, kts, nm
from bluesky.tools.geo import kwikpos

import numpy as np
import pandas as pd
import datetime

def init_plugin():
    # Configuration parameters
    config = {
        'plugin_name': 'log_uncertainty',
        'plugin_type': 'sim',
        'reset': reset
    }
    
    bs.traf.log_uncertainty = LogUncertainty()
    return config

def reset():
    bs.traf.log_uncertainty.reset()

def create_pos_noise_samples(lat_ground_truth, lon_ground_truth, nb_samples = 10000, sigma = 15):
    qdr = np.degrees(np.random.uniform(0, 2*np.pi, nb_samples))
    dist = np.random.normal(0, sigma, nb_samples)/nm

    lat_noise = kwikpos(lat_ground_truth, lon_ground_truth, qdr, dist)[0]
    lon_noise = kwikpos(lat_ground_truth, lon_ground_truth, qdr, dist)[1]

    return lat_noise, lon_noise

class LogUncertainty(Entity):
    def __init__(self):
        super().__init__()
        
    @stack.command(name="CreateUncertainty")
    def log_cpa(self, scenario_name, rpz: float):
        lat_ownship = 52.3169
        lon_ownship = 4.7459
        hdg_ownship = 0
        gs_ownship = 20 * kts
        id_ownship = 'GA123'

        id_intruder = 'JT123'
        dpsi = 45
        dcpa = 0
        tlosh = 15
        gs_intruder = 25 * kts

        print(gs_intruder)

        bs.traf.cre(id_ownship, actype="M600", aclat=lat_ownship, aclon=lon_ownship, achdg=hdg_ownship, acalt=200, acspd=gs_ownship)

        bs.traf.creconfs(id_intruder, "M600", bs.traf.id2idx(id_ownship), dpsi, dcpa, tlosh*1.1, dH=None, tlosv=None, spd=gs_intruder)

        # df_1 = pd.DataFrame(bs.traf.cd.dist_closest, index=bs.traf.id, columns=bs.traf.id)

        # current_datetime = datetime.datetime.now()
        # formatted_datetime = current_datetime.strftime("%Y_%m_%d_%H_%M_%S")

        # dictionary_cpa = {(index, column): (rpz-value)/rpz*100 for index, row in df_1.iterrows() for column, value in row.items() if value < 100}
        # values = list(dictionary_cpa.values())
        # keys = list(dictionary_cpa.keys())

        # df = pd.DataFrame({'los_sev_pair': keys, 'los_sev_val': values})

        # df.to_csv(f'{self.log_dir}/dist_{scenario_name}_{formatted_datetime}.log')
        
        return
    
    @timed_function(name="PrintConfPairs", dt=0.5)
    def printconfpairs(self):
        print(bs.traf.cd.confpairs)

        return