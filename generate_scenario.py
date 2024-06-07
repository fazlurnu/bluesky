"""
Scenario generator for ADSL plugin testing.
Generates 
"""

import pickle as pkl
import datetime
import pandas as pd
import numpy as np
import os, sys
from bluesky.tools.geo import *
sys.path.append(os.path.join(os.path.dirname("adsl_bluesky/scenario/ADSL"), "lib"))

ft  = 0.3048                # m    of 1 foot
nm  = 1852.                 # m    of 1 nautical mile

"""
Settings
"""
initial_ownship_spawnloc = [50, 4] # lat, lon [deg]
theta_res = 1 # angular resolution [deg]

rpz_m = 50
rpz = rpz_m/nm # [m], converted to nm

dtlook = 15 #[s]
own_spd = 20 # [kts]

"""
Functions for scen writing
"""
def bstime(time_start):
    return str(datetime.timedelta(seconds=float(time_start)))

def write_line(text,file_id):
    name = f"testscen_{file_id}"
    with open(f'scenario/ADSL/{name}.scn', 'a') as fd:
        fd.write(text + '\n')

def write_to_batch(text,fname):
    with open(f'{filename}', 'a') as fd:
            fd.write(text + '\n')
    return


def write_lines_to_scen(df, file_id):
    # Basic setup for sim
    write_line("00:00:00.00> NOISE ON", file_id) # load adsl plugin
    write_line("00:00:00.00> asas on", file_id)
    write_line("00:00:00.00> plugin load adsl", file_id) # load adsl plugin
    write_line("00:00:00.00> IMPL ADSB ADSL", file_id) # load adsl plugin 
    write_line("00:00:00.00> plugin load DetectADSL", file_id) # load detection
    write_line("00:00:00.00> plugin load adsl_logger", file_id) # load adsl plugin
    write_line("00:00:00.00> plugin load pos_logger", file_id) # load adsl plugin
    write_line("00:00:00.00> asas detectADSL", file_id) # detection -> set correct
    write_line("00:00:00.00> plugin load ssd", file_id) # load SSD
    write_line("00:00:00.00> plugin load vo", file_id) # load VO
    write_line("00:00:00.00> reso MVP", file_id) # MVP on
    # write_line("00:00:00.00> startCDRlogs", file_id) # MVP on
    # write_line("00:00:00.00> reso SSD", file_id) # set conf reso
    # write_line("00:00:00.00> PRIORULES TRUE RS1", file_id) # set conf reso
    
    # write_line("00:00:00.00> plugin load traffichandler", file_id) # handler
    # write_line("00:00:00.00> plugin load m22logger", file_id) # log
    write_line("00:00:00.00> pan 50 4", file_id)
    write_line("00:00:00.00> zoom 25", file_id)
    write_line(f"00:00:00.00> rpz {rpz}", file_id) # set rpz
    write_line(f"00:00:00.00> dtlook {dtlook}", file_id) # set lookahead
    # write_line(f"00:00:00.00> startlogs", file_id) # start logs
    write_line(f"00:00:00.00> SCHEDULE 00:01:00.00 LogPos {file_id}",file_id)


    # Spawn Intruder
    dcpa_m = np.random.randint(0, 50) # 50 m is the rpz
    dcpa = dcpa_m / nm
    dH = 0
    tlosv = 0

    for idx, row in df.iterrows():
        spawn_col_index, spawn_row_index = divmod(idx, row_per_ac)

        heading = row['heading']
        speed = row['speed']
        ownship_id = row['ownship']
        intruder_id = row['intruder']

        ownship_lat = initial_ownship_spawnloc[0] + (spawn_row_index * 0.01)
        ownship_lon = initial_ownship_spawnloc[1] + (spawn_col_index * 0.01)

        # Spawn ownship, always same position. ID D1 for ownship.
        text = f"{bstime(0)}>CRE {ownship_id} M600 {ownship_lat} {ownship_lon} 0 100 {own_spd}" # ownship parameters are not varied
        write_line(text, file_id) # hdg and speed only used for filename purposes here

        text =  f"{bstime(0)}>CRECONFS {intruder_id} M600 {ownship_id} {heading} {dcpa} {dtlook*1.1} {dH} {tlosv} {speed}"
        write_line(text, file_id) # idem

    return 


"""
Scenario Generator loop
"""

from itertools import product

## create df

heading = np.arange(0, 360, 1)
# gs_intruder = np.arange(25, 35, 10)
gs_intruder = np.arange(5, 45, 10)

# heading = [180] * 10
# gs_intruder = [15] * 100

combinations = list(product(heading, gs_intruder))

ownship_list = []
intruder_list = []

for i in range(len(combinations)):
    ownship_list.append(f"DRO{i:04d}")
    intruder_list.append(f"DRI{i:04d}")

df = pd.DataFrame(combinations, columns=['heading', 'speed'])

df['ownship'] = ownship_list
df['intruder'] = intruder_list

# nb_ac_per_scen = 6*6
row_per_ac = 12
nb_ac_per_scen = row_per_ac * row_per_ac
partition = int(len(df)/(nb_ac_per_scen))

partial_scenario = 5

for nb_file in range(500):
    for i in range(partition):
        # if(i == partial_scenario):
        if(True):
            file_id = f"{i}_{nb_file}"
            df_here = df.iloc[(nb_ac_per_scen*i) : (nb_ac_per_scen*i)+nb_ac_per_scen]

            name = write_lines_to_scen(df_here, file_id)

# remove DS_store and find all scen files to put in batch
ds_store_file_location = f"scenario/ADSL/.DS_store"
if os.path.isfile(ds_store_file_location):
    os.remove(ds_store_file_location)

namelist = os.listdir(f"scenario/ADSL")
namelist = sorted(namelist)

# determine stoppage: for now 1 hour
hours_max = 0

filename = f"scenario/ADSL/ADSL_batch.scn"

counter = 0
for scenario_name in namelist:
    counter += 1
    
    # if(f"testscen_{partial_scenario}_") in scenario_name:
    if(True):
        scname = scenario_name.split(".")[0]
        write_to_batch(f"00:00:00.00>SCEN {scname}",filename)
        write_to_batch(f"00:00:00.00>PCALL adsl/{scenario_name}",filename)
        write_to_batch(f"00:00:00.00> ff",filename)
        write_to_batch(f"00:00:00.00> SCHEDULE 00:01:00.00 HOLD",filename)
        write_to_batch(f"00:00:00.00> SCHEDULE 00:01:00.00 LOGCPA {scname} {rpz_m}",filename)