""" BlueSky plugin template. The text you put here will be visible
    in BlueSky as the description of your plugin. """
import random
import numpy as np
from math import degrees, radians, sin, cos, sqrt, atan2

# Import the global bluesky objects. Uncomment the ones you need
from bluesky import core, stack, traf, sim  #, settings, navdb, sim, scr, tools
from bluesky.tools.aero import ft, nm, kts

from bluesky.tools import datalog, geo

from .ADSL import ADSL

import os
import time

### Initialization function of your plugin. Do not change the name of this
### function, as it is the way BlueSky recognises this file as a plugin.
def init_plugin():
    ''' Plugin initialisation function. '''
    trafADSL = TrafADSL()

    # Configuration parameters
    config = {
        # The name of your plugin
        'plugin_name':     'TrafADSL',

        # The type of this plugin. For now, only simulation plugins are possible.
        'plugin_type':     'sim',
        }

    # init_plugin() should always return a configuration dict.
    return config

class TrafADSL(core.Entity):
    ''' Example new entity object for BlueSky. '''
    def __init__(self):
        super().__init__()
        
        self.ac_counter = 0
        self.box_counter = 0

        self.R = 6737.0
        # log_header = "id,latitude,longitude"
        # self.loggedstuff = datalog.crelog("/Users/sryhandiniputeri/bluesky/log/TrafADSL_dtlookahead_50", None, log_header)

        self.adsl = ADSL()

    def create(self, n=1):
        ''' This function gets called automatically when new aircraft are created. '''
        # Don't forget to call the base class create when you reimplement this function!
        super().create(n)
        # After base creation we can change the values in our own states for the new aircraft

    def id2idx(self, acid):
        """Find index of aircraft id"""
        if not isinstance(acid, str):
            # id2idx is called for multiple id's
            # Fast way of finding indices of all ACID's in a given list
            tmp = dict((v, i) for i, v in enumerate(traf.id))
            return [tmp.get(acidi, -1) for acidi in acid]
        else:
             # Catch last created id (* or # symbol)
            if acid in ('#', '*'):
                return traf.ntraf - 1

            try:
                return traf.id.index(acid.upper())
            except:
                return -1

    @stack.command(name='DEL_ALL')
    def del_all(self):
        for id in traf.id:
            stack.stack(f'DEL {id}')

    @stack.command(name='CRECONFSADSL')
    def creconfs(self, acid, actype, targetid, dpsi: float, dcpa: float, tlosh: float, spd: float = -1.0):
        ''' Create an aircraft in conflict with target aircraft.
            Adapted from https://github.com/TUDelft-CNS-ATM/bluesky/blob/master/bluesky/traffic/traffic.py

            Arguments:
            - acid: callsign of new aircraft
            - actype: aircraft type of new aircraft
            - targetidx: id (callsign) of target aircraft
            - dpsi: Conflict angle (angle between tracks of ownship and intruder) (deg)
            - cpa: Predicted distance at closest point of approach (NM)
            - tlosh: Horizontal time to loss of separation ((hh:mm:)sec)
            - spd: Speed of new aircraft (CAS/Mach, kts/-)
        '''
        targetidx = self.id2idx(targetid)

        latref  = traf.lat[targetidx]  # deg
        lonref  = traf.lon[targetidx]  # deg
        altref  = traf.alt[targetidx]  # m
        trkref  = radians(traf.trk[targetidx])
        gsref   = traf.gs[targetidx]   # m/s
        cpa     = dcpa * nm            # m
        pzr     = traf.cd.rpz[targetidx] # m
        trk     = trkref + radians(dpsi)

        acvs = 0.0

        # Groundspeed is the same as ownship
        if spd < 0:
            gsn, gse = gsref * cos(trk), gsref * sin(trk)
        else:
            gsn, gse = spd * cos(trk) * kts, spd * sin(trk) * kts

        # Horizontal relative velocity vector
        vreln, vrele = gsref * cos(trkref) - gsn, gsref * sin(trkref) - gse
        # Relative velocity magnitude
        vrel    = sqrt(vreln * vreln + vrele * vrele)
        # Relative travel distance to closest point of approach
        drelcpa = tlosh * vrel + (0 if cpa > pzr else sqrt(pzr * pzr - cpa * cpa))
        # Initial intruder distance
        dist    = sqrt(drelcpa * drelcpa + cpa * cpa)
        # Rotation matrix diagonal and cross elements for distance vector
        rd      = drelcpa / dist
        rx      = cpa / dist
        # Rotate relative velocity vector to obtain intruder bearing
        brn     = degrees(atan2(-rx * vreln + rd * vrele,
                                 rd * vreln + rx * vrele))

        # Calculate intruder lat/lon
        aclat, aclon = geo.kwikpos(latref, lonref, brn, dist / nm)
        
        # convert groundspeed to CAS, and track to heading using actual
        # intruder position
        acspd      = sqrt(gsn*gsn + gse*gse)
        achdg      = degrees(atan2(gse, gsn))

        # Create and, when necessary, set vertical speed
        stack.stack(f'CRE {acid} {actype} {aclat} {aclon} {achdg} {altref / ft} {acspd / kts}')
        traf.ap.selaltcmd(len(traf.lat) - 1, altref, acvs)
        traf.vs[-1] = acvs
    
    @stack.command(name = "CRE_BATCH_CONF_ADSL")
    def cre_batch(self, lat: float = 52.5, lon: float = 5.3, nb_scen: int = 20):
        scenario_path = "/Users/sryhandiniputeri/bluesky/scenario/"
        filename = f'certiflight_batch_spawn_conflict_all.scn'
        fullname = f'{scenario_path}{filename}'

        if os.path.exists(fullname):
            os.remove(fullname)
        
        f = open(fullname, "a")

        scenario_number = 0

        list_use_adsl = [False]
        list_adsl_stdev = [0.0, 3.0]
        # list_hpos = [1.5, 5.0, 15.0]
        list_hpos = [15.0]
        list_dt_lookahead = [15]
        list_rpz = [30, 50]
        # list_dpsi_range = ['1', '2', '3', '4', '5', '6', '7', '8']
        list_dpsi_range = ['0']
    
        for hpos in list_hpos:
            for dt_lookahead in list_dt_lookahead:
                if(dt_lookahead < 20):
                    simtime = 3*dt_lookahead
                else:
                    simtime = 2*dt_lookahead

                minute, seconds = self.seconds_to_minutes(simtime)

                for rpz in list_rpz:
                    for dpsi_range in list_dpsi_range:
                        for i in range(int(nb_scen)):
                            created_scenarios = self.cre_scen(dt_lookahead, rpz, hpos, dpsi_range, lat, lon, i)
                            
                            for created_scen in created_scenarios:
                                scenario_number += 1
                                scen_name = f'spawn_conflict_{scenario_number}'
                                str_1 = f'0:00:00.00>SCEN {scen_name}'
                                str_2 = f'00:00:00.00>PCALL {created_scen}'
                                str_3 = f'00:00:00.00>FF'
                                str_4 = f'00:00:00>SCHEDULE 00:{minute}:{seconds} HOLD'
                                str_5 = f'00:00:00>SCHEDULE 00:{minute}:{seconds} LOGCPA {scen_name} {rpz}'
                                str_6 = f'00:00:00>SCHEDULE 00:{minute}:{seconds} DELETEALL'

                                f.write(f'{str_1}\n{str_2}\n{str_3}\n{str_4}\n{str_5}\n{str_6}\n\n')

        f.close()

        return filename

    @stack.command(name = "CRE_SCEN_ADSL")
    def cre_scen(self, dt_lookahead: int, rpz: float, hpos: float = 1.5, dpsi_range = '0',
                 lat: float = 52.5, lon: float = 5.3, index = 1):
        
        # scenario setup
        lat1 = lat
        lon1 = lon

        max_speed = 35 * kts # m/s
        dist = (6*dt_lookahead*max_speed)/1000 # in km

        rpz_in_nm = round(rpz / nm, 4)

        dpsi_dict = {'0': [10, 350],
                     '1': [10, 45], '2': [45, 90], '3': [90, 135], '4': [135, 180],
                     '5': [180, 225], '6': [225, 270], '7': [270, 315], '8': [315, 350]}

        lower_dpsi = dpsi_dict[dpsi_range][0]
        upper_dpsi = dpsi_dict[dpsi_range][1]

        # scenario saving setup
        # save as three different scenarios: 1. adsl_false, adsl_true, adsl_true_stdev

        scenario_path = f'/Users/sryhandiniputeri/bluesky/scenario/'

        filename_list = []
        filename_base = f'certiflight_dt_{dt_lookahead}_rpz_{int(rpz)}_hpos_{hpos}_dpsi_range_{dpsi_range}'

        # adsl_pair = [(0, 0.0), (1, 0.0), (1, 3.0), (1, 5.0), (1, 10.0), (1, 15.0)]
        adsl_pair = [(1, 3.0)]

        for adsl in adsl_pair:
            filename_adsl = f'_adsl_{adsl[0]}_delstdev_{adsl[1]}_{index}.scn'
            fullname_adsl = scenario_path + filename_base + filename_adsl
            filename_list.append(fullname_adsl)

            if os.path.exists(fullname_adsl):
                os.remove(fullname_adsl)

            # set for adsl_false_0
            set_detect_mode = f'00:00:00.00>ASAS DETECTADSL'
            set_use_adsl = f'00:00:00.00>DETECT_USING_ADSL {adsl[0]}'
            set_lookahead = f'00:00:00.00>DTLOOK {dt_lookahead}'
            set_rpz = f'00:00:00.00>RPZ {rpz_in_nm}'
            set_detect = f'{set_detect_mode}\n{set_use_adsl}\n{set_lookahead}\n{set_rpz}\n'

            set_adsl_1 = f'00:00:00.00>NOISE ON'
            set_adsl_2 = f'00:00:00.00>IMPL ADSB ADSL'
            set_adsl_3 = f'00:00:00.00>ADSL_HPOS_NOISE {hpos}'
            set_adsl_4 = f'00:00:00.00>ADSL_DELAY_STDEV {adsl[1]}'
            set_adsl = f'{set_adsl_1}\n{set_adsl_2}\n{set_adsl_3}\n{set_adsl_4}\n'

            
            set_reso = f'00:00:00.00>RESO MVP'

            set_log = f'00:00:00.00>LOGADSL {dt_lookahead} {rpz_in_nm} {hpos} {adsl[0]} {adsl[1]} {dpsi_range}'


            file = open(fullname_adsl, "a")
            file.write(f'{set_detect}\n{set_adsl}\n{set_reso}\n{set_log}\n')
            file.close()
    
        #######

        self.ac_counter = 0
        self.box_counter = 0

        nb_of_conflict = 5

        for j in range(nb_of_conflict):
            for i in range(nb_of_conflict):
                lat2, lon2 = self.km_to_lat_lon(lat1, lon1, dist)
                dlat = lat2 - lat1
                dlon = lon2 - lon1
                
                ac_lat = lat1 + 0.2*dlat
                ac_lon = lon1 + 0.5*dlon
                hdg = 0
                alt_def = 100
                spd = random.uniform(15, 35)

                self.ac_counter += 1
                drone_id = "DR" + "{0:04}".format(self.ac_counter)

                create_drone = f'00:00:00.00>CRE {drone_id} M600 {ac_lat} {ac_lon} {hdg} {alt_def} {spd}'

                self.ac_counter += 1
                drone_id_conf = "DR" + "{0:04}".format(self.ac_counter)
                dpsi = random.uniform(lower_dpsi, upper_dpsi)

                max_dcpa = rpz + 2*hpos
                dcpa = random.uniform(0, max_dcpa / nm) # m

                tlosh = random.uniform(dt_lookahead*0.9, dt_lookahead*1.1)
                spd_intruder = random.uniform(15, 35)

                create_conflict_drone = f'00:00:00.00>CRECONFSADSL {drone_id_conf} M600 {drone_id} {dpsi} {dcpa} {tlosh} {spd_intruder}'

                for filename in filename_list:
                    file = open(filename, "a")
                    file.write(f'{create_drone}\n{create_conflict_drone}\n')
                    file.close()

                lon1 = lon2

            lon1 = lon
            lat1 = lat2

        stack.stack(f'ECHO {len(filename_list)} scenarios created')

        return filename_list
        # return fullname_adsl_true_3
    
    def km_to_lat_lon(self,
        latitude = 52.0, longitude = 5.0, distance = 1.0
    ):
        # convert latitude and longitude to radians
        latitude = float(latitude)
        longitude = float(longitude)
        distance = float(distance)
        
        lat1 = radians(latitude)
        lon1 = radians(longitude)

        # calculate the change in latitude and longitude
        dlat = distance / self.R
        dlon = distance / (self.R * cos(lat1))

        # convert back to degrees
        lat2 = lat1 + dlat
        lon2 = lon1 + dlon

        # convert back to decimal degrees
        lat2 = degrees(lat2)
        lon2 = degrees(lon2)

        return lat2, lon2
    
    def seconds_to_minutes(self, seconds):
        minutes = seconds // 60   # Integer division to get the number of full minutes
        remaining_seconds = seconds % 60   # Modulo operator to get the number of remaining seconds
        return '{:02d}'.format(int(minutes)), '{:02d}'.format(int(remaining_seconds))