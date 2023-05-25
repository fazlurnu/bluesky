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

    @stack.command(name='SPAWN_CONF_LOOKAHEAD')
    def spawn_conf_lookahead(self, lat_input: float = 52, lon_input: float = 5, lookahead: float = 100):
        lat1 = lat_input
        lon1 = lon_input

        max_speed = 25 * kts # m/s
        dist = (2*lookahead*max_speed)/1000

        for j in range(20):
            for i in range(20):
                lat2, lon2 = self.km_to_lat_lon(lat1, lon1, dist)
                dlat = lat2 - lat1
                dlon = lon2 - lon1

                # box_id = 'B' + '{:04d}'.format(self.box_counter)
                # stack.stack(f'BOX {box_id} {lat1} {lon1} {lat2} {lon2}')
                # self.box_counter += 1
                
                ac_lat = lat1 + 0.1*dlat
                ac_lon = lon1 + 0.5*dlon
                hdg = 0
                alt_def = 100
                spd = 25

                self.ac_counter += 1
                drone_id = "DR" + "{0:03}".format(self.ac_counter)

                stack.stack(f'CRE {drone_id} M600 {ac_lat} {ac_lon} {hdg} {alt_def} {spd}')

                self.ac_counter += 1
                drone_id_conf = "DR" + "{0:03}".format(self.ac_counter)
                dpsi = random.uniform(10, 350)
                dcpa = random.uniform(0, 30 / nm) # m
                tlosh = random.uniform(lookahead*0.9, lookahead*1.1)
                # spd_intruder = random.uniform(15, 35)

                stack.stack(f'CRECONFSADSL {drone_id_conf} M600 {drone_id} {dpsi} {dcpa} {tlosh} {spd}')

                lon1 = lon2

            lon1 = lon_input
            lat1 = lat2
        
        return True
    
    @stack.command(name = "CRE_BATCH_CONF_ADSL")
    def cre_batch(self, lat: float = 52.5, lon: float = 5.3, nb_scen: int = 20):
        scenario_path = "/Users/sryhandiniputeri/bluesky/scenario/"
        filename = f'certiflight_batch_spawn_conflict_all.scn'
        fullname = f'{scenario_path}{filename}'

        if os.path.exists(fullname):
            os.remove(fullname)
        
        f = open(fullname, "a")

        scenario_number = 0

        # for hpos in [1.5, 5, 15]:
        for use_adsl in [True]:
            for adsl_stdev in [0.0, 3.0]:
                for hpos in [1.5, 5, 15]:
                    for dt_lookahead in [6, 15, 50, 100]:
                        simtime = 3*dt_lookahead

                        minute, seconds = self.seconds_to_minutes(simtime)

                        for rpz in [30, 50]:
                            for i in range(int(nb_scen)):
                                scenario_number += 1
                                scen_name = f'Spawn_conflict_{scenario_number}'
                                created_scen = self.cre_scen(dt_lookahead, rpz, hpos, use_adsl, adsl_stdev, lat, lon, i)
                                str_1 = f'0:00:00.00>SCEN {scen_name}'
                                str_2 = f'00:00:00.00>PCALL {created_scen}'
                                str_3 = f'# 00:00:00.00>FF'
                                str_4 = f'00:00:00>SCHEDULE 00:{minute}:{seconds} HOLD'
                                str_5 = f'00:00:00>SCHEDULE 00:{minute}:{seconds} LOGCPA {scen_name} {rpz}'
                                str_6 = f'00:00:00>SCHEDULE 00:{minute}:{seconds} DELETEALL'

                                f.write(f'{str_1}\n{str_2}\n{str_3}\n{str_4}\n{str_5}\n{str_6}\n\n')

        f.close()

        return filename

        

    @stack.command(name = "CRE_SCEN_ADSL")
    def cre_scen(self, dt_lookahead: int, rpz: float, hpos: float = 1.5,
                 use_adsl: bool = True, adsl_stdev: float = 3.0,
                 lat: float = 52.5, lon: float = 5.3, index = 1):
        lat1 = lat
        lon1 = lon

        max_speed = 35 * kts # m/s
        dist = (4*dt_lookahead*max_speed)/1000 # in km

        rpz_in_nm = round(rpz / nm, 4)

        scenario_path = "/Users/sryhandiniputeri/bluesky/scenario/"
        filename = f'certiflight_spawn_conflict_dt_{dt_lookahead}_rpz_{int(rpz)}_hpos_{hpos}_adsl_{use_adsl}_delstdev_{adsl_stdev}_{index}.scn'
        fullname = f'{scenario_path}{filename}'

        if os.path.exists(fullname):
            os.remove(fullname)

        str_1 = f'00:00:00.00>NOISE ON'
        str_2 = f'00:00:00.00>IMPL ADSB ADSL'
        str_3 = f'00:00:00.00>ADSL_HPOS_NOISE {hpos}'
        str_4 = f'00:00:00.00>ADSL_DELAY_STDEV {adsl_stdev}'
        str_5 = f'00:00:00.00>ASAS DETECTADSL'
        str_6 = f'00:00:00.00>DETECT_USING_ADSL {use_adsl}'
        str_7 = f'00:00:00.00>RESO MVP'
        str_8 = f'00:00:00.00>PAN {lat} {lon}'
        str_9 = f'# 00:00:00.00>++++++++++++++'
        str_10 = f'00:00:00.00>DTLOOK {dt_lookahead}'
        str_11 = f'00:00:00.00>RPZ {rpz_in_nm}'
        str_12 = f'00:00:00.00>LOGADSL {dt_lookahead} {rpz_in_nm} {hpos} {use_adsl} {adsl_stdev}'

        f = open(fullname, "a")
        f.write(f'{str_1}\n{str_2}\n{str_3}\n{str_4}\n{str_5}\n{str_6}\n{str_7}\n{str_8}\n{str_9}\n{str_10}\n{str_11}\n{str_12}\n')
        
        self.ac_counter = 0
        self.box_counter = 0

        for j in range(20):
            for i in range(20):
                lat2, lon2 = self.km_to_lat_lon(lat1, lon1, dist)
                dlat = lat2 - lat1
                dlon = lon2 - lon1

                # box_id = 'B' + '{:04d}'.format(self.box_counter)
                # str_11 = f'00:00:00.00>BOX {box_id} {lat1} {lon1} {lat2} {lon2}'
                # self.box_counter += 1
                
                ac_lat = lat1 + 0.2*dlat
                ac_lon = lon1 + 0.5*dlon
                hdg = 0
                alt_def = 100
                spd = random.uniform(15, 35)

                self.ac_counter += 1
                drone_id = "DR" + "{0:04}".format(self.ac_counter)

                str_13 = f'00:00:00.00>CRE {drone_id} M600 {ac_lat} {ac_lon} {hdg} {alt_def} {spd}'

                self.ac_counter += 1
                drone_id_conf = "DR" + "{0:04}".format(self.ac_counter)
                dpsi = random.uniform(10, 350)

                max_dcpa = rpz + 2*hpos
                dcpa = random.uniform(0, max_dcpa / nm) # m

                tlosh = random.uniform(dt_lookahead*0.9, dt_lookahead*1.1)
                spd_intruder = random.uniform(15, 35)

                str_14 = f'00:00:00.00>CRECONFSADSL {drone_id_conf} M600 {drone_id} {dpsi} {dcpa} {tlosh} {spd_intruder}'
                f.write(f'{str_13}\n{str_14}\n')

                lon1 = lon2

            lon1 = lon
            lat1 = lat2
            
        
        f.close()

        stack.stack(f'ECHO Scenario Created {filename}')

        return filename
    
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