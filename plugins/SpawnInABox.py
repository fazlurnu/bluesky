import random
import string

import numpy as np
# Import the global bluesky objects. Uncomment the ones you need
from bluesky import core, stack, traf  #, settings, navdb, sim, scr, tools
from math import sin, cos, sqrt, atan2, radians, degrees

from .ADSL import ADSL

def init_plugin():
    ''' Plugin initialisation function. '''
    # Instantiate our example entity
    spawn = SpawnInABox()

    # Configuration parameters
    config = {
        # The name of your plugin
        'plugin_name':     'SpawnInABox',

        # The type of this plugin. For now, only simulation plugins are possible.
        'plugin_type':     'sim',
        }

    # init_plugin() should always return a configuration dict.

    return config

class SpawnInABox(core.Entity):
    ''' Example new entity object for BlueSky. '''
    def __init__(self):
        super().__init__()
        # All classes deriving from Entity can register lists and numpy arrays
        # that hold per-aircraft data. This way, their size is automatically
        # updated when aircraft are created or deleted in the simulation.
        with self.settrafarrays():
            self.npassengers = np.array([])
            self.adsl     = ADSL()

        # approximate radius of the Earth in km
        self.R = 6373.0
        self.box_counter = 1
        
    def create(self, n=1):
        ''' This function gets called automatically when new aircraft are created. '''
        # Don't forget to call the base class create when you reimplement this function!
        super().create(n)
        # After base creation we can change the values in our own states for the new aircraft
        self.drone_id = 'DR' + '{:04d}'.format(traf.ntraf+1)

    # @stack.command(name='CREATE_BOX')
    # def create_km_box(self, lat1, lon1, dist):

    @stack.command(name='SPAWN_IN_BOX')
    def create_km_box(self, lat1: float = 52, lon1: float = 5, dist: float = 10):
        # lat1 = float(lat1)
        # lon1 = float(lon1)
        # dist = float(dist)

        lat2, lon2 = self.km_to_lat_lon(lat1, lon1, dist)

        box_id = 'B' + '{:04d}'.format(self.box_counter)
        stack.stack(f'BOX {box_id} {lat1} {lon1} {lat2} {lon2}')
        stack.stack(f'AREA {box_id}')
        self.box_counter += 1
        
        ac_lat = random.uniform(lat1, lat2)
        ac_lon = random.uniform(lon1, lon2)
        hdg = random.uniform(0, 359)
        alt_def = 100
        spd = random.uniform(10, 30)
        drone_id = self.get_random_drone_id()
        stack.stack(f'CRE {drone_id} M600 {ac_lat} {ac_lon} {hdg} {alt_def} {spd}')

        ac_lat = random.uniform(lat1, lat2)
        ac_lon = random.uniform(lon1, lon2)
        hdg = random.uniform(0, 359)
        alt_def = 100
        spd = random.uniform(10, 30)
        drone_id = self.get_random_drone_id()
        stack.stack(f'CRE {drone_id} M600 {ac_lat} {ac_lon} {hdg} {alt_def} {spd}')
                    
        return True

    @stack.command(name='ECHO_ADSL')
    def echo_adsl(self, acid: 'acid'):
        s7 = f'HPOS_ACC: {self.adsl.hpos_acc[acid]}'

        stack.stack(f'ECHO {s7}')

        return True

    def get_random_drone_id(self):
        # Generate two random letters
        letters = ''.join(random.choices(string.ascii_uppercase, k=2))

        # Generate four random digits
        digits = ''.join(random.choices(string.digits, k=4))

        # Combine the letters and digits into a string
        return f"{letters}{digits}"

    def spawn_many(self, lat_default, lon_default, dist, N):
        lat1 = lat_default
        lon1 = lon_default
        counter = 1

        for i in range(N):
            for j in range(N):
                box_id = 'B' + '{:04d}'.format(counter)
                drone_id = 'DR' + '{:04d}'.format(counter)
                counter += 1
                lat2, lon2 = self.km_to_lat_lon(lat1, lon1, dist)

                ac_lat = uniform(lat1, lat2)
                ac_lon = uniform(lon1, lon2)

                hdg = uniform(0, 359)
                alt_def = 100
                spd = uniform(10, 30)

                stack.stack(f'BOX {box_id} {lat1} {lon1} {lat2} {lon2}')
                stack.stack(f'CRE {drone_id} M600 {ac_lat} {ac_lon} {hdg} {alt_def} {spd}')

                lon1 = lon2

            lat1 = lat2
            lon1 = lon_default    


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
