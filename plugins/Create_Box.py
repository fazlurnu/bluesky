from random import randint
import numpy as np
# Import the global bluesky objects. Uncomment the ones you need
from bluesky import core, stack, traf  #, settings, navdb, sim, scr, tools
from math import sin, cos, sqrt, atan2, radians, degrees

def init_plugin():
    ''' Plugin initialisation function. '''
    # Instantiate our example entity
    create_box = Create_Box()

    # Configuration parameters
    config = {
        # The name of your plugin
        'plugin_name':     'CREATE_BOX',

        # The type of this plugin. For now, only simulation plugins are possible.
        'plugin_type':     'sim',
        }

    # init_plugin() should always return a configuration dict.
    return config

class Create_Box(core.Entity):
    ''' Example new entity object for BlueSky. '''
    def __init__(self):
        super().__init__()
        # All classes deriving from Entity can register lists and numpy arrays
        # that hold per-aircraft data. This way, their size is automatically
        # updated when aircraft are created or deleted in the simulation.
        with self.settrafarrays():
            self.npassengers = np.array([])

        # approximate radius of the Earth in km
        self.R = 6373.0

    @stack.command(name='CREATE_KM_BOX')
    def create_km_box(self, lat, lon, distance):
        lat2, lon2 = self.km_to_lat_lon(lat, lon, distance)

        stack.stack(f'BOX BOX0001 {lat} {lon} {lat2} {lon2}')
                    
        return True

    @stack.command(name='CREATE_KM_NXN_BOX')
    def create_km_box(self, lat, lon, distance, N):
        counter = 1
        N = int(N)
        lat_default = lat
        lon_default = lon

        for i in range(N):
            for j in range(N):
                box_id = '{:04d}'.format(counter)
                counter += 1
                lat2, lon2 = self.km_to_lat_lon(lat, lon, distance)
                stack.stack(f'BOX {box_id} {lat} {lon} {lat2} {lon2}')
                lon = lon2
            lat = lat2
            lon = lon_default
                    
        return True

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
