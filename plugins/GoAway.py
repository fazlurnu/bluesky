import numpy as np
from bluesky import core, stack, traf  #, settings, navdb, sim, scr, tools
from bluesky.traffic.asas import ConflictResolution

    
def init_plugin():
    config = {
        # The name of your plugin
        'plugin_name':     'GoAway',

        # The type of this plugin.
        'plugin_type':     'sim'
        }      
    return config  

class GoAway(ConflictResolution):
    
    def resolve(self, conf, ownship, intruder):
        dv = np.zeros((ownship.ntraf, 3))

        stack.stack(f'ECHO Ownship: {ownship.type}')
        stack.stack(f'ECHO Intrueder: {intruder.type}')

        # for (ac1, ac2) in conf.confpairs:            
        #     stack.stack(f'ECHO {ac1}: ')
        #     stack.stack(f'ECHO {ac2}: ')
        
        return hdg_new, spd_new, vs_new, alt_new

    def is_drone(self, acid):
        drone_list = ["M600", "Amzn", "Mnet", "Phan4", "M100", "M200", "Mavic", "Horsefly"]
        drone_list = [type.upper() for type in drone_list]

        return traf.type[acid] in drone_list
    
    def update(self, conf, ownship, intruder):
        ''' Perform an update step of the Conflict Resolution implementation. '''
        if ConflictResolution.selected() is not ConflictResolution:
            # Only perform CR when an actual method is selected
            if conf.confpairs:
                self.trk, self.tas, self.vs, self.alt = self.resolve(conf, ownship, intruder)
            self.resumenav(conf, ownship, intruder)