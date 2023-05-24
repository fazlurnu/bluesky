import numpy as np
from bluesky import core, stack, traf  #, settings, navdb, sim, scr, tools
from bluesky.traffic.asas import ConflictResolution

    
def init_plugin():
    config = {
        # The name of your plugin
        'plugin_name':     'ImplicitVO',

        # The type of this plugin.
        'plugin_type':     'sim'
        }      
    return config  

class ImplicitVO(ConflictResolution):
    # Create the priority rules here
    def applyprio(self, cat_own, cat_int, stat_own, stat_int, pos_own, pos_int, v_own, v_int):
        drone_list = ["M600", "Amzn", "Mnet", "Phan4", "M100", "M200", "Mavic", "Horsefly"]
        prio = np.zero(2)

        own_is_drone = cat_own in drone_list
        int_is_drone = cat_int in drone_list

        

            
    def resolve(self, conf, ownship, intruder):
        drone_list = ["M600", "Amzn", "Mnet", "Phan4", "M100", "M200", "Mavic", "Horsefly"]
        
        prio = np.zeros(ownship.ntraf)
        hdg_new = ownship.ap.trk
        spd_new = ownship.ap.tas
        vs_new = ownship.ap.vs
        alt_new = ownship.ap.alt

        for idx in range(ownship.ntraf):
            if(ownship.type[idx] not in drone_list):
                hdg_new[idx] -= 1

        print(ownship.type, intruder.type, conf.confpairs)

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