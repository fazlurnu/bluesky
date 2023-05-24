import numpy as np
from bluesky import core, stack, traf  #, settings, navdb, sim, scr, tools
from bluesky.traffic.asas import ConflictResolution

    
def init_plugin():
    config = {
        # The name of your plugin
        'plugin_name':     'ResoADSL',

        # The type of this plugin.
        'plugin_type':     'sim'
        }      
    return config  

class ResoADSL(ConflictResolution):
    def resolve(self, conf, ownship, intruder):
        newtrack = ownship.trk

        for (ac1, ac2) in conf.confpairs:
            idx1 = ownship.id.index(ac1)
            idx2 = intruder.id.index(ac2)

            newtrack[idx1] -= 0.2

        newgscapped = ownship.gs
        vscapped = ownship.vs
        alt = ownship.alt

        return newtrack, newgscapped, vscapped, alt
    
    def change_trk_if_drone_single(self, trk, ac_cat):
        if (ac_cat):
            return trk - 1
        else:
            return trk
    
    def change_trk_if_drone(self, vector_trk, vector_ac_cat):
        func = np.vectorize(self.change_trk_if_drone_single)
        return func(vector_trk, vector_ac_cat)
