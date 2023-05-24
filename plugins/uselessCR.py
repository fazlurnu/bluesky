import numpy as np
from bluesky import core, stack, traf  #, settings, navdb, sim, scr, tools
from bluesky.traffic.asas import ConflictResolution

    
def init_plugin():
    config = {
        # The name of your plugin
        'plugin_name':     'uselessCR',

        # The type of this plugin.
        'plugin_type':     'sim'
        }      
    return config  

class uselessCR(ConflictResolution):
    def resolve(self, conf, ownship, intruder):
        hdg_new = ownship.ap.trk
        spd_new = ownship.ap.tas
        vs_new = ownship.ap.vs
        alt_new = ownship.ap.alt
        stack.stack("PLUGIN LOAD CITYROUTE")

        for pair in conf.confpairs:
            #Resolve for ownship only
            self.uselessCRreso(pair)

        return hdg_new, spd_new, vs_new, alt_new

    
    def uselessCRreso(self, pair):
        # Send the ownship to brazil
        stack.stack(f'GOTOBRAZIL {pair[0]}')
        return