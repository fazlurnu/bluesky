''' State-based conflict detection. '''
import numpy as np
from bluesky import stack, settings, traf
from bluesky.tools import geo
from bluesky.tools.aero import nm, ft
from bluesky.traffic.asas import ConflictDetection

from math import radians, degrees, cos, sin, sqrt

from .ADSL import ADSL

def init_plugin():
    ''' Plugin initialisation function. '''

    # Configuration parameters
    config = {
        # The name of your plugin
        'plugin_name':     'DetectCR',

        # The type of this plugin. For now, only simulation plugins are possible.
        'plugin_type':     'sim',
        }

    # init_plugin() should always return a configuration dict.
    return config
