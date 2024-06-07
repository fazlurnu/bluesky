import bluesky as bs
from bluesky.core import Entity, timed_function
from bluesky.stack import command
from bluesky import stack, core
from bluesky.tools import datalog
from bluesky.tools.aero import ft

import pandas as pd
import datetime

def init_plugin():
    # Configuration parameters
    config = {
        'plugin_name': 'pos_logger',
        'plugin_type': 'sim',
        'reset': reset
    }
    
    bs.traf.AdslLogger = PosLogger()
    return config

def reset():
    bs.traf.AdslLogger.reset()

class PosLogger(Entity):
    def __init__(self):
        super().__init__()
        # Create the loggers
        self.conflict_list = []
        self.los_list = []
        self.log_dir = "output"

        # Define the columns
        self.columns = ['id', 'lat', 'lon']

        # Create an empty DataFrame with the specified columns
        self.df = pd.DataFrame(columns=self.columns)
        
    def reset(self):
        # Define the columns
        self.columns = ['id', 'lat', 'lon']

        # Create an empty DataFrame with the specified columns
        self.df = pd.DataFrame(columns=self.columns)

    @core.timed_function(name="pos_tracking", dt=1.0)
    def pos_tracking(self):
        # Initial positions and parameters for 5 aircraft
        data = {
            'id': bs.traf.id,
            'lat': bs.traf.lat,
            'lon': bs.traf.lon,
        }

        # Convert initial data to a DataFrame
        data_t = pd.DataFrame(data, columns=self.columns)

        self.df = pd.concat([self.df, data_t], ignore_index = True)


    @stack.command(name="LogPos")
    def log_cpa(self, scenario_name):
        current_datetime = datetime.datetime.now()
        formatted_datetime = current_datetime.strftime("%Y_%m_%d_%H_%M_%S")

        self.df.to_csv(f'{self.log_dir}/pos_evolve_{scenario_name}_{formatted_datetime}.log')
        
        return
        