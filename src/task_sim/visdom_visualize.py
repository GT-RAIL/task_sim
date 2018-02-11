#!/usr/bin/env python
# Helper file to visualize in visdom from Nirbhay. Modified to also save windows
# between runs if desired. WARNING: updateTrace will be deprecated in future
# releases of visdom.
#
# Usage:
# ```
#   viz.append_data(iterId, loss.data[0], 'loss', 'loss')
#   viz.append_data(iterId, runningLoss, 'loss', 'running loss')
# ```

import os.path as pth
import json
import numpy as np
import visdom

class VisdomVisualize():
    def __init__(self, env_name='main', port=8000, server="http://localhost",
                 rewrite_windows=True,
                 config_file='data/visdom_config.json'):
        '''
            Initialize a visdom server on $server:$port. If `rewrite_windows`,
            then the window names will be reinitialized before every run

            Override port and server using the local configuration from
            the json file at $config_file (containing a dict with optional
            keys 'server' and 'port').
        '''
        print("Initializing visdom env [%s]"%env_name)
        wins = None

        self.config_file = config_file
        if pth.exists(self.config_file):
            with open(self.config_file, 'r') as f:
                config = json.load(f)
                if 'server' in config:
                    server = config['server']
                if 'port' in config:
                    port = int(config['port'])
                if 'wins' in config:
                    wins = config['wins']

        self.viz = visdom.Visdom(
            port = port,
            env = env_name,
            server = server,
        )
        self.wins = (wins if rewrite_windows else {}) or {}

    def save_config(self):
        config = {
            'server': self.viz.server,
            'port': self.viz.port,
            'wins': self.wins
        }
        with open(self.config_file, 'w') as fd:
            json.dump(config, fd)

    def append_data(self, x, y, key, line_name, xlabel="Iterations"):
        '''
            Add or update a plot on the visdom server self.viz
            Argumens:
                x : Scalar -> X-coordinate on plot
                y : Scalar -> Value at x
                key : Name of plot/graph
                line_name : Name of line within plot/graph
                xlabel : Label for x-axis (default: # Iterations)

            Plots and lines are created if they don't exist, otherwise
            they are updated.
        '''
        if key in self.wins.keys():
            self.viz.updateTrace(
                X = np.array([x]),
                Y = np.array([y]),
                win = self.wins[key],
                name = line_name
            )
        else:
            self.wins[key] = self.viz.line(
                X = np.array([x]),
                Y = np.array([y]),
                opts = dict(
                    xlabel = xlabel,
                    ylabel = key,
                    title = key,
                    marginleft   = 30,
                    marginright  = 30,
                    marginbottom = 30,
                    margintop    = 30,
                    legend = [line_name]
                )
            )
