#!/usr/bin/env python
# Helper file to visualize in visdom from Nirbhay. Modified to also save windows
# between runs if desired.
#
# Usage:
# ```
#   viz.update_line(iterId, loss.data[0], 'loss', 'loss')
#   viz.update_line(iterId, runningLoss, 'loss', 'running loss')
# ```

from __future__ import print_function, division

import os
import json
import numpy as np
import visdom

class VisdomVisualize():
    def __init__(self, env_name='main', port=8000, server="http://localhost",
                 rewrite_windows=True,
                 config_file='data/visdom_config.json',
                 *args, **kwargs):
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
        if os.path.exists(self.config_file):
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
        self.env_name = env_name
        self.wins = (wins if rewrite_windows else {}) or {}

    def save_config(self):
        config = {
            'server': self.viz.server,
            'port': self.viz.port,
            'wins': self.wins
        }
        with open(self.config_file, 'w') as fd:
            json.dump(config, fd)

    def update_line(self, x, y, key, line_name, xlabel="Iterations"):
        '''
            Add or update a line plot on the visdom server self.viz
            Arguments:
                x : Scalar -> X-coordinate on plot
                y : Scalar -> Value at x
                key : Name of plot/graph
                line_name : Name of line within plot/graph
                xlabel : Label for x-axis (default: # Iterations)

            Plots are created if they don't exist, otherwise
            they are updated.
        '''
        if key in self.wins.keys():
            self.viz.line(
                X = np.array([x]),
                Y = np.array([y]),
                env = self.env_name,
                win = self.wins[key],
                name = line_name,
                update = 'append'
            )
        else:
            self.wins[key] = self.viz.line(
                X = np.array([x]),
                Y = np.array([y]),
                env = self.env_name,
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

    def update_bar(self, x, key, rownames=None, legend=None, stacked=False):
        '''
            Add or update a bar plot on the visdom server self.viz
            Arguments:
                x : Vector -> X-coordinate on plot. 2nd dimension specifies the
                    groups
                key : Name of plot/graph
                rownames: Array -> Name of the X coordinates (default: None)
                legend: Names of the groups in X (default: None)
                stacked : Whether to stack the groups in X (default: False)

            Plots are created if they don't exist, otherwise
            they are updated.
        '''
        opts = dict(
            title = key,
            stacked = stacked,
            legend = legend,
            rownames = rownames,
            marginleft   = 30,
            marginright  = 30,
            marginbottom = 30,
            margintop    = 30,
        )
        if key in self.wins.keys():
            self.viz.bar(
                X = np.array(x),
                env = self.env_name,
                win = self.wins[key],
                opts = opts
            )
        else:
            self.wins[key] = self.viz.bar(
                X = np.array(x),
                env = self.env_name,
                opts = opts
            )
