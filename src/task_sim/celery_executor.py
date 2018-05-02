#!/usr/bin/env python
# This is the abstraction to the celery queue between HER and table_sim.
# Make sure the rabbitmq and redis are running:
#   docker run -d --name redis redis
#   docker run -d --hostname rabbitmq --name rabbitmq rabbitmq

# Celery
from celery import Celery

# Regardless of language, all the apps must remain aware of the Celery
config = dict(
    broker='pyamqp://guest:guest@172.17.0.2:5672',
    backend = 'redis://172.17.0.3:6379'
)
app = Celery('celery_executor', **config)

def setup_table_sim(is_worker, table_sim=None):
    """
    Function to setup celery and return a task. If this is the simulator, then
    table_sim is used to point to the simulator and get the next state from the
    simulator. If this is the baselines code, then table_sim is none and it's
    the function wrapper that's more useful
    """

    if is_worker:
        @app.task(name='table_sim.execute_action')
        def execute(actions):
            return table_sim.execute_celery(actions)

        @app.task(name='table_sim.reset')
        def reset():
            table_sim.reset_sim(None) # Don't really need to send a request
            return True

        @app.task(name='table_sim.query_state')
        def query_state():
            return table_sim.query_state_celery()

    else:
        @app.task(name='table_sim.execute_action')
        def execute(actions):
            return []

        @app.task(name='table_sim.reset')
        def reset():
            return False

        @app.task(name='table_sim.query_state')
        def query_state():
            return []

    return {
        'execute': execute,
        'reset': reset,
        'query_state': query_state,
    }
