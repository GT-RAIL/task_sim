#!/usr/bin/env python
# This is the abstraction to the celery queue between HER and table_sim.
# Make sure the rabbitmq and redis are running:
#   docker run -d --name redis redis
#   docker run -d --hostname rabbitmq --name rabbitmq rabbitmq

# Celery
from celery import Celery

# Regardless of language, all the apps must remain aware of the Celery
config = dict(
    broker='pyamqp://guest:guest@172.17.0.3:5672',
    backend = 'redis://172.17.0.4:6379'
)
app = Celery('celery_executor', **config)

def setup_celery(is_worker, table_sim=None):
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
    else:
        @app.task(name='table_sim.execute_action')
        def execute(actions):
            return []

    return execute
