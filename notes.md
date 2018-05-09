# Notes

Just some random notes on what needs to be done in order to evaluate the algorithms on all the conditions.

Parameters:

```
TableSim:
    seed - pick the simulation from the dataset of training/testing simulations
    objects/complexity - pick whether to use the demo env/execution env

LearnTransitionFunction:
    simulator - which table_sim instance to use
    amdp_id - open,close (0,1;6,7) / put in (2;8)
    classifier_name/action_bias - method to use (Need to refine/define)
    task - drawer (4) / box (7). Used for the demos

AMDPNode:
    experiment - the mode in which to run. Probably constant
    execution_policy - How to deal with value function and unknowns

AMDPValueIteration:
    <none>
```

Architecture:

1. Spin up 5 table sim instances - each with different values of complexity based on the needs of that instance. 1 instance, `eval_sim` - complexity: 1, evaluate the performance of the transition function and AMDP; 2 instances, `drawer_sim1` & `drawer_sim2` - complexity: 0, learn the transitions for open/close and put in respectively; `box_sim1` & `box_sim2` - complexity: 0, learn the transitions for open/close and put in respectively.
1. There are 4 `LearnTransitionFunction` executors associated with each of the learning nodes. After an epoch of training, 4 `AMDPValueIteration` nodes are used update the value functions. Finally there is 1 `AMDPNode` that takes the combined value functions from the learning nodes to execute.
1. Use h5py to save the transition functions and the utilities table?
