# task-sim
Basic simulation for robot manipulation tasks.

We also include supporting code for training and evaluating demonstration-guided relation-based AMDP task learning, as published in Humanoids 2018 (publication forthcoming).  For details on where this code sits and how to run it, please skip to the [AMDP Training and Evaluation](#amdp-training-and-evaluation) section below.

## Adding a new action selector
1. Implement an action selection service on the topic `/table_sim/select_action` of type SelectAction (takes a State and returns an Action)
1. Implement a status service on the topic `/table_sim/query_status` of type QueryStatus (takes a State and returns a Status, the status is a code indicating whether the state is terminal (COMPLETED or FAILED), IN_PROGRESS (should continue executing normally), or if an intervention is needed)
   * This'll probably be very similar (or the same) as what's already implemented in `classifier_node.py`.  I left this within each node in case an approach needs a unique way of determining when an intervention is needed.
1. That's it; see `classifier_node.py` and `random_action.py` for examples.

## Running trials
1. Start the table simulator: `rosrun task_sim table_sim.py`.
   * To suppress input (when not gathering demonstrations), disable the `terminal_input` flag
   * To suppress output, enable the `quiet_mode` flag
   * For consistency, the initial state can be optionally seeded with the `seed` parameter
1. Start your action selector node
1. Start the executor node: `rosrun task_sim executor.py`.
   * Interventions can be enabled/disabled with the `allow_interventions` flag
   * Execution can be run in batch mode by setting the number of trials with the `trials` parameter
1. If you're using interventions, they will be performed in the `table_sim` terminal (make sure you are not in quiet mode so that the state can be seen!).
1. A report with metrics will print out at the end in the `executor` terminal.

## AMDP Training and Evaluation
The following documentation describes the work published in Humanoids 2018 (publication forthcoming).

### Running experiments
Each of the conditions given in the paper can be evaluated over multiple episodes by running the either `launch/train_amdp.launch` (for SC, AC, SC+AC, and no-training baselines (set the `baseline_mode` arg to true for this)), or `launch/train_amdp_q_learning` (for Q-learning variants).

Details of how the demonstrations are used to bias exploration can be found in `scripts/amdp_trainer.py`.  Exploration can use any combination of the following, by setting its associated parameter (listed in parentheses) to `True`:  
 * RANDOM (`~demo_mode/random`) - select a random action
 * SHADOW (`~demo_mode/shadow`) - repeat an action exactly as shown in the demonstrations
 * CLASSIFIER (`~demo_mode/classifier`) - select an action returned by the state-centric classifier (SC)
 * PLAN_NETWORK (`~demo_mode/plan_network`) - select an action returned by the action-centric plan network (AC)

### Collecting demonstrations and retraining exploration biasing models
To collect new demonstrations, simply run the `scripts/table_sim.py` node with terminal input while recording the `~/task_log` topic with rosbag:  

```
rosbag record table_sim/task_log
rosrun task_sim table_sim.py _quiet_mode:=False _terminal_input:=True _complexity:=0 _env_type:=0 _seed:=None
```

Make sure to set `env_type` to the desired environment (0 for a drawer task, 1 for a box task) and set `seed` to an appropriate integer if you are using fixed seeds for training environments.

Save the .bag files in `data/task<n>/demos`, as shown in the included data folder.

Once demos are collected, new classifiers can be trained using `scripts/train_amdp_classifier.py` for each AMDP (set the classifier type with the `classifier_types` parameter, the name of the task directory where demos are saved with the `task` parameter, and the AMDP to train a model for with the `amdp_id` parameter).  Similarly, new plan networks can be trained using `scripts/train_plan_network.py` (setting the `task` parameter appropriately).
