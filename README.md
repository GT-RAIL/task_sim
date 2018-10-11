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

## Training RL

Running RL (detached) with docker

```
# $(pwd) is used assuming that you are running this from the `task_sim` dir
# <config_file> is the file path relative to the `task_sim` directory
# Change /root to the $TASKSIM_FILES_ROOT if the image build args were different
# Update <name> by setting it as desired

docker run --runtime=nvidia -d --rm -v $(pwd)/../:/root/software/ --name <name> gt-rail/task_sim:rl /root/software/task_sim/bin/train_rl.sh <config_file>

# Run with `-it` instead of `-d` to stay attached to the container
```

To observe the container's output:

```
docker logs <container> # To see the whole output
docker logs -f <container> # To follow the output. ctrl+c stops following
```

There is also a way to attach to the container with `docker attach` but I haven't been able to figure out a method of detaching from the container once attached.

### Building Docker Image

There is a docker image at `bin/docker/rl` that can be used to run an isolated instance of the RL training code. Run the following command from the top-level `task_sim` directory. The output is an image with the tag `gt-rail/task_sim:rl`

```
docker build -t gt-rail/task_sim:rl -f bin/docker/rl/Dockerfile .
```

The Dockerfile takes the additional arg of `$TASKSIM_FILES_ROOT` that is by default set to the directory of `/root` in the target image. You can change this if you so desire.

### Config file

The launch file used to train requires the trainer parameters be specified in a config file. By convention, these config files live in `config/rl`. The parameters expected in the file are:

```
visdom_config: Dictionary of configs for the visdom interface
  visualize: Master flag to turn visualization on or off
  plot_frequency: Number of episodes after which to plot the data
  config_file: Location of JSON file with server, port, and window names
  env_name: Visdom environment name
  rewrite_windows: Flag to ignore window names in JSON file

execute_post_episode: Test policy after these many episodes
num_episodes: Number of training episodes to run
change_seeds: Boolean to randomize the table_sim between episodes, or list of seeds
rate: The rate of executing actions, in case you want to follow along
save_path: Folder to save the trained agent at. Relative to `task_sim` folder
save_prefix: Filename to use when saving the agent
save_suffix: More filename options. If empty (cannot be NULL), use the datetime
save_every: Number of episodes after which to save the agent

task: Dictionary of keyword args to initialize the task. Cannot use keyword `viz`

agent: Dictionary of keyword args to initialize the agent. Cannot use keywords `task` and `viz`
```

See `config/rl/template.yaml` for a template of the config file for learning task1 with an epsilon-greedy Q-table agent.


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

