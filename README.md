# task-sim
Basic simulation for robot manipulation tasks

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

docker run -d --rm -v $(pwd)/../:/usr/src ros:indigo /usr/src/task_sim/bin/docker_train_rl.sh <config_file>

# Run with `-it` instead of `-d` to stay attached to the container
```

To observe the container's output:

```
docker logs <container> # To see the whole output
docker logs -f <container> # To follow the output. ctrl+c stops following
```

There is also a way to attach to the container with `docker attach` but I haven't been able to figure out a method of detaching from the container once attached.

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
change_seeds: Flag to randomize the table_sim between episodes, or list of seeds
rate: The rate of executing actions, in case you want to follow along
save_path: Folder to save the trained agent at. Relative to `task_sim` folder
save_prefix: Filename to use when saving the agent
save_suffix: More filename options. If empty (cannot be NULL), use the datetime

task: Dictionary of keyword args to initialize the task. Cannot use keyword `viz`

agent: Dictionary of keyword args to initialize the agent. Cannot use keywords `task` and `viz`
```

See `config/rl/template.yaml` for a template of the config file for learning task1 with an epsilon-greedy Q-table agent.
