# Notes

Just some deprecated notes on processes to follow and things to do.

## Training RL (Deprecated)

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

## TODO (Deprecated)

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
