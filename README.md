# task-sim
Basic simulation for robot manipulation tasks

## Adding a new action selector
1. Implement an action selection service on the topic `/table_sim/query_status` of type SelectAction (takes a State and returns an Action)
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
