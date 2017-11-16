# Demonstrations

Some demonstrations were non-standard. This README contains a list of those.

**Sid**:

- `2017-11-14-11-38-19.bag` - Unsuccessful because batteries got lost
- `2017-11-14-11-47-41.bag` - Forgot to close the drawer
- `2017-11-14-15-21-43.bag` - Crash in the simulator:
```
    0    5    10   15   20   25   30   35   40
    |    |    |    |    |    |    |    |    |
15| ::::::::::::::::::::::::::::::::::::::::: | Objects:
14| :     @@@@@                             : |  Apple: (38, 2, 0)
13| :     @@@@@       G                     : |  Batteries: (19, 8, 2)
12| :     @@@@@                             : |  Flashlight: (37, 2, 0)
11| :     @@@@@                             : |  Granola: (18, 13, 0)
10| :     @@@@@     %%%%%#######            : |  Knife: (18, 6, 0)
 9| :               %*---#######            : |
 8| :               %*[B]#######            : | Gripper: (19, 8, 2) (open)
 7| :     %%%%%     %*---#######            : |
 6| :     %+++%     %%%$$#######            : | Drawers:
 5| :     %+++%        $$                   : |  Stack center, height: (24, 8), 2
 4| :     %+++%        $$                   : |  Drawer opening: 5
 3| :     %%%%%        $$                   : |
 2| :                  $$$               FA : | Box:
 1| :                  $$$                  : |  Center, height: (8, 5), 1
 0| ::::::::::::::::::::$:::::::::::::::::::: |  Lid: (8, 12, 0)
    |    |    |    |    |    |    |    |    |
    0    5    10   15   20   25   30   35   40
Action?: m 37 1
Traceback (most recent call last):
  File "/home/banerjs/Workspaces/hlpr/src/task_sim/scripts/table_sim.py", line 1560, in <module>
    if not table_sim.getInput():
  File "/home/banerjs/Workspaces/hlpr/src/task_sim/scripts/table_sim.py", line 1546, in getInput
    self.worldUpdate(action_msg)
  File "/home/banerjs/Workspaces/hlpr/src/task_sim/scripts/table_sim.py", line 285, in worldUpdate
    self.move(action.position)
  File "/home/banerjs/Workspaces/hlpr/src/task_sim/scripts/table_sim.py", line 528, in move
    object_points = self.interpolate(object.position.x, object.position.y, object_goal.x, object_goal.y)
AttributeError: 'NoneType' object has no attribute 'x'
```

**Dave**:
2017-11-16-16-08-56.bag
