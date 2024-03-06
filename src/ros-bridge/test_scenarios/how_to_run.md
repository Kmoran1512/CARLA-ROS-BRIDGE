This is a short list of instructions on how to run a test scenario.

Begin by launching the test scenarios
`ros2 launch test_scenarios test_scenarios.launch.py`

This will run a basic scenario with 1 pedestrian in the center

## Location and Number

If you wish to change this there are several controls. If you are OK without any pedestrians moving then simple controls such as:

`ros2 launch test_scenarios test_scenarios.launch.py peds:='[1,1,1]'`

Will spawn 1 pedestrian in each lane.
Similarly the following will spawn two pedestrians in the left lane, one in the right, and none in the center. This pattern can be continued.

`ros2 launch test_scenarios test_scenarios.launch.py peds:='[2,0,1]'`

## Blueprinting

For controlling how the pedestrian model that is used

`ros2 launch test_scenarios test_scenarios.launch.py bp:=23`

OR

`ros2 launch test_scenarios test_scenarios.launch.py peds:='[2,0,1]' bps:=[1,4,42]`

In the former, all of the same pedestrian will be used for the experiment. In the latter there must be an entry for the number of pedestrians present in the scenario.
_Note_: `bps` (plural) is used in the latter `bp` (singular) is used in the former

## Movement

Using the two commands `direction` (yaw in degrees) and `speed` (m/s) you can control which pedestrians move and how quickly. Similarly, you can set a delay either based on time in seconds: `tdelay` or distance from the ego_vehicle in meters `mdelay`. So if you want a single pedesrian that walks from right to left when the ego_vehicle is 10m away it would look like the following.
_Note_: These commands can be pluralized in the same way as `bp`.

`ros2 launch test_scenarios test_scenarios.launch.py peds:='[0,0,1]' direction:=90 speed:=1.7`

Or for walkers crossing. One going left to right and the other right to left.

`ros2 launch test_scenarios test_scenarios.launch.py peds:='[1,0,1]' bps='[23,17]' directions:='[-90,90]' speeds:='[1.8,1.7]'`
