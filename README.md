## Functions
In the script *wheelencoder.py*, function `calibration(node)` is used to do wheel radius calibration. To start with, you need to slightly move the robot so that the node can get the initial tick number. Then put the duckiebot at the start point and enter 's'. After driving for a certian distance, enter 'f' to finish calibration and enter the distance from start to finish position(in meter). The sript will automatically output the wheel radius.

You should delete line `calibration(node)` in the main function if you don't need to do calibration.

## Execution
1.Build docker image on duckiebot

Run command

`dts devel build -f -H <robot_name>.local`

Replace `<robot_name>` with name of your duckiebot.

2.Start docker container

Run command

`dts devel run -H <robot_name>.local`

Replace `<robot_name>` with name of your duckiebot.

