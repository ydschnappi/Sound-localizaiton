## Package `Planner`

Contributor: Dian Yuan

#### Content

This package contains two node `planner` and `visualization`.

- In the `planner` node it subscribe signal information from the `signal_generate_node`, and very basic [TDoA](https://en.wikipedia.org/wiki/Time_of_arrival)(Time Difference of Arrival) algorithm is implemented to locate the sound source. A state machine and a series of functions are created to navigate the car to the source position. Based on the sound strength this node also plays a role to check if the sound source has been found.
- In the `visualization` the exact sound position is published to rviz with `marker` (of course it is unknown to the car). In this way, you could observe the position of the sound source more intuitively on the rviz interface.

#### Known issues

- Because of the accurancy of the estimated pose from SLAM node, some time the source position estimation would have relative large error. In this case the robot would try to relocate the signal source. But if localization part (in SLAM) still publish pose with large error(especially orientation), the sound source can never been located and this mission would fail.
- Real-time sound locating are not supported yet, so the locating part and navigate part are done idividually at this moment.
