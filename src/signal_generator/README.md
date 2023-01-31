# signal_generator package  
Contributor: Yan Wang
## Purpose  

The goal of the signal generator package is to generate signals and publish signal info generated from our "virtual sensors "on the TAS car. So the planner package can transform the signal info to the estimated signal position and then let the TAS car navigate to the signal position.

## Package information  

In the **/signal_pose_node**, we declared three parameters representing the coordinates of the signal source and published the true signal source position in topic **/signal_pose**.   

In the **/signal_publisher_node**, we subscribed to the **/map** and **/base_footprint** topics and then got the current position of the TAS car through the tf transformation between **/map** and **/base_footprint** topics.  Besides, we also subscribed the **/signal_pose** to get the true signal position, so that we can calculte the distance between signal source and the sensors.

Since we did not find the appropriate sensor plug-in from the gazebo, we used "virtual sensors ": we assume that we have two sensors on both the right and left side of the TAS car, and each sensor is 0.2m away from our TAS car. Moreover, the sensors can detect a signal strength that is linearly related to the distance between them and the signal sources. So, we calculated the distance between the sensors and the signal source and then computed the signal info based on the distances.   

The **/signal_publisher_node** will publish the **/signal_info** topic for the planner package to subscribe and then estimate the signal source position.  

## What we have achieved  

The planner can estimate the signal position and publish the estimated coordinate to the navigator.  
We can now manually create a set of signal source points, and once the TAS car has reached one source point, the signal generator will automatically publish the next signal source.  

## The problems that have not been solved yet  

We want to let the signal generator generate a random new signal source that is reachable on the map after the TAS car reached the previous source point. The random new signal source generating part is already finished: in the **/signal_pose_node**, we directly read the .pgm file of the map, and the feasible area should have a value of 254. So, we randomly choose the coordinate of a pixel with a value of 254 as the new signal souce point once we reach the previous one.  
The problem is that the map size is very small at the beginning. The TAS car can reach the area on the map that is still unknown, but it still cannot be navigated to the coordinate outside the range of the map. So, if we randomly generate a coordinate exceeding the size of the current map, the TAS car cannot reach the signal source.  
In order to fix this, we tried to publish in **/signal_pose_node** an intermediate waypoint and let the TAS car follow the intermediate waypoint to explore, thus increasing the map's size, until the signal source is included in the range of the map. We have finished the implementation of sending the intermediate point in the signal_generator package, but due to the time limit, we have not tested it yet. Our idea is to choose the farthest point on the current map, the four corners, and then select the corner closest to the estimated signal source point as our intermediate waypoint.
