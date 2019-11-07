
## objection
This code aims at compare slam output and groundtruth(in fact, it is estimated ground truth from `state_groundtruth_estimate0` folder) from different dataset.
After the slam gets poses of each keyframe, this package will try to find which groundtruth's timestamp is the same as each slam keyframe pose's timestamp(timestamp difference smaller than 3ms will be seen as the same timestamp, you can set it in the program by yourself in `find_same_timestamp.h`, value `same_timestamp_threshold_` ). 
After it finds all the poses that matching, it will do 3d registration according to 

http://graphics.stanford.edu/courses/cs348a-17-winter/Papers/quaternion.pdf
5  Application: 3-D Shape Registration

then the slam pose and groundtruth pose will be published to rviz. The slam pose is in red color and the groundtruth pose is in blue color.

at the same time, in the terminal you can see the transformation matrix and the RMSE error of slam and ground truth.

## how to run
1: put this package in your ros_workspace/src and then use catkin_make to compile as usual package <br>
2: make your slam can publish geometry_msgs::PoseStamped message when it gets each keyframe pose.(for example, when your slam system finishes or almost finish, you extract all the poses into a vector) set the topic of the publisher name in the launch file `sub_topic_name_`. <br>
3: run rviz configuration in this package. `roslaunch data_registration run_rviz.launch`. <br>
4: run this package `roslaunch data_registration run_regi.launch` <br>
5: run your slam system

## note
1:You'd better stop your own slam system's visualizer before using the rviz. <br>
2:You can modify after how many keyframes you want to do registration in `run_regi.launch` by yourself. The variable is called `do_registration_threshold`. If this number is smaller than 1 or if this number if too large, for example, you want to do registration after getting 1000 pose message but your slam system only publishes 900, then the system will do registration after it didn't receive slam message more than 8 seconds. <br>
3:You need specify the location of your ground truth csv file in the launch file. <br>
4:You can choose not to visualize the poses in rviz if you just want to see RMSE error by setting the variable `visualization_choice_` to `visualize` or something else.<br>
5:If there are some keyframes that don't have corresponding groundtruth, don't worry, they'll be skiped

## result example
I run my lab's slam system using euroc dataset and compare it with the euroc estimated ground truth
![alt text](https://github.com/zhaozhongch/slam_groundtruth_comparison/blob/master/result_example/example.png)
![alt text](https://github.com/zhaozhongch/slam_groundtruth_comparison/blob/master/result_example/print_result.png)

