## objection
This code aims at compare slam output and groundtruth(in fact, it is estimated ground truth from `state_groundtruth_estimate0` folder) from euroc dataset.
After the slam gets poses of each keyframe, this package will try to find which groundtruth's timestamp is the same as each slam keyframe pose's timestamp(timestamp difference smaller than 1ms will be seen as the same timestamp, you can set it in the program by yourself). 
After it finds all the poses that matching, it will do 3d registration according to 

http://graphics.stanford.edu/courses/cs348a-17-winter/Papers/quaternion.pdf
5  Application: 3-D Shape Registration

then the slam pose and groundtruth pose will be published to rviz. The slam pose is in red color and the groundtruth pose is in blue color.

at the same time, in the terminal you can see the transformation matrix and the RMSE error of slam and ground truth.

## how to run
1: put this package in your ros_workspace/src and then use catkin_make to compile as usual package
2: make your slam can publish geometry_msgs::PoseStamped message when it gets each keyframe pose.(for example, when your slam system finishes or almost finish, you extract all the poses into a vector) set the topic of the publisher as "keyframe_pose". <br>
3: run rviz. use `rosrun rviz rviz`. set `fix_keyframe` as `my_frame` then press `Add` button , choose `MarkerArrary` and press `OK`. <br>
4: run this package `roslaunch data_registration run_regi.launch` <br>
5: run your slam system

## note
1:You'd better stop your own slam system's visualizer before using the rviz. <br>
2:You need modify after how many keyframes you want to do registration in `run_regi.launch` by yourself. The variable is called `do_registration_threshold` <br>
3:You need specify the location of your ground truth csv file in `run_regi.launch`. <br>
4:You can choose not to visualize the poses in rviz if you just want to see RMSE error by setting the variable `visualization_choice_` to `visualize` or something else.

# slam_groundtruth_comparison
used to compare slam pose output and ground truth. ROS needed

