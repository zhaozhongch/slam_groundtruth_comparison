
## objection
This code aims at compare slam output and groundtruth from different datasets.
After the slam gets poses of each keyframe, this package will try to find which groundtruth's timestamp is the same(or similar) as each slam keyframe pose's timestamp.
After it finds all the poses that matching, it will do 3d registration according to 

http://graphics.stanford.edu/courses/cs348a-17-winter/Papers/quaternion.pdf
5  Application: 3-D Shape Registration

then the slam pose and groundtruth pose will be published to rviz. The slam pose is in red color and the groundtruth pose is in blue color.

at the same time, in the terminal you can see the transformation matrix and the RMSE error of slam and ground truth.

## how to run
1: put this package in your ros_workspace/src and then use catkin_make to compile as usual package <br>
2: (This part needs your effort) Make your slam can publish geometry_msgs::PoseStamped message when it gets pose update. Set the topic of the publisher name in the launch file `sub_topic_name_` so that it matches the poses' topic name of your slam system publishes. <br>
3: run rviz configuration in this package. `roslaunch data_registration run_rviz.launch`. <br>
4: run this package `roslaunch data_registration run_regi_mh3.launch` (if your dataset is euroc MachineHall 3 for example) <br>
5: run your slam system

## paramters in launch file
1: You need specify the location of your ground truth csv/txt file in the launch file by changing `groundtruth_csv_file_address_`. <br>
2: You need specify the format of your groundtruth format. Only supprt euroc format and tum format. The format of euroc dataset's groundtruth is 
```
timestamp(in nano second) positonX postionY postionZ QuaternionW QuaternionX QuaternionY QuaternionZ
```
The format of tum dataset's groundtruth is 
```
timestamp(in second) positonX postionY postionZ QuaternionX QuaternionY QuaternionZ QuaternionW
```
3: (No need to change) You can modify after how many poses you want to do registration in launch file by yourself. The variable is called `do_registration_threshold_`. If this number is smaller than 1 or if this number if too large, for example, you want to do registration after getting 1000 pose message but your slam system only publishes 900 at the very end, then the system will do registration after it didn't receive slam message more than 10 seconds. <br>
4: (No need to change) The parameter `same_timestamp_threshold_` is used to locate which two poses(one from slam one from the groundtruth) are seens having the same timestamp. The timestamp from slam may never be able to match the totally same timestamp as the groundtruth. The default value of is `0.003` which means if two poses' timestamp is smaller than 3ms they will match.
5: If there are some poses from the slam that don't have corresponding groundtruth, don't worry, they'll be skiped

## result example
I run my lab's slam system using euroc dataset and compare it with the euroc estimated ground truth
![alt text](https://github.com/zhaozhongch/slam_groundtruth_comparison/blob/master/result_example/example.png)
![alt text](https://github.com/zhaozhongch/slam_groundtruth_comparison/blob/master/result_example/print_result.png)

