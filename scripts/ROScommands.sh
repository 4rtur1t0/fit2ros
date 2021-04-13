ROSBAG SAVE COMMANDS
# Save to rosbag 1 out of 5 images--> this results in 30fps/5=6fps
python fit2rosbag.py -f ../virb360/s0/0.0/2021-03-03-12-53.fit -v ../virb360/s0/0.0/2021-03-03-12-53_front.MP4 -b ../virb360/s0/0.0/2021-03-03-12-53_6fps.bag -s 5

# Save to rosbag 1 out of 30 images--> this results in 30fps/30=1fps
python fit2rosbag.py -f ../virb360/s0/0.0/2021-03-03-12-53.fit -v ../virb360/s0/0.0/2021-03-03-12-53_front.MP4 -b ../virb360/s0/0.0/2021-03-03-12-53_1fps.bag -s 30

# Save to rosbag 1 out of 60 images--> this results in 30fps/60=0.5fps
python fit2rosbag.py -f ../virb360/s0/0.0/2021-03-03-12-53.fit -v ../virb360/s0/0.0/2021-03-03-12-53_front.MP4 -b ../virb360/s0/0.0/2021-03-03-12-53_0.5fps.bag -s 60

PUBLISH COMMANDS from FitFile and Video file
a) start roscore
$roscore
b) 
c) publish from Fitfile and video file

python fit2ros.py -f ../virb360/s0/0.0/2021-03-03-12-53.fit -v ../virb360/s0/0.0/2021-03-03-12-53_front.MP4 -s 60



PUBLISH COMMANDS from rosbag
$roscore
set simulation time true
$rosparam set use_sim_time true

d) you can either publish from an already generated rosbag file, for example. if simulated time is needed, then use the --clock option
rosbag play ../virb360/s0/0.0/2021-03-03-12-53_1fps.bag --clock --pause -s 220




RVIZ: Visualize in Rviz
e) run rviz
rosrun rviz rviz
