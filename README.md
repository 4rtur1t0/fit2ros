PURPOSE
fit2ros is a set of python scripts that allow to read a FIT file and video files and publish them in the ROS ecosystem.

The main purpose is to read FIT files and videos and convert them to rosbag format so that they can be used in research
activities

fit2ros expects different messages from the FIT file, in particular:
"camera_event": that indicates the VIRB system time when the recording of the video started

DEPENDENCIES
It is based on the (very nice) fit2parse library to parse FIT files. Also, depends upon
cv2
numpy
pandas

USAGE

fit2ros: Publish
$ python fit2ros.py -f testdata/test.fit -v testdata/testvideo.mp4 -d 0.1 -s 1

In order to work properly, roscore must be active
$ roscore


fit2rosbag:
fit2rosbag saves a rosbag file.
All times in the rosbag are saved using the UTC times found in the FIT file.

$python fit2rosbag.py -f testdata/test.fit -v testdata/testvideo.mp4 -b rosbag.bag -s 5











fit2ros broadcasts messages to the ROS ecosystem. The real-time /clock is not emulated. However, a rosbag file can be
saved and then played back so that the real time is emulated.
Steps:
1)launch roscore:
$ roscore

2) launch rosbag record

Images from the VIRB video file are published both in image/image_raw and image/image_raw/compressed.
Only the compressed image should be usually saved to the rosbag to avoid a very large rosbag file.

Exclude image_raw
$rosbag record -a -x "/image/image_raw" -O test.bag

Save everything (may generate very large rosbag files)
$rosbag record -a -O test.bag

3) launch fit2ros
$python fit2ros -fitfile test.fit --videofile video.mp4



Once finished, stop rosbag record and check the generated .bag file.

4) Just replay with rosbag play
$rosbag play test.bag

5) The rosbag is saved with the time corresponding to the video and fit files
In order to publish using this time, just use:
$ roscore
$ rosparam set use_sim_time true
$ rosbag play --clock --pause rosbag.bag




MAIN ISSUES
when using fit2ros, adjust the deltatime parameter to slow the publication of topics, please be aware that the publication
of topics is no performed in real time.


EXAMPLES:
Create a rosbag file downsampling video fps by 5
python fit2rosbag.py -f testdata/test.fit -v testdata/testvideo.MP4 -b testdata/testbag.bag -s 5


Publish to ROS, also downsample video fps by 30
python fit2ros.py -f testdata/test.fit -v testdata/testvideo.MP4 -s 30


