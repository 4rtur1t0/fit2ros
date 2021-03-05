# experiment s0 0.0
mkdir testdata/frames10fps
python fit2rosbag.py -f testdata/test.fit -v testdata/testvideo.mp4 -b testdata/rosbag.bag -s 3 -i testdata/frames10fps
