# experiment s0 0.0
BASE_DIR='../../experiments/virb360/s0/0.0'
BASE_NAME='2021-03-03-12-53'
mkdir $BASE_DIR/frames10fps
mkdir $BASE_DIR/frames6fps
python fit2rosbag.py -f $BASE_DIR/$BASE_NAME.fit -v $BASE_DIR/${BASE_NAME}_front.MP4 -b $BASE_DIR/${BASE_NAME}_front_10fps.bag -s 3 -i $BASE_DIR/frames10fps
python fit2rosbag.py -f $BASE_DIR/$BASE_NAME.fit -v $BASE_DIR/${BASE_NAME}_front.MP4 -b $BASE_DIR/${BASE_NAME}_front_6fps.bag -s 5 -i $BASE_DIR/frames6fps



BASE_DIR='../../experiments/virb360/s0/0.1'
BASE_NAME='2021-03-03-13-09'
mkdir $BASE_DIR/frames10fps
mkdir $BASE_DIR/frames6fps
python fit2rosbag.py -f $BASE_DIR/$BASE_NAME.fit -v $BASE_DIR/${BASE_NAME}_front.MP4 -b $BASE_DIR/${BASE_NAME}_front_10fps.bag -s 3 -i $BASE_DIR/frames10fps
python fit2rosbag.py -f $BASE_DIR/$BASE_NAME.fit -v $BASE_DIR/${BASE_NAME}_front.MP4 -b $BASE_DIR/${BASE_NAME}_front_6fps.bag -s 5 -i $BASE_DIR/frames6fps

