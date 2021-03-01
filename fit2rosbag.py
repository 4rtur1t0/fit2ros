"""
    Read from Fit files and save them to a bag file. Do not publish topics in ROS.
"""
import fitparse
from helpers import get_properties, time_operations, gps_messages, videoframes
from helpers.ros_publish import RosPublisher
import pandas as pd
import time
import sys, getopt
import rospy


def get_command_line_args(argv):
    fitfilename = ''
    videofilename = ''
    rosbag = ''
    try:
        opts, args = getopt.getopt(argv, "hf:v:b:", ["fitfile=", "videofile=", "rosbag="])
    except getopt.GetoptError:
        print 'fit2rosbab: save fit data to rosbag file'
        print 'USAGE:'
        print 'fit2rosbag.py -f <fitfile> -v <videofile> -b <rosbag>'
        sys.exit(2)
    for opt, arg in opts:
        if opt == '-h':
            print 'fit2rosbab: save fit data to rosbag file'
            print 'USAGE:'
            print 'fit2rosbag.py -f <fitfile> -v <videofile> -b <rosbag>'
            sys.exit()
        elif opt in ("-f", "--fitfile"):
            fitfilename = arg
        elif opt in ("-v", "--videofile"):
            videofilename = arg
        elif opt in ("-b", "--rosbag"):
            rosbag = arg
    print 'FIT filename is: ', fitfilename
    print 'Video filename: ', videofilename
    print 'Rosbag file is: ', rosbag
    return fitfilename, videofilename, rosbag


def save_data(ros_publisher, row, gps_data, video_data):
    # publish clock, encoded in epoch time
    ros_publisher.publish_clock(row['epoch'])
    if row['data_type'] == 'gps_metadata':
        # if gps_metadata message read, then publish in ros
        # print 'publishing gps metadata in ROS'
        # the UTC time is encoded in every gps element
        gps = gps_data.get_by_index(row['data_index'])
        ros_publisher.save_gps(gps)
        ros_publisher.save_gps_speed(gps)
        ros_publisher.save_gps_velocity(gps)

    elif row['data_type'] == 'video_frame':
        # now publish image in ros. Image indexes are consecutive in the df_global
        # at each timestep the next frame is captured
        success, image = video_data.get_next_frame()
        if success:
            # print 'publishing image in ros'
            # either publish the raw image and the compressed one
            # ros_publisher.publish_image(image, row['epoch'])
            ros_publisher.save_image_compressed(image, row['epoch'])
        else:
            print 'ERROR CAPTURING VIDEO: MAJOR FAILURE'
            return


def publish_video_and_sensors(fitfilename, videofilename, rosbagfilename):
    # Load the FIT file
    fitobject = fitparse.FitFile(fitfilename)

    # start by finding the start and stop times in the system. For the VIRB 360 these times should be expressed
    # in seconds and milliseconds.
    video_start_time, video_stop_time = get_properties.get_video_start_stop_times(fitobject)
    print 'FOUND TIMES'
    print 'start_time', video_start_time, ' (s)'
    print 'stop_time', video_stop_time, ' (s)'

    # get the correlation between system time and UTC time. This correlation occurs whenever the VIRB is able to get
    # a nice GPS signal
    # returning the UTC time for which system time is zero
    system_utc_time_origin = get_properties.get_timestamp_correlation(fitobject)
    # transform video times to UTC
    video_start_utc_time = time_operations.convert_system_seconds_to_utc(system_utc_time_origin=system_utc_time_origin,
                                                                         seconds_time=video_start_time)
    video_stop_utc_time = time_operations.convert_system_seconds_to_utc(system_utc_time_origin=system_utc_time_origin,
                                                                        seconds_time=video_stop_time)
    print 'In consequence: '
    print 'Video start time is: ', video_start_utc_time
    print 'Video stop time is: ', video_stop_utc_time

    # GPS READINGS ARE IN UTC!
    gps_data = gps_messages.GPSDataList(fitobject)
    df_gps = gps_data.get_pandas_df()

    # create camera frames, all times are referred to video_start_utc_time
    video_data = videoframes.VideoFrames(videofilename, video_start_utc_time)
    df_video = video_data.get_pandas_df()

    df_global = pd.concat([df_gps, df_video])
    df_global = df_global.sort_values(by=['epoch'])
    # open now video to get frames
    video_data.open_video_capture()

    # create a Ros publisher
    ros_publisher = RosPublisher(rosbagfilename=rosbagfilename)
    total_rows = len(df_global)

    # total_rows = 350
    # for each time in the table, and depending on the data type, publish data in ros
    for index in range(0, total_rows, 1):
        print 'Completed: ', 100.0*index/total_rows, '%'
        row = df_global.iloc[index, :]
        save_data(ros_publisher, row, gps_data, video_data)

    print 30*'--'
    print 'FINISHED! CHECK THE GENERATED ROSBAG FILE!'


if __name__ == '__main__':
    [fitfile, videofile, rosbagfilename] = get_command_line_args(sys.argv[1:])
    try:
        publish_video_and_sensors(fitfile, videofile, rosbagfilename)
    except rospy.ROSInterruptException:
        pass