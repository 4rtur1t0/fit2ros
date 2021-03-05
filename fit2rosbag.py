"""
    Read from Fit files and save them to a bag file. Do not publish topics in ROS, just write to rosbag.
    The UTC found in the FIT messages is directly written to the rosbag.

    USAGE:
    $python fit2rosbag.py -f testdata/test.fit -v testdata/testvideo.mp4 -b rosbag.bag

    USAGE:
    fit2rosbag.py -f <fitfile> -v <videofile> -b <rosbag>

    --fitfile: file in FIT GARMIN format.
    --videofile: the video in any format readable by Opencv.
    --rosbag: rosbag output file
"""
import fitparse
from helpers import get_properties, time_operations, gps_messages, videoframes
from helpers.ros_save import RosSaver
from helpers.json_save import JsonSaver
import pandas as pd
import sys, getopt
import rospy
import numpy as np


def get_command_line_args(argv):
    fitfilename = ''
    videofilename = ''
    rosbag = ''
    images_dir = 'testdata/frames'
    # no downsampling by default in video frames
    downsample = 1
    try:
        opts, args = getopt.getopt(argv, "hf:v:b:s:i:", ["fitfile=", "videofile=", "rosbag=", "sampling=", "imdir="])
    except getopt.GetoptError:
        print('fit2rosbab: save fit data to rosbag file')
        print('USAGE:')
        print('fit2rosbag.py -f <fitfile> -v <videofile> -b <rosbag> -d <downsample> -i <directory>')
        print('OPTIONS: \n --fitfile\n --videofile\n --rosbag \n --sampling \n --imdir')
        print('The FIT file in Garmin format, a video file in any format and the output rosbag name.')
        print('optional: use -s, --sampling <samples> to specify the resamling. P.e. 30 means that 1 out of 30 '
              'images are published.')
        sys.exit(2)
    for opt, arg in opts:
        if opt == '-h':
            print('fit2rosbab: save fit data to rosbag file')
            print('USAGE:')
            print('fit2rosbag.py -f <fitfile> -v <videofile> -b <rosbag> -s <sampling> -i <directory>')
            print('OPTIONS: \n --fitfile\n --videofile\n --rosbag \n --sampling \n --imdir')
            print('The FIT file in Garmin format, a video file in any format and the output rosbag name.')
            print('optional: use -s, --sampling <samples> to specify the resamling. P.e. 30 means that 1 out of 30 '
                  'images are published.')
            sys.exit()
        elif opt in ("-f", "--fitfile"):
            fitfilename = arg
        elif opt in ("-v", "--videofile"):
            videofilename = arg
        elif opt in ("-b", "--rosbag"):
            rosbag = arg
        elif opt in ("-i", "--imdir"):
            images_dir = arg
        elif opt in ("-s", "--sampling"):
            downsample = int(arg)
    # fitfilename = 'testdata/test.fit'
    # videofilename = 'testdata/testvideo.mp4'
    # images_dir = 'testdata/frames'
    # rosbag = 'testdata/rosbag.bag'
    print('FIT filename is: ', fitfilename)
    print('Video filename: ', videofilename)
    print('Output Rosbag file is: ', rosbag)
    print('Output images directory is: ', images_dir)
    print('Sampling to: ', downsample)
    return fitfilename, videofilename, rosbag, images_dir, downsample


def save_json(json_saver, df_global, gps_data, video_data, downsample):
    total_rows = len(df_global)
    video_indexes = create_sampling_vector(downsample, video_data.get_number_of_frames())
    print('SAVING JSON FILE!')
    # for each time in the table, and depending on the data type, publish data in ros
    for index in range(0, total_rows, 1):
        print('Completed: ', 100.0 * index / total_rows, '%', end='\r')
        row = df_global.iloc[index, :]
        #print(row)
        # publish clock, encoded in epoch time
        json_saver.append_clock(row['epoch'])
        if row['data_type'] == 'gps_metadata':
            # if gps_metadata message read, then publish in ros
            # print('publishing gps metadata in ROS')
            # the UTC time is encoded in every gps element
            gps = gps_data.get_by_index(row['data_index'])
            json_saver.append_gps(gps)
            json_saver.append_gps_speed(gps)
            json_saver.append_gps_velocity(gps)

        # saving images, images are resampled according to the video_indexes vector
        elif row['data_type'] == 'video_frame':
            save_image = video_indexes.pop(0)
            if save_image:
                json_saver.append_image(row['epoch'])
    # finally save all data to a json file
    json_saver.save_json_file()
    print('JSON FILE SAVED!')


def save_rosbag(ros_saver, df_global, gps_data, video_data, downsample):
    total_rows = len(df_global)
    # video_indexes establisheh the frames that need to be captured
    video_indexes = create_sampling_vector(downsample, video_data.get_number_of_frames())
    print('SAVING ROSBAG FILE!')
    # for each time in the table, and depending on the data type, publish data in ros
    for index in range(0, total_rows, 1):
        print('Completed: ', 100.0 * index / total_rows, '%', end='\r')
        row = df_global.iloc[index, :]
        # publish clock, encoded in epoch time
        ros_saver.save_clock(row['epoch'])
        if row['data_type'] == 'gps_metadata':
            # if gps_metadata message read, then publish in ros
            # print('publishing gps metadata in ROS')
            # the UTC time is encoded in every gps element
            gps = gps_data.get_by_index(row['data_index'])
            ros_saver.save_gps(gps)
            ros_saver.save_gps_speed(gps)
            ros_saver.save_gps_velocity(gps)

        elif row['data_type'] == 'video_frame':
            # now publish image in ros. Image indexes are consecutive in the df_global
            # at each timestep the next frame is captured
            success, image = video_data.get_next_frame()
            save_image = video_indexes.pop(0)
            if success and save_image:
                # save the raw image to rosbag
                ros_saver.save_image(image, row['epoch'])
                # save the image to a file in directory
                ros_saver.save_image_to_dir(image, row['epoch'])
                # ros_saver.save_image_compressed(image, row['epoch'])
            elif not success:
                print('ERROR CAPTURING VIDEO: MAJOR VIDEO FAILURE')
                return
    print('ROSBAG FILE SAVED!')


def create_sampling_vector(downsample, total_rows):
    # construct base sampling vector
    x = [1]
    for i in range(1, downsample, 1):
        x.append(0)
    # repeat n times
    total_list = np.tile(x, int(np.ceil(total_rows/downsample)+1))
    return total_list.tolist()


def save_video_and_sensors(fitfilename, imagesdirectory, videofilename, rosbagfilename, downsamplevideo):
    # Load the FIT file
    fitobject = fitparse.FitFile(fitfilename)

    # start by finding the start and stop times in the system. For the VIRB 360 these times should be expressed
    # in seconds and milliseconds.
    video_start_time, video_stop_time = get_properties.get_video_start_stop_times(fitobject)
    print('FOUND TIMES')
    print('start_time', video_start_time, ' (s)')
    print('stop_time', video_stop_time, ' (s)')

    # get the correlation between system time and UTC time. This correlation occurs whenever the VIRB is able to get
    # a nice GPS signal
    # returning the UTC time for which system time is zero
    system_utc_time_origin = get_properties.get_timestamp_correlation(fitobject)
    # transform video times to UTC
    video_start_utc_time = time_operations.convert_system_seconds_to_utc(system_utc_time_origin=system_utc_time_origin,
                                                                         seconds_time=video_start_time)
    video_stop_utc_time = time_operations.convert_system_seconds_to_utc(system_utc_time_origin=system_utc_time_origin,
                                                                        seconds_time=video_stop_time)
    print('Video start time is: ', video_start_utc_time)
    print('Video stop time is: ', video_stop_utc_time)

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

    # saving the json file to the same directory
    json_saver = JsonSaver(jsondirectory=imagesdirectory)
    save_json(json_saver, df_global, gps_data, video_data, downsamplevideo)

    # create a Ros saver to save data to a rosbag file
    ros_saver = RosSaver(rosbagfilename=rosbagfilename, imagesdirectory=imagesdirectory)
    save_rosbag(ros_saver, df_global, gps_data, video_data, downsamplevideo)

    print('\n---')
    print('FINISHED! CHECK THE GENERATED ROSBAG FILE!')


if __name__ == '__main__':
    [fitfile, videofile, rosbagfilename, images_dir, downsamplevideo] = get_command_line_args(sys.argv[1:])
    try:
        save_video_and_sensors(fitfile, images_dir, videofile, rosbagfilename, downsamplevideo)
    except rospy.ROSInterruptException:
        pass