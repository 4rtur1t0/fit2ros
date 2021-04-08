"""
    Read from Fit files and save them to EUROC FORMAT

    USAGE:
    $python fit2euroc.py -f testdata/test.fit -v testdata/testvideo.mp4 -dir output

    USAGE:
    fit2euroc.py -f <fitfile> -v <videofile> -dir <directory>

    --fitfile: file in FIT GARMIN format.
    --videofile: the video in any format readable by Opencv.
    --directory: output directory in EUROC format
"""
import fitparse
from helpers import get_properties, time_operations, gps_messages, videoframes
from helpers.euroc_save import EurocSaver
import sys, getopt
import rospy


def get_command_line_args(argv):
    fitfilename = ''
    videofilename = ''
    out_euroc_dir = '/euroc'
    # no downsampling by default in video frames
    downsample = 1
    try:
        opts, args = getopt.getopt(argv, "hf:v:e:s:", ["fitfile=", "videofile=", "eurocdir=", "sampling="])
    except getopt.GetoptError:
        print('fit2euroc: save fit data to EUROC format file')
        print('USAGE:')
        print('fit2euroc.py -f <fitfile> -v <videofile> -e <eurocdir> -d <downsample>')
        print('OPTIONS: \n --fitfile\n --videofile\n --euroc \n --sampling \n')
        print('The FIT file in Garmin format, a video file in any format and the output EUROC directory name.')
        print('optional: use -s, --sampling <samples> to specify the resamling. P.e. 30 means that 1 out of 30 '
              'images are considered.')
        sys.exit(2)
    for opt, arg in opts:
        if opt == '-h':
            print('USAGE:')
            print('fit2euroc.py -f <fitfile> -v <videofile> -e <eurocdir> -d <downsample>')
            print('OPTIONS: \n --fitfile\n --videofile\n --euroc \n --sampling \n')
            print('The FIT file in Garmin format, a video file in any format and the output EUROC directory name.')
            print('optional: use -s, --sampling <samples> to specify the resamling. P.e. 30 means that 1 out of 30 '
                  'images are considered.')
            sys.exit()
        elif opt in ("-f", "--fitfile"):
            fitfilename = arg
        elif opt in ("-v", "--videofile"):
            videofilename = arg
        elif opt in ("-e", "--eurocdir"):
            rosbag = arg
        elif opt in ("-s", "--sampling"):
            downsample = int(arg)
    #fitfilename = 'testdata/test.fit'
    #videofilename = 'testdata/testvideo.mp4'
    #out_euroc_dir = 'euroc/'
    print('FIT filename is: ', fitfilename)
    print('Video filename: ', videofilename)
    print('Output EUROC DIRECTORY IS: ', out_euroc_dir)
    print('Sampling to: ', downsample)
    return fitfilename, videofilename, out_euroc_dir, downsample


def save_euroc(euroc_saver, gps_data, video_data, downsamplevideo):
    """
    Data structure:
    EUROC_DIRECTORY/
                   mav0/
                        cam0/
                            data/
                                1403636579763555584.png
                                1403636579763555588.png
                                1403636579763555655.png
                            data.csv --> csv with two columns timestamp/filename
                            sensor.yaml --> an opencv calibration file
                        cam1
                        imu0
                        imu1

                   mav1/


                   mav2/


                   ...

    :param euroc_saver:
    :param df_global:
    :param gps_data:
    :param video_data:
    :param downsamplevideo:
    :return:
    """
    print('SAVING EUROC FORMAT!')
    print('SAVING GPS!')
    euroc_saver.save_gps(gps_data)
    print('SAVING VIDEO IMAGES!')
    euroc_saver.save_video_images(video_data, downsamplevideo)

    print('FINISHED!')


def save_images(fitfilename, videofilename, downsamplevideo, output_euroc_dir):
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
    #df_gps = gps_data.get_pandas_df()

    # create camera frames, all times are referred to video_start_utc_time
    video_data = videoframes.VideoFrames(videofilename, video_start_utc_time)
    #df_video = video_data.get_pandas_df()

    #df_global = pd.concat([df_gps, df_video])
    #df_global = df_global.sort_values(by=['epoch'])
    # open now video to get frames
    video_data.open_video_capture()

    euroc_saver = EurocSaver(euroc_directory=output_euroc_dir)
    save_euroc(euroc_saver, gps_data, video_data, downsamplevideo)

    print('\n---')
    print('FINISHED! CHECK THE GENERATED ROSBAG FILE!')


if __name__ == '__main__':
    [fitfilename, videofilename, out_euroc_dir, downsample] = get_command_line_args(sys.argv[1:])
    try:
        save_images(fitfilename, videofilename, downsample, out_euroc_dir)
    except rospy.ROSInterruptException:
        pass