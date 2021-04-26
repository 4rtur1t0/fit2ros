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
import sys
import getopt
import rospy


def get_command_line_args(argv):
    fitfilename = None
    videofront = None
    videoback = None
    out_euroc_dir = '/euroc'
    # downsample=1 means no downsampling
    # p.e. downsample = 5 means 1 out of 5 images are kept
    downsample = 1
    try:
        opts, args = getopt.getopt(argv, "hf:vf:fb:e:s:", ["fitfile=", "videofront=", "videoback=", "eurocdir=", "sampling="])
    except getopt.GetoptError:
        print('fit2euroc: save fit data to EUROC format file')
        print('USAGE:')
        print('fit2euroc.py -f <fitfile> -vf <videofront> -vb <videoback> -e <eurocdir> -d <downsample>')
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
        elif opt in ("-v", "--videofront"):
            videofront = arg
        elif opt in ("-b", "--videoback"):
            videoback = arg
        elif opt in ("-e", "--eurocdir"):
            out_euroc_dir = arg
        elif opt in ("-s", "--sampling"):
            downsample = int(arg)
    # fitfilename = '/media/arvc/50324D7B324D6756/datasets/umhVIRB360/garmin_last/2021-03-25-16-41-44.fit'
    # videofront = '/media/arvc/50324D7B324D6756/datasets/umhVIRB360/garmin_last/V6813138.MP4'
    # videoback =  '/media/arvc/50324D7B324D6756/datasets/umhVIRB360/garmin_last/V6813139.MP4'
    # out_euroc_dir = '/media/arvc/50324D7B324D6756/datasets/umhVIRB360/garmin_last/s1/0'

    # fitfilename = '/home/arvc/Escritorio/datasets/umhVIRB360/s1/1/2021-03-25-16-26.fit'
    # videofront = '/home/arvc/Escritorio/datasets/umhVIRB360/s1/1/front.MP4'
    # videoback = '/home/arvc/Escritorio/datasets/umhVIRB360/s1/1/back.MP4'
    # out_euroc_dir = 'euroc/'

    print('FIT filename is: ', fitfilename)
    print('Video front filename: ', videofront)
    print('Video back filename: ', videoback)
    print('Output EUROC DIRECTORY IS: ', out_euroc_dir)
    print('Sampling to: ', downsample)
    return fitfilename, videofront, videoback, out_euroc_dir, downsample


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
                            data/
                        imu0
                        imu1

                   mav1/


                   mav2/


                   ...

        cam0 is associated
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


def save_images(fitfilename, videofront, videoback, downsamplevideo, output_euroc_dir):
    # if fit file is provided
    # all times are taken from the fit file
    if fitfilename is not None:
        # Load the FIT file
        fitobject = fitparse.FitFile(fitfilename)

        # start by finding the start and stop times in the system. For the VIRB 360 these times should be expressed
        # in seconds and milliseconds.
        video_start_time, video_stop_time = get_properties.get_video_start_stop_times(fitobject)
        print(30*'*')
        print('TIME INFORMATION FROM FIT FILE')
        print('FOUND TIMES')
        print('start_time', video_start_time, ' (s)')
        print('stop_time', video_stop_time, ' (s)')
        print('DURATION: ', video_stop_time-video_start_time, ' (s)')
        print(30*'*')

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
        gps_data.build_gps_lists()
        # save front images in EUROC
        ###############################################################
        # create camera frames, all times are referred to video_start_utc_time
        video_data = videoframes.VideoFrames(videofront, video_start_utc_time)
        # open now video to get frames
        video_data.open_video_capture()
        # save in Euroc format
        euroc_saver = EurocSaver(euroc_directory=output_euroc_dir, camera_directory='/cam0')
        save_euroc(euroc_saver, gps_data, video_data, downsamplevideo)
        # save back images in EUROC
        ###############################################################
        # create camera frames, all times are referred to video_start_utc_time
        video_data = videoframes.VideoFrames(videoback, video_start_utc_time)
        # open now video to get frames
        video_data.open_video_capture()
        # save in Euroc format
        euroc_saver = EurocSaver(euroc_directory=output_euroc_dir, camera_directory='/cam1')
        save_euroc(euroc_saver, gps_data, video_data, downsamplevideo)

        print('\n---')
        print('FINISHED! CHECK THE GENERATED EUROC DIRECTORY!')
    # if no fit file is provided, just refer everything to 01/01/1970
    # no gps data is saved
    else:
        gps_data = gps_messages.GPSDataList(None)
        video_start_utc_time = time_operations.convert_epoch_to_utc(0)
        # create camera frames, all times are referred to video_start_utc_time
        video_data = videoframes.VideoFrames(videofront, video_start_utc_time)
        # open now video to get frames
        video_data.open_video_capture()
        # save in Euroc format
        euroc_saver = EurocSaver(euroc_directory=output_euroc_dir, camera_directory='/cam0')
        save_euroc(euroc_saver, gps_data, video_data, downsamplevideo)

        ####################################
        gps_data = gps_messages.GPSDataList(None)
        video_start_utc_time = time_operations.convert_epoch_to_utc(0)
        # create camera frames, all times are referred to video_start_utc_time
        video_data = videoframes.VideoFrames(videoback, video_start_utc_time)
        # open now video to get frames
        video_data.open_video_capture()
        # save in Euroc format
        euroc_saver = EurocSaver(euroc_directory=output_euroc_dir, camera_directory='/cam1')
        save_euroc(euroc_saver, gps_data, video_data, downsamplevideo)

        print('\n---')
        print('FINISHED! CHECK THE GENERATED EUROC DIRECTORY!')


if __name__ == '__main__':
    [fitfilename, videofront, videoback, out_euroc_dir, downsample] = get_command_line_args(sys.argv[1:])
    try:
        save_images(fitfilename, videofront, videoback, downsample, out_euroc_dir)
    except rospy.ROSInterruptException:
        pass