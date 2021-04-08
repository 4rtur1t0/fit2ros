"""
    Read from Fit files and save them to EUROC FORMAT

    USAGE:
    $python fit2euroc.py -input <directory with images> -output data.csv

    USAGE:
    fit2euroc.py -i <directory> -o data.csv

    --input: input directory with images
    --output: output csv file in EUROC format
"""
import fitparse
from helpers import get_properties, time_operations, gps_messages, videoframes
from helpers.ros_save import RosSaver
from helpers.json_save import JsonSaver
import pandas as pd
import sys, getopt
import rospy
import numpy as np
import datetime
import glob
import os

from helpers.time_operations import convert_utc_to_epoch, convert_utc_to_epoch_ns


def get_command_line_args(argv):
    output_file = ''
    input_dir = ''
    # no downsampling by default in video frames
    try:
        opts, args = getopt.getopt(argv, "hi:o:", ["input=", "output="])
    except getopt.GetoptError:
        print('images2euroc: save raw images to EUROC format')
        print('USAGE:')
        print('images2euroc.py -i <input_directory> -o <output csv data file>')
        print('OPTIONS: \n --input\n --output\n ')
        print('The input directory for images and output directory for EUROC format.')

        sys.exit(2)
    for opt, arg in opts:
        if opt == '-h':
            print('images2euroc: save raw images to EUROC format')
            print('USAGE:')
            print('images2euroc.py -i <input_directory> -o <output_directory>')
            print('OPTIONS: \n --input\n --output\n ')
            print('The input directory for images and output directory for EUROC format.')
            sys.exit()
        elif opt in ("-i", "--input"):
            input_dir = arg
        elif opt in ("-o", "--output"):
            output_file = arg
    #input_dir = 'testdata/frames'
    #output_file = 'data.csv'
    print('Input directory is: ', input_dir)
    print('Output file is: ', output_file)
    return input_dir, output_file


def get_filenames(input_dir):
    """
    Getting only filenames as EUROC format requires
    :param input_dir:
    :return:
    """
    images_list = []
    for fileName_relative in glob.glob(input_dir+"**/*", recursive=True):
        fileName_absolute = os.path.basename(fileName_relative)
        images_list.append(fileName_absolute)
    return images_list


def save_images_euroc_format(input_dir, output_file):
    """
    Generates a data.csv file that must be copied to the corresponding camera directory next to the data directory.
    :param input_dir:
    :param output_file:
    :return:
    """
    images_list = get_filenames(input_dir=input_dir)
    print('FOUND IMAGES: ', len(images_list))
    time_list = []
    for i in range(0, len(images_list), 1):
        tiempo = datetime.datetime.utcnow()
        tiempo = convert_utc_to_epoch_ns(tiempo)
        time_list.append("{:10.0f}".format(tiempo))
        i += 1

    raw_data = {'timestamp': time_list,
                'filename': images_list}

    df = pd.DataFrame(raw_data, columns=['timestamp', 'filename'])
    df.to_csv(output_file, index=False, header=True)
    print('\n---')
    print('FINISHED! CHECK THE GENERATED EUROC DIRECTORY')


if __name__ == '__main__':
    [input_dir, output_file] = get_command_line_args(sys.argv[1:])
    save_images_euroc_format(input_dir, output_file)