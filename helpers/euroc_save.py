import os
import pandas as pd
import cv2
import numpy as np


class EurocSaver():
    """
    Class that saves FIT information (fit file and video file) to EUROC format.
    """
    def __init__(self, euroc_directory=None, camera_directory=None):
        self.euroc_directory = euroc_directory
        self.images_directory = euroc_directory + '/mav0' + camera_directory

    def save_gps(self, gps_data):
        try:
            os.makedirs(self.euroc_directory+'/mav0/'+'/gps0')
        except OSError:
            print("Creation of the directory %s failed" % self.euroc_directory)
        else:
            print("Successfully created the directory %s " % self.euroc_directory)
        lat_list = []
        lng_list = []
        altitude_list = []
        epoch_list = []
        for i in range(0, len(gps_data.data_list), 1):
            print('Completed: ', 100.0 * i / len(gps_data.data_list), '%', end='\r')
            epoch_list.append(int(gps_data.epoch_list[i]*1000))
            lat_list.append(gps_data.data_list[i].lat)
            lng_list.append(gps_data.data_list[i].lng)
            altitude_list.append(gps_data.data_list[i].altitude)

        raw_data = {'timestamp': epoch_list,
                    'lat': lat_list,
                    'lng': lng_list,
                    'altitude': altitude_list}
        df = pd.DataFrame(raw_data, columns=['timestamp', 'lat', 'lng', 'altitude'])
        df.to_csv(self.euroc_directory+'/mav0/'+'/gps0/data.csv', index=False, header=['#timestamp [ns]', 'lat', 'lng', 'altitude'])
        print('\n---')

    def save_video_images(self, video_data, downsamplevideo):
        try:
            os.makedirs(self.euroc_directory + '/mav0')
        except OSError:
            print("Creation of the directory %s failed" % self.euroc_directory)
        else:
            print("Successfully created the directory %s " % self.euroc_directory)

        try:
            os.makedirs(self.images_directory + '/data')
        except OSError:
            print("Creation of the directory %s failed" % self.images_directory)
        else:
            print("Successfully created the directory %s " % self.images_directory)

        # video_indexes establishes the frames that need to be captured
        video_indexes = create_sampling_vector(downsamplevideo, video_data.get_number_of_frames())

        epoch_list = []
        filenames = []
        for i in range(0, video_data.get_number_of_frames(), 1):
            print('Saved ', 100*i/video_data.get_number_of_frames(), '%', end='\r')
            success, image = video_data.get_next_frame()
            save_image = video_indexes.pop(0)
            if success and save_image:
                epoch = str(int(video_data.epoch_list[i]*1000))
                self.save_image_to_dir(image, epoch + '.png')
                epoch_list.append(epoch)
                filenames.append(epoch + '.png')

        raw_data = {'timestamp': epoch_list,
                    'filenames': filenames}
        df = pd.DataFrame(raw_data, columns=['timestamp', 'filenames'])
        df.to_csv(self.images_directory + '/data.csv', index=False, header=['#timestamp [ns]', 'filenames'])
        print('\n---')

    def save_image_to_dir(self, image, imagename):
        """
        Saves image to a directory
        :param image:
        :param epoch:
        :return:
        """
        filename = self.images_directory + '/data/' + imagename
        cv2.imwrite(filename, image)


def create_sampling_vector(downsample, total_rows):
    # construct base sampling vector
    x = [1]
    for i in range(1, downsample, 1):
        x.append(0)
    # repeat n times
    total_list = np.tile(x, int(np.ceil(total_rows/downsample)+1))
    return total_list.tolist()