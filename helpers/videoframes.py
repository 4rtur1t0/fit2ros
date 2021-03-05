"""
    A class to capture frames from a video using OpenCV.
"""
import cv2
import pandas as pd
from datetime import datetime
from datetime import timedelta
from helpers import time_operations


class VideoFrames():
    """
        A class to read frames from videos using opencv.
        The method build_lists builds a list of frames and times.
        Times are computed starting at video_start_time_utc which is provided from the FIT file using the message
        correlation time
    """
    def __init__(self, filename, video_start_time_utc):
        self.fps = None
        self.frame_count = None
        self.vidcap = None
        self.filename = filename
        self.epoch_list = []
        self.utc_list = []
        self.index_list = []
        self.data_type_list = []
        self.build_lists(video_start_time_utc)

    def open_video_capture(self):
        """
        :param record:
        :return:
        """
        self.vidcap = cv2.VideoCapture(self.filename)
        self.fps = self.vidcap.get(cv2.CAP_PROP_FPS)
        print('Video FPS found: ', self.fps)

    def build_lists(self, video_start_time_utc):
        """
        The function starts by considering the initial UTC time of the first time. Next frames' times are computed based
        on the fps of the video, as read by OpenCV. A list of frames and UTC times is returned.
        :param video_start_time_utc:
        :return:
        """
        print('Opening video and counting frames!')
        # get FPS of video and get total number of frames
        self.vidcap = cv2.VideoCapture(self.filename)
        self.fps = self.vidcap.get(cv2.CAP_PROP_FPS)
        self.frame_count = int(self.vidcap.get(cv2.CAP_PROP_FRAME_COUNT))
        duration = self.frame_count/self.fps
        print('Building image list! Found duration: ', duration, 's')
        print('Found ', self.frame_count, 'frames in video')

        count = 0
        # emulating video frame capture
        while count < self.frame_count:
            elapsed_time = (1/self.fps)*count
            # print('Frame number: ', count)
            # print('Elapsed time: ', elapsed_time, ' (s)')
            # print('Total time UTC: ', video_start_time_utc + timedelta(seconds=elapsed_time))
            total_utc_time = video_start_time_utc + timedelta(seconds=elapsed_time)
            self.utc_list.append(total_utc_time)
            self.epoch_list.append(time_operations.convert_utc_to_epoch(total_utc_time))
            self.index_list.append(count)
            self.data_type_list.append('video_frame')
            count += 1

    def get_pandas_df(self):
        raw_data = {'epoch': self.epoch_list,
                    'data_index': self.index_list,
                    'utc_time': self.utc_list,
                    'data_type': self.data_type_list}
        df = pd.DataFrame(raw_data)
        return df

    def get_next_frame(self):
        success, image = self.vidcap.read()
        return success, image

    def get_fps(self):
        return self.fps

    def get_number_of_frames(self):
        return self.frame_count
