"""
    get_properties: gets the needed properties from the FIT list of messages

    author: Arturo Gil
    email: arturo.gil@umh.es
    institution: Universidad Miguel Hernandez de Elche, Alicante, Spain
    date: february 2021
"""
import datetime


class CameraEvent():
    """
        A class to read the camera_event event from the FIT messages.
    """
    def __init__(self, record):
        self.timestamp = None
        self.timestamp_ms = None
        self.camera_event_type = None

        self.read_record(record)
        self.system_time = self.timestamp + self.timestamp_ms/1000.0

    def read_record(self, record):
        for data in record.fields:
            if data.name == 'timestamp':
                self.timestamp = data.value
            elif data.name == 'timestamp_ms':
                self.timestamp_ms = data.value
            elif data.name == 'camera_event_type':
                self.camera_event_type = data.value

    def is_start(self):
        return self.camera_event_type == 'video_start'

    def is_stop(self):
        return self.camera_event_type == 'video_end'

    def get_time(self):
        return self.system_time


class TimestampCorrelation():
    """
        A class to read the time_correlation record
    """
    def __init__(self, record):
        self.timestamp = None
        self.timestamp_ms = None
        self.system_timestamp = None
        self.system_timestamp_ms = None
        self.local_timestamp = None
        self.read_record(record)
        self.utc_timestamp = self.timestamp + datetime.timedelta(milliseconds=self.timestamp_ms)
        #self.system_timestamp = self.system_timestamp + self.system_timestamp_ms/1000.0

    def read_record(self, record):
        for data in record.fields:
            if data.name == 'timestamp':
                self.timestamp = data.value
            elif data.name == 'timestamp_ms':
                self.timestamp_ms = data.value
            elif data.name == 'system_timestamp':
                self.system_timestamp = data.value
            elif data.name == 'system_timestamp_ms':
                self.system_timestamp_ms = data.value
            elif data.name == 'local_timestamp':
                self.local_timestamp = data.value

    def get_utc_time(self):
        return self.utc_timestamp


def get_video_start_stop_times(fitfile):
    """
    Finds start and stop time from fit file file
    :param fitfile:
    :return:
    """
    a = set()
    camera_events_start = []
    camera_events_stop = []
    # Iterate over all messages of type "camera_event"
    for record in fitfile.get_messages("camera_event"):
        camera_event = CameraEvent(record)
        if camera_event.is_start():
            camera_events_start.append(camera_event)
        elif camera_event.is_stop():
            camera_events_stop.append(camera_event)
    if len(camera_events_start)==0 or len(camera_events_stop)==0:
        print('ERROR: COULD NOT GET START AND STOP TIMES OF VIDEO FROM FIT FILE')
        print('EXITING')
        exit()

    for i in range(0, len(camera_events_start), 1):
        tiempo = camera_events_stop[i].get_time()-camera_events_start[i].get_time()
        print(tiempo)
        print("---")
    start_time = camera_events_start[0].get_time()
    stop_time = camera_events_stop[0].get_time()
    print('CAUTION: returning the first set of start and stop')
    return start_time, stop_time


def get_timestamp_correlation(fitfile):
    """
    Finds the correlation time between the system and UTC
    :param fitfile:
    :return:
    """
    timestamp_correlations = []
    # Iterate over all messages of type "camera_event"
    for record in fitfile.get_messages("timestamp_correlation"):
        timestamp_correlation = TimestampCorrelation(record)
        print('Found correlation in FIT')
        print('UTC time: ', timestamp_correlation.utc_timestamp, ' corresponds to')
        print('System seconds time (s): ', timestamp_correlation.system_timestamp)
        timestamp_correlations.append(timestamp_correlation)
    if len(timestamp_correlations) == 0:
        print('CAUTION: NO TIMESTAMP CORRELATION FOUND, SELECTING UTCTIME=JANUARY, 1ST 1970!')
        return datetime.datetime(year=1970, month=1, day=1), 0.0
    elif len(timestamp_correlations) > 1:
        print('CAUTION: MORE THAN ONE TIMESTAMP CORRELATION FOUND, SELECTING THE LAST FOUND!')

    utc_time_origin = timestamp_correlation.utc_timestamp - \
                      datetime.timedelta(seconds=timestamp_correlation.system_timestamp,
                                         milliseconds=timestamp_correlation.system_timestamp_ms)
    return utc_time_origin