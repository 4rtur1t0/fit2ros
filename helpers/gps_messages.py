"""

"""
import datetime
#from datetime import datetime
import pandas as pd


class GPSDataSample():
    """
        A class to read the time_correlation record
    """
    def __init__(self, record):
        self.timestamp = None
        self.timestamp_ms = None
        self.utc_timestamp = None
        self.lat = None
        self.lng = None
        self.speed = None
        self.heading = None
        self.altitude = None
        self.velocity = None
        self.epoch = None
        # read previous data
        self.read_record(record)
        # minor tweaks
        self.timestamp = self.timestamp + self.timestamp_ms / 1000.0
        # caution, adding ms to utc timestamp to be consistent
        self.utc_timestamp = self.utc_timestamp + datetime.timedelta(milliseconds=self.timestamp_ms)
        self.epoch = self.find_epoch_time()

    def read_record(self, record):
        """
        degrees = semicircles * ( 180 / 2^31 )
        :param record:
        :return:
        """
        for data in record.fields:
            if data.name == 'timestamp':
                self.timestamp = data.value
            elif data.name == 'timestamp_ms':
                self.timestamp_ms = data.value
            elif data.name == 'utc_timestamp':
                self.utc_timestamp = data.value
            elif data.name == 'position_lat':
                self.lat = data.value * (180.0 / pow(2, 31))
            elif data.name == 'position_long':
                self.lng = data.value * (180.0 / pow(2, 31))
            elif data.name == 'heading':
                self.heading = data.value
            elif data.name == 'enhanced_altitude':
                self.altitude = data.value
            elif data.name == 'velocity':
                self.velocity = data.value
            elif data.name == 'enhanced_speed':
                self.speed = data.value

    def find_epoch_time(self):
        unix_epoch = datetime.datetime(1970, 1, 1)
        log_dt = self.utc_timestamp
        seconds_from_epoch = (log_dt - unix_epoch).total_seconds()
        return seconds_from_epoch

    def get_utc_time(self):
        return self.utc_timestamp

    def get_epoch_time(self):
        return self.epoch



class GPSDataList():
    def __init__(self, fitfile):
        self.fitfile = fitfile
        self.epoch_list = []
        self.utc_list = []
        self.index_list = []
        self.data_list = []
        self.data_type_list = []
        self.build_gps_lists()

    def build_gps_lists(self):
        """
        Finds start and stop time from fit file file
        :param fitfile:
        :return:
        """
        index = 0
        # Iterate over all messages of type "gps_metadata"
        for record in self.fitfile.get_messages("gps_metadata"):
            gps_data_record = GPSDataSample(record)
            self.data_list.append(gps_data_record)
            self.index_list.append(index)
            self.utc_list.append(gps_data_record.get_utc_time())
            self.epoch_list.append(gps_data_record.get_epoch_time())
            self.data_type_list.append('gps_metadata')
            index += 1

    def get_by_index(self, index):
        return self.data_list[index]

    def get_lists(self):
        return self.epoch_list, self.utc_list, self.index_list

    def get_pandas_df(self):
        """
        Get the list in df format for pandas.
        :return:
        """
        raw_data = {'epoch': self.epoch_list,
                    'data_index': self.index_list,
                    'utc_time': self.utc_list,
                    'data_type': self.data_type_list}
        df = pd.DataFrame(raw_data)
        return df
