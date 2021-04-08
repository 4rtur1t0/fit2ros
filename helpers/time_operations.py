"""
    time operations: main operations dealing with time in UTC

    author: Arturo Gil
    email: arturo.gil@umh.es
    institution: Universidad Miguel Hernandez de Elche, Alicante, Spain
    date: february 2021
"""
import datetime


def convert_system_seconds_to_utc(system_utc_time_origin, seconds_time):
    return system_utc_time_origin + datetime.timedelta(seconds=seconds_time)


def convert_utc_to_epoch(utc_timestamp):
    unix_epoch = datetime.datetime(1970, 1, 1)
    log_dt = utc_timestamp
    seconds_from_epoch = (log_dt - unix_epoch).total_seconds()
    return seconds_from_epoch


def convert_utc_to_epoch_ns(utc_timestamp):
    unix_epoch = datetime.datetime(1970, 1, 1)
    log_dt = utc_timestamp
    seconds_from_epoch = (log_dt - unix_epoch).total_seconds()*1e9
    return seconds_from_epoch

def convert_epoch_to_utc(utc_epoch):
    unix_epoch = datetime.datetime(1970, 1, 1)
    return unix_epoch + datetime.timedelta(seconds=utc_epoch)
