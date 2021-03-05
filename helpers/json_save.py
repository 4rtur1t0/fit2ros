import json


class JsonSaver():
    """
    Class that saves different messages to a json file.
    """
    def __init__(self, jsondirectory=None):
        self.data = []
        self.jsondirectory = None
        self.jsonfile = None
        if jsondirectory:
            self.jsondirectory = jsondirectory
            self.jsonfile = open(jsondirectory + '/' + 'virb360_data.json', 'w')

    def __del__(self):
        print("Closing json file")
        self.jsonfile.close()

    def append_clock(self, epoch):
        clock_d = {'topic': '/clock',
                   'time': str(epoch)}
        self.data.append(clock_d)

    def append_gps(self, gps):
        gps_d = {'topic': 'virb360/gps/fix',
                 'lat': gps.lat,
                 'lng': gps.lng,
                 'altitude': gps.altitude,
                 'time': str(gps.epoch)}
        self.data.append(gps_d)

    def append_gps_speed(self, gps):
        """
        Save GPS speed as measured by the GPS on the FIT device.
        :param gps:
        :return:
        """
        speed_d = {'topic': 'virb360/gps/speed',
                   'speed': gps.speed,
                   'time': str(gps.epoch)}
        self.data.append(speed_d)

    def append_gps_velocity(self, gps):
        """
        Save a 3D velocity vector.
        :param gps:
        :return:
        """
        velocity_d = {'topic': 'virb360/gps/velocity',
                      'velocity': '(' + str(gps.velocity[0]) + ', '
                                  + str(gps.velocity[1]) + ', ' + str(gps.velocity[2]) + ')',
                      'time': str(gps.epoch)}
        self.data.append(velocity_d)

    def append_image(self, epoch):
        """
        Saves image to a file in self.jsondirectory directory.
        This name must match the name in RosSave().save_image()
        :param image:
        :param epoch:
        :return:
        """
        image_d = {'topic': 'image/image_raw',
                   'image_name': self.jsondirectory + '/' + str(epoch) + '.png',
                   'time': str(epoch)}
        self.data.append(image_d)

    def save_json_file(self):
        json.dump(self.data, self.jsonfile, indent=4)


