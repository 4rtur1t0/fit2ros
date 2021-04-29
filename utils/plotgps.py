"""
Plot GPS data from EUROC dir
"""
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import cartopy.crs as ccrs
import cartopy.io.img_tiles as cimgt
import io
from urllib.request import urlopen, Request
from PIL import Image
import getopt
import sys


def get_command_line_args(argv):
    input_euroc_dir = None
    zoom = 0.0005
    try:
        opts, args = getopt.getopt(argv, "h:e:z", ["help=", "eurocdir=", "zoom="])
    except getopt.GetoptError:
        print('plotgps: read from GPS euroc format and plot')
        print('USAGE:')
        print('plotgps.py -e <eurocdir>')
        print('OPTIONS: \n --eurocdir\n \n')
        print('Please specify a directory in EUROC/ASL format')
        sys.exit(2)
    for opt, arg in opts:
        if opt == '-h':
            print('plotgps: read from GPS euroc format and plot')
            print('USAGE:')
            print('plotgps.py -e <eurocdir>')
            print('OPTIONS: \n --eurocdir\n \n')
            print('Please specify a directory in EUROC/ASL format')
            sys.exit()
        elif opt in ("-e", "--eurocdir"):
            input_euroc_dir = arg
        elif opt in ("-z", "--zoom"):
            zoom = arg
    input_euroc_dir = '/media/arvc/50324D7B324D6756/datasets/umhVIRB360/euroc/s3/0/'
    print('Input EUROC dir: ', input_euroc_dir)
    return input_euroc_dir, zoom


def associate_gps(df_images, df_gps):
    """
    For each time found in df_images, find the closest time in df_gps and create an extended df with
    timestamp image_name lat lng altitude
    Save to a common df
    :param df_images:
    :param df_gps:
    :return:
    """
    df_images_extended = []
    for ind in df_images.index:
        images_timestamp = df_images['#timestamp [ns]'][ind]
        [lat, lng, altitude] = find_closest(timestamp=images_timestamp, df_gps=df_gps)
        df_images_extended.append([images_timestamp, df_images['filenames'][ind], lat, lng, altitude])
    df_images_extended = pd.DataFrame(df_images_extended, columns=('#timestamp [ns]', 'filenames', 'lat', 'lng', 'altitude'))
    print('FOUND ', len(df_images_extended), ' GPS-IMAGE ASSOCIATIONS')
    return df_images_extended


def find_closest(timestamp, df_gps):
    result_index = df_gps['#timestamp [ns]'].sub(timestamp).abs().idxmin()
    delta_time = np.abs((timestamp - df_gps['#timestamp [ns]'][result_index]))
    if delta_time > 10000:
        print('ERROR FINDING CORRESPONDING GPS!!!')
        print('Delta time is: ', delta_time, 'ns')
    return df_gps['lat'][result_index], df_gps['lng'][result_index], df_gps['altitude'][result_index]


def read_data(input_euroc_dir):
    print("Reading data.csv from EUROC directory: ", input_euroc_dir)
    # TODO: add options to access other robots or gps1 gps2...etc.
    csv_filename = input_euroc_dir + '/mav0/gps0/data.csv'
    df_gps = pd.read_csv(csv_filename)
    print('FOUND GPS: ', len(df_gps))
    print(df_gps)
    csv_filename = input_euroc_dir + '/mav0/cam0/data.csv'
    df_images = pd.read_csv(csv_filename)
    print('FOUND IMAGES: ',  len(df_images))
    print(df_images)
    return df_gps, df_images


def image_spoof(self, tile): # this function pretends not to be a Python script
    url = self._image_url(tile) # get the url of the street map API
    req = Request(url) # start request
    req.add_header('User-agent','Anaconda 3') # add user agent to request
    fh = urlopen(req)
    im_data = io.BytesIO(fh.read()) # get image
    fh.close() # close url
    img = Image.open(im_data) # open image with PIL
    img = img.convert(self.desired_tile_form) # set image format
    return img, self.tileextent(tile), 'lower' # reformat for cartopy


def plot_gps(lats, lons, zoom):
    cimgt.OSM.get_image = image_spoof # reformat web request for street map spoofing
    osm_img = cimgt.OSM() # spoofed, downloaded street map

    fig = plt.figure(figsize=(12,9)) # open matplotlib figure
    ax1 = plt.axes(projection=osm_img.crs) # project using coordinate reference system (CRS) of street map
    center_pt = [lats[0], lons[0]]
    # adjust to zoom
    extent = [center_pt[1]-(zoom*2.0), center_pt[1]+(zoom*2.0), center_pt[0]-zoom, center_pt[0]+zoom]
    ax1.set_extent(extent) # set extents

    scale = np.ceil(-np.sqrt(2)*np.log(np.divide(zoom, 350.0))) # empirical solve for scale based on zoom
    scale = (scale < 20) and scale or 19 # scale cannot be larger than 19
    ax1.add_image(osm_img, int(scale)) # add OSM with zoom specification
    # NOTE: zoom specifications should be selected based on extent:
    # -- 2     = coarse image, select for worldwide or continental scales
    # -- 4-6   = medium coarseness, select for countries and larger states
    # -- 6-10  = medium fineness, select for smaller states, regions, and cities
    # -- 10-12 = fine image, select for city boundaries and zip codes
    # -- 14+   = extremely fine image, select for roads, blocks, buildings

    ax1.plot(lons, lats, markersize=5, marker='o', linestyle='-', color='#3b3b3b', transform=ccrs.PlateCarree())
    # show the plot
    plt.show()


if __name__ == '__main__':
    print('PRINTING GPS COORDINATES THAT CORRESPOND IN TIME WITH THE IMAGES')
    print('EUROC/ASL FORMAT IS EXPECTED')
    [input_euroc_dir, zoom] = get_command_line_args(sys.argv[1:])
    [df_gps, df_images] = read_data(input_euroc_dir)
    df_extended = associate_gps(df_images, df_gps)
    plot_gps(lats=df_extended['lat'].tolist(), lons=df_extended['lng'].tolist(), zoom=float(zoom))

