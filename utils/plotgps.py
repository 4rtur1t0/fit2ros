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

    #input_euroc_dir = '/media/arvc/50324D7B324D6756/datasets/umhVIRB360/s1/1/'
    print('Input EUROC dir: ', input_euroc_dir)
    return input_euroc_dir, zoom


def read_gps_data(input_euroc_dir):
    print("Reading data.csv from EUROC directory: ", input_euroc_dir)
    # TODO: add options to access other robots or gps1 gps2...etc.
    csv_filename = input_euroc_dir + '/mav0/gps0/data.csv'
    df = pd.read_csv(csv_filename)
    print(df)
    print('FOUND: ')
    print('latitudes/longitudes: ', len(df['lat']))
    lats = df['lat'].tolist()
    lngs = df['lng'].tolist()
    return lats, lngs


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
    [input_euroc_dir, zoom] = get_command_line_args(sys.argv[1:])
    [lats, lons] = read_gps_data(input_euroc_dir)
    plot_gps(lats=lats, lons=lons, zoom=float(zoom))

