FITFILE='/media/arvc/50324D7B324D6756/datasets/umhVIRB360/garmin_last/2021-03-25-16-41-44.fit'
FRONTVIDEO='/media/arvc/50324D7B324D6756/datasets/umhVIRB360/garmin_last/V6813138.MP4'
BACKVIDEO='/media/arvc/50324D7B324D6756/datasets/umhVIRB360/garmin_last/V6813139.MP4'
OUTPUT_DIRECTORY='/media/arvc/50324D7B324D6756/datasets/umhVIRB360/garmin_last/s1/0'
SAMPLING=10
python fit2euroc.py --videofront $FRONTVIDEO --videoback $BACKVIDEO --fitfile $FITFILE -e $OUTPUT_DIRECTORY -s $SAMPLING
