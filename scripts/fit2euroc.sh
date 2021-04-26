FITFILE='/media/arvc/50324D7B324D6756/datasets/umhVIRB360/euroc/s0/1/2021-03-03-13-09.fit'
FRONTVIDEO='/media/arvc/50324D7B324D6756/datasets/umhVIRB360/euroc/s0/1/2021-03-03-13-09_front.MP4'
BACKVIDEO='/media/arvc/50324D7B324D6756/datasets/umhVIRB360/euroc/s0/1/2021-03-03-13-09_back.MP4'
OUTPUT_DIRECTORY='/media/arvc/50324D7B324D6756/datasets/umhVIRB360/euroc/s0/1/'
SAMPLING=10
python fit2euroc.py --videofront $FRONTVIDEO --videoback $BACKVIDEO --fitfile $FITFILE -e $OUTPUT_DIRECTORY -s $SAMPLING
