# change for different runs
RUN="s2/1"
FITFILE="/media/arvc/50324D7B324D6756/datasets/umhVIRB360/fit/$RUN/fitfile.fit"
FRONTVIDEO="/media/arvc/50324D7B324D6756/datasets/umhVIRB360/fit/$RUN/front.MP4"
BACKVIDEO="/media/arvc/50324D7B324D6756/datasets/umhVIRB360/fit/$RUN/back.MP4"
OUTPUT_DIRECTORY="/media/arvc/50324D7B324D6756/datasets/umhVIRB360/euroc/$RUN/"
SAMPLING=30
python fit2euroc.py --videofront $FRONTVIDEO --videoback $BACKVIDEO --fitfile $FITFILE -e $OUTPUT_DIRECTORY -s $SAMPLING
