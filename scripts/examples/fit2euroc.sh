FRONTVIDEO='/home/arvc/Escritorio/datasets/umhVIRB360/calibration/calibration_euroc_13042021/front.MP4'
BACKVIDEO='/home/arvc/Escritorio/datasets/umhVIRB360/calibration/calibration_euroc_13042021/back.MP4'
OUTPUT_DIRECTORY='/home/arvc/Escritorio/datasets/umhVIRB360/calibration/calibration_euroc_13042021'
SAMPLING=10
python fit2euroc.py --videofront $FRONTVIDEO --videoback $BACKVIDEO -e $OUTPUT_DIRECTORY -s $SAMPLING
