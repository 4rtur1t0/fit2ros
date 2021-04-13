CALIBRATION_DIRECTORY='/home/arvc/Escritorio/datasets/umhVIRB360/calibration/calibration_basalt'
FRONTVIDEO='/home/arvc/Escritorio/datasets/umhVIRB360/calibration/calibration_basalt/V2134224_front.MP4'
BACKVIDEO='/home/arvc/Escritorio/datasets/umhVIRB360/calibration/calibration_basalt/V2134225_back.MP4'
SAMPLING=30
python fit2euroc.py -v $FRONTVIDEO -e $CALIBRATION_DIRECTORY/front -s $SAMPLING
python fit2euroc.py -v $BACKVIDEO -e $CALIBRATION_DIRECTORY/back -s $SAMPLING
