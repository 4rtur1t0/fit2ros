# change for different runs
RUN="s2/0"
FITFILE="/media/arvc/50324D7B324D6756/datasets/VIRB360/fit/$RUN/fitfile.fit"
FRONTVIDEO="/media/arvc/50324D7B324D6756/datasets/VIRB360/fit/$RUN/front.MP4"
BACKVIDEO="/media/arvc/50324D7B324D6756/datasets/VIRB360/fit/$RUN/back.MP4"
OUTPUT_DIRECTORY="/media/arvc/50324D7B324D6756/datasets/VIRB360/euroc/$RUN/"
EXP_FILE="/media/arvc/50324D7B324D6756/datasets/VIRB360/euroc/$RUN/experiment_info.txt"
SAMPLING=1
python fit2euroc.py --videofront $FRONTVIDEO --videoback $BACKVIDEO --fitfile $FITFILE -e $OUTPUT_DIRECTORY -s $SAMPLING
echo "SAVING EXPERIMENT INFO: "
echo "Experiment info: " > $EXP_FILE
echo $RUN >> $EXP_FILE
echo $FITFILE >> $EXP_FILE
echo $FRONTVIDEO >> $EXP_FILE
echo $BACKVIDEO >> $EXP_FILE
echo $OUTPUT_DIRECTORY >> $EXP_FILE
echo $EXP_FILE >> $EXP_FILE
echo $SAMPLING >> $EXP_FILE

