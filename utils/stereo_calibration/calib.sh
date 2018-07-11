# Argument
# host port swap_camera(0/1) num_frames
pushd ../../src
rm -rf output || true
mkdir -p output/LEFT/
mkdir -p output/RIGHT/
python ObRobbyCalibrateCapture.py $1 $2 $3 $4
popd
rm -rf ../../data/output || true
mv  ../../src/output ../../data/
