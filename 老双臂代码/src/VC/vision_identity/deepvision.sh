echo "symlink to eyeToHand_calibration.yaml"
ln -sf /home/robot/robot/VC/vision_calibrate/config/eyeToHand_calibration.yaml /home/robot/robot/VC/vision_identity/eyeToHand_calibration.yaml
#conda activate v5
cd /home/robot/robot/VC/vision_identity
python vision_inference.py
