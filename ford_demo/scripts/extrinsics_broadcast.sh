#!/bin/bash

CALIBRATION_DIR=$2
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$DIR/.."

EXTRINSICS_FILES=(
  "${CALIBRATION_DIR}/lidarRed_body.yaml"
  "${CALIBRATION_DIR}/lidarBlue_body.yaml"
  "${CALIBRATION_DIR}/lidarYellow_body.yaml"
  "${CALIBRATION_DIR}/lidarGreen_body.yaml"
  "${CALIBRATION_DIR}/cameraCenter_body.yaml"
  "${CALIBRATION_DIR}/cameraFrontLeft_body.yaml"
  "${CALIBRATION_DIR}/cameraFrontRight_body.yaml"
  "${CALIBRATION_DIR}/cameraSideLeft_body.yaml"
  "${CALIBRATION_DIR}/cameraSideRight_body.yaml"
  "${CALIBRATION_DIR}/cameraRearLeft_body.yaml"
  "${CALIBRATION_DIR}/cameraRearRight_body.yaml"
  "${CALIBRATION_DIR}/imu_body.yaml"
)

function help() {
  echo 'Options:
    publish: broadcast the extrinsic
    clean: extrinsics clean
  '
}

function clean() {
  ps -ef | grep static_transform_publisher | awk '{print $2}' | xargs kill -2
}

function publish() {
  for i in "${EXTRINSICS_FILES[@]}"
  do
    python ${DIR}/extrinsics_broadcaster.py $i &
  done
}

case $1 in
  publish)
    # Clean the previous Transforms
    clean $@

    #Publish new transforms
    publish $@
    ;;
  clean)
    clean $@
    ;;
  *)
    # Clean the previous Transforms
    clean $@

    #Publish new transforms
    publish $@
    ;;
esac
