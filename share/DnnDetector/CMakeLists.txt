# Copyright: 2021 UC3M
# Author: Elisabeth Menendez
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

yarp_install(FILES conf/DnnDetector.ini
                   conf/coco-object-categories.txt
                   conf/object_detection_classes_yolov3_sharon.txt
                   conf/DnnDetectorSimulation.ini
                   conf/object_detection_classes_yolov3_sharon_simulation.txt
                   conf/yolov3-tiny-custom-simulation.cfg
                   conf/yolov3-tiny-custom-simulation_last.weights
                   conf/DnnDetectorNonBreakable.ini
                   conf/yolov3-tiny-custom-non-breakable.cfg
                   conf/yolov3-tiny-custom-non-breakable_last.weights
                   conf/yolov3-tiny-custom-non-breakable_final.weights
             DESTINATION ${ROBOTICSLAB-VISION_CONTEXTS_INSTALL_DIR}/DnnDetector)

if(DOWNLOAD_MODELS)
    yarp_install(DIRECTORY ${CMAKE_SOURCE_DIR}/models/yolov3-tiny
                 DESTINATION ${ROBOTICSLAB-VISION_CONTEXTS_INSTALL_DIR}/DnnDetector)
endif()

yarp_install(FILES tests/teddy-bear.ppm
             DESTINATION ${ROBOTICSLAB-VISION_CONTEXTS_INSTALL_DIR}/DnnDetector/tests)
