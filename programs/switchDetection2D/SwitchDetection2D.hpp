// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __SWITCH_DETECTION_2D_HPP__
#define __SWITCH_DETECTION_2D_HPP__

#include <yarp/os/BufferedPort.h>
#include <yarp/os/Port.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/RFModule.h>

#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/FrameGrabberInterfaces.h>

#include <yarp/sig/Image.h>

#include "DetectorThread.hpp"

namespace roboticslab
{
/**
 * @ingroup switchDetection2D
 *
 * @brief Computer Vision detection.
 */
class SwitchDetection2D : public yarp::os::RFModule
{
public:
    ~SwitchDetection2D()
    { close(); }

    bool configure(yarp::os::ResourceFinder &rf);

private:
    DetectorThread detectorThread;

    yarp::dev::PolyDriver cameraDevice;
    yarp::dev::IFrameGrabberImage *camera;

    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > outImg;
    yarp::os::Port outPort;


    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > outCropSelectorImg;
    yarp::os::Port inCropSelectorPort;

    int cropSelector;
    double watchdog;

    bool interruptModule();
    double getPeriod();
    bool updateModule();
    bool close();
};

}  // namespace roboticslab

#endif  // __SWITCH_DETECTION_2D_HPP__
