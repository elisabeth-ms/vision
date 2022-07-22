// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __RGBD_OBJECT_DETECTION_HPP__
#define __RGBD_OBJECT_DETECTION_HPP__

#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/RFModule.h>

#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/IRGBDSensor.h>

#include <yarp/sig/Image.h>
#include <yarp/sig/IntrinsicParams.h>

#include "IDetector.hpp"
#ifdef HAVE_CROP
# include "YarpCropCallback.hpp"
#endif

#include "opencv2/opencv.hpp"
#include "yarp/cv/Cv.h"
#include "yarp/sig/all.h"

namespace roboticslab
{

/**
 * @ingroup rgbdObjectDetection
 * @brief 2.5D detection.
 */
class RgbdObjectDetection : public yarp::os::RFModule
{
public:
    ~RgbdObjectDetection()
    { close(); }

    bool configure(yarp::os::ResourceFinder &rf) override;
    double getPeriod() override;
    bool updateModule() override;
    bool interruptModule() override;
    bool close() override;
    void depthFilter( yarp::sig::ImageOf<yarp::sig::PixelRgb>&colorFrame, yarp::sig::ImageOf<yarp::sig::PixelFloat>& depth, float maxDistance);

private:
    yarp::dev::PolyDriver sensorDevice;
    yarp::dev::IRGBDSensor * iRGBDSensor;
    yarp::sig::IntrinsicParams depthIntrinsicParams;

    yarp::dev::PolyDriver detectorDevice;
    IDetector * iDetector;

    yarp::os::Port statePort;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb>> imagePort;

#ifdef HAVE_CROP
    yarp::os::BufferedPort<yarp::os::Bottle> cropPort;
    YarpCropCallback cropCallback;
#endif

    double period;
};

} // namespace roboticslab

#endif // __RGBD_OBJECT_DETECTION_HPP__
