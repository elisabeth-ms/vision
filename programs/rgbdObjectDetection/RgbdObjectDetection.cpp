// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "RgbdObjectDetection.hpp"

#include <cstdio>
#include <tuple>
#include <vector>

#include <yarp/conf/version.h>
#include <yarp/os/LogComponent.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Property.h>
#include <yarp/sig/ImageDraw.h>

#ifdef HAVE_CROP
# include <yarp/sig/ImageUtils.h>
#endif

constexpr auto DEFAULT_SENSOR_DEVICE = "RGBDSensorClient";
constexpr auto DEFAULT_SENSOR_REMOTE = "/rgbd";
constexpr auto DEFAULT_LOCAL_PREFIX = "/rgbdObjectDetection";
constexpr auto DEFAULT_PERIOD = 0.02; // [s]

#define DEFAULT_MAX_DISTANCE 1.5

using namespace roboticslab;

namespace
{
    YARP_LOG_COMPONENT(RGBD, "rl.RgbdObjectDetection")

    void scaleXY(const yarp::sig::Image & frame1, const yarp::sig::Image & frame2, int px1, int py1, int * px2, int * py2)
    {
        if (frame1.width() != frame2.width() || frame1.height() != frame2.height())
        {
            *px2 = px1 * ((double)frame2.width() / (double)frame1.width());
            *py2 = py1 * ((double)frame2.height() / (double)frame1.height());
        }
        else
        {
            *px2 = px1;
            *py2 = py1;
        }
    }
}

bool RgbdObjectDetection::configure(yarp::os::ResourceFinder &rf)
{
    if (rf.check("help"))
    {
        std::printf("RgbDetection options:\n");
        std::printf("\t--help (this help)\t--from [file.ini]\t--context [path]\n");
        std::printf("\t--sensorDevice (device we create, default: \"%s\")\n", DEFAULT_SENSOR_DEVICE);
        std::printf("\t--sensorRemote (if accesing remote, remote port name, default: \"%s\")\n", DEFAULT_SENSOR_REMOTE);
        std::printf("\t--localPrefix (local port name prefix, default: \"%s\")\n", DEFAULT_LOCAL_PREFIX);
        std::printf("\t--period ([s] default: \"%f\")\n", DEFAULT_PERIOD);
        std::printf("\t--detector (detector device)\n");
        return false;
    }

    yCDebug(RGBD) << "Config:" << rf.toString();

    auto strSensorDevice = rf.check("sensorDevice", yarp::os::Value(DEFAULT_SENSOR_DEVICE)).asString();
    auto strSensorRemote = rf.check("sensorRemote", yarp::os::Value(DEFAULT_SENSOR_REMOTE)).asString();
    auto strLocalPrefix = rf.check("localPrefix", yarp::os::Value(DEFAULT_LOCAL_PREFIX)).asString();
    auto strDetector = rf.check("detector", yarp::os::Value("")).asString();

    period = rf.check("period", yarp::os::Value(DEFAULT_PERIOD)).asFloat64();

    yCInfo(RGBD) << "Using --sensorDevice" << strSensorDevice;
    yCInfo(RGBD) << "Using --sensorRemote" << strSensorRemote;
    yCInfo(RGBD) << "Using --localPrefix" << strLocalPrefix;
    yCInfo(RGBD) << "Using --period" << period;
    yCInfo(RGBD) << "Using --detector" << strDetector;

    yarp::os::Property sensorOptions;
    sensorOptions.fromString(rf.toString());
    sensorOptions.put("device", strSensorDevice);
    sensorOptions.put("localImagePort", strLocalPrefix + "/rgbImage:i");
    sensorOptions.put("localDepthPort", strLocalPrefix + "/depthImage:i");
    sensorOptions.put("localRpcPort", strLocalPrefix + "/rpc:o");
    sensorOptions.put("remoteImagePort", strSensorRemote + "/rgbImage:o");
    sensorOptions.put("remoteDepthPort", strSensorRemote + "/depthImage:o");
    sensorOptions.put("remoteRpcPort", strSensorRemote + "/rpc:i");

    if (!sensorDevice.open(sensorOptions) || !sensorDevice.view(iRGBDSensor))
    {
        yCError(RGBD) << "Unable to initiate camera device";
        return false;
    }

    yarp::os::Property depthParams;

    if (!iRGBDSensor->getDepthIntrinsicParam(depthParams))
    {
        yCError(RGBD) << "Unable to retrieve depth intrinsic parameters";
        return false;
    }

    depthIntrinsicParams.fromProperty(depthParams);

#if YARP_VERSION_MINOR < 5
    // Wait for the first few frames to arrive. We kept receiving invalid pixel codes
    // from the depthCamera device if started straight away.
    // https://github.com/roboticslab-uc3m/vision/issues/88
    yarp::os::Time::delay(1.0);
#endif

    yarp::os::Property detectorOptions;
    detectorOptions.fromString(rf.toString());
    detectorOptions.put("device", strDetector);

    if (!detectorDevice.open(detectorOptions) || !detectorDevice.view(iDetector))
    {
        yCError(RGBD) << "Unable to initiate detector device";
        return false;
    }

    if (!statePort.open(strLocalPrefix + "/state:o"))
    {
        yCError(RGBD) << "Unable to open output state port" << statePort.getName();
        return false;
    }

    if (!imagePort.open(strLocalPrefix + "/img:o"))
    {
        yCError(RGBD) << "Unable to open output image port" << imagePort.getName();
        return false;
    }

    statePort.setWriteOnly();
    imagePort.setWriteOnly();

#ifdef HAVE_CROP
    if (!cropPort.open(strLocalPrefix + "/crop:i"))
    {
        yCError(RGBD) << "Unable to open input crop port" << cropPort.getName();
        return false;
    }

    cropPort.setReadOnly();
    cropPort.useCallback(cropCallback);
#endif

    return true;
}

double RgbdObjectDetection::getPeriod()
{
    return period;
}

bool RgbdObjectDetection::updateModule()
{
    yarp::sig::FlexImage colorFrame;
    yarp::sig::ImageOf<yarp::sig::PixelFloat> depthFrame;

    if (!iRGBDSensor->getImages(colorFrame, depthFrame))
    {
        yCWarning(RGBD) << "Frame acquisition failure";
        return true;
    }

    yarp::sig::ImageOf<yarp::sig::PixelRgb> rgbImage;
    yarp::sig::ImageOf<yarp::sig::PixelRgb> depthFilterRgbImage;
    int offsetX = 0;
    int offsetY = 0;

#ifdef HAVE_CROP
    auto vertices = cropCallback.getVertices();

    if (vertices.size() != 0)
    {
        if (!yarp::sig::utils::cropRect(colorFrame, vertices[0], vertices[1], rgbImage))
        {
            yCWarning(RGBD) << "Crop failed, using full color frame";
            rgbImage.copy(colorFrame);
        }
        else
        {
            offsetX = vertices[0].first;
            offsetY = vertices[0].second;
        }
    }
    else
    {
        rgbImage.copy(colorFrame);
        depthFilterRgbImage.copy(colorFrame);
        // cv::Mat matRgbImage = yarp::cv::toCvMat(depthFilterRgbImage);

        depthFilter(depthFilterRgbImage, depthFrame, DEFAULT_MAX_DISTANCE);

        //cv::cvtColor(matRgbImage, matRgbImage, cv::COLOR_BGR2RGB);

        // depthFilterRgbImage = yarp::cv::fromCvMat<yarp::sig::PixelRgb>(matRgbImage);

        //yarp::sig::file::write(depthFilterRgbImage,"/home/elisabeth/test-2.ppm");
    }
#else
    rgbImage.copy(colorFrame);
#endif

    yarp::os::Bottle detectedObjects;
    yarp::os::Bottle state;


    if (!iDetector->detect(depthFilterRgbImage, detectedObjects))
    {
        yCWarning(RGBD) << "Detector failure";
    }

    if (detectedObjects.size() != 0)
    {
        static const yarp::sig::PixelRgb red(255, 0, 0);
        static const yarp::sig::PixelRgb green(0, 255, 0);

        std::vector<std::tuple<float, int, int, int, int, int, int>> records;
        decltype(records)::value_type * closest = nullptr;

        for (auto i = 0; i < detectedObjects.size(); i++)
        {
            const auto * detectedObject = detectedObjects.get(i).asDict();
            auto tlx = detectedObject->find("tlx").asInt32();
            auto tly = detectedObject->find("tly").asInt32();
            auto brx = detectedObject->find("brx").asInt32();
            auto bry = detectedObject->find("bry").asInt32();

            int pxColor = (tlx + brx) / 2;
            int pyColor = (tly + bry) / 2;

            int width = detectedObject->find("brx").asInt32() - detectedObject->find("tlx").asInt32();
            int height = detectedObject->find("bry").asInt32() - detectedObject->find("tly").asInt32();

            int pxDepth, pyDepth;
            scaleXY(colorFrame, depthFrame, pxColor + offsetX, pyColor + offsetY, &pxDepth, &pyDepth);
            float depth = depthFrame.pixel(pxDepth, pyDepth);

            if (depth > 0.0f)
            {
                double X_tmp =  ( (pxDepth - depthIntrinsicParams.principalPointX) * depth) / depthIntrinsicParams.focalLengthX;
                double Y_tmp =  ( (pyDepth - depthIntrinsicParams.principalPointY) * depth) / depthIntrinsicParams.focalLengthY;

                yarp::os::Property objectsDict;
                objectsDict.put("X", - X_tmp );  // Points right thanks to change sign so (x ^ y = z). Expects --noMirror.
                objectsDict.put("Y", Y_tmp );    // Points down.
                objectsDict.put("Z", depth );    // Points forward.
                objectsDict.put("category", detectedObject->find("category").asString());
                objectsDict.put("confidence", detectedObject->find("confidence").asFloat32());;
                objectsDict.put("tlx", detectedObject->find("tlx").asInt32() );
                objectsDict.put("tly", detectedObject->find("tly").asInt32() );
                objectsDict.put("brx", detectedObject->find("brx").asInt32() );
                objectsDict.put("bry", detectedObject->find("bry").asInt32() );
                state.addDict() = objectsDict;

                yarp::sig::draw::addRectangleOutline(rgbImage, green, pxColor, pyColor, width/2, height/2);

               
            }
        }
        if (state.size() > 0)
        {
            statePort.write(state);
        }

    }

    imagePort.prepare() = rgbImage;
    imagePort.write();
    return true;
}

bool RgbdObjectDetection::interruptModule()
{
    statePort.interrupt();
    imagePort.interrupt();
#ifdef HAVE_CROP
    cropPort.interrupt();
    cropPort.disableCallback();
#endif
    return true;
}

bool RgbdObjectDetection::close()
{
    sensorDevice.close();
    detectorDevice.close();
    statePort.close();
    imagePort.close();
#ifdef HAVE_CROP
    cropPort.close();
#endif
    return true;
}

void RgbdObjectDetection::depthFilter( yarp::sig::ImageOf<yarp::sig::PixelRgb>&colorFrame, yarp::sig::ImageOf<yarp::sig::PixelFloat>& depth, float maxDistance){

    //imshow("Imagen", image);
    //waitKey(1);

    static const yarp::sig::PixelRgb white(0, 0, 0);


    float maxZ = 0, minZ = maxDistance;
    for(int x = 0; x < depth.width(); x++){
        for(int y = 0; y < depth.height(); y++){

            double mmZ_tmp = depth.pixel(int(x), int(y));

            if(mmZ_tmp > maxDistance){
                yarp::sig::draw::addCircle(colorFrame, white, x, y, 5);
            }
            if(mmZ_tmp>maxZ){
                maxZ = mmZ_tmp;
            }
            if(mmZ_tmp<minZ){
                minZ = mmZ_tmp;
            }

        }
    }

}