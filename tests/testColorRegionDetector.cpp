#include "gtest/gtest.h"

#include <yarp/os/Property.h>

#include <yarp/dev/Drivers.h>
#include <yarp/dev/PolyDriver.h>

#include <yarp/sig/Image.h>
#include <yarp/sig/ImageDraw.h>

#include <ColorDebug.h>

#include "IDetector.hpp"

namespace roboticslab
{

/**
 * @ingroup vision-tests
 * @brief Tests \ref ColorRegionDetector
 */
class ColorRegionDetectorTest : public testing::Test
{

    public:
        virtual void SetUp()
        {
            yarp::os::Property deviceOptions;
            deviceOptions.put("device", "ColorRegionDetector");
            deviceOptions.put("algorithm", "redMinusGreen");

            if(!detectorDevice.open(deviceOptions))
            {
                CD_ERROR("Failed to open ColorRegionDetector device\n");
                return;
            }

            if (!detectorDevice.view(iDetector))
            {
                CD_ERROR("Problems acquiring detector interface\n");
                return;
            }

            yarpImage.resize(300,200);
        }

        virtual void TearDown()
        {
        }


    protected:
        roboticslab::IDetector *iDetector;
        yarp::dev::PolyDriver detectorDevice;
        yarp::sig::ImageOf<yarp::sig::PixelRgb> yarpImage;
};

TEST_F( ColorRegionDetectorTest, ColorRegionDetector1)
{
    yarpImage.zero();
    std::vector<yarp::os::Property> detectedObjects;
    iDetector->detect(yarpImage,detectedObjects);
    ASSERT_EQ(detectedObjects.size(), 0);
}

TEST_F( ColorRegionDetectorTest, ColorRegionDetector2)
{
    yarpImage.zero();
    yarp::sig::draw::addCircle(yarpImage,yarp::sig::PixelRgb(255,0,0),
                               yarpImage.width()/2,yarpImage.height()/2,
                               yarpImage.height()/4); // x, y, radius

    std::vector<yarp::os::Property> detectedObjects;
    iDetector->detect(yarpImage,detectedObjects);
    ASSERT_EQ(detectedObjects.size(), 1);

    ASSERT_TRUE(detectedObjects[0].check("tlx"));
    ASSERT_TRUE(detectedObjects[0].check("brx"));
    ASSERT_TRUE(detectedObjects[0].check("tly"));
    ASSERT_TRUE(detectedObjects[0].check("bry"));

    int cx = (detectedObjects[0].find("tlx").asInt32() + detectedObjects[0].find("brx").asInt32()) / 2;
    int cy = (detectedObjects[0].find("tly").asInt32() + detectedObjects[0].find("bry").asInt32()) / 2;

    ASSERT_NEAR(cx, yarpImage.width()/2, 2);
    ASSERT_NEAR(cy, yarpImage.height()/2, 2);
}

}  // namespace roboticslab
