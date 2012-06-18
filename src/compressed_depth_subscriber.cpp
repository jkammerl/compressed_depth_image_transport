#include "compressed_depth_image_transport/compressed_depth_subscriber.h"
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv/cvwimage.h>
#include <opencv/highgui.h>
#include <opencv2/imgproc/imgproc.hpp>

#include "compressed_depth_image_transport/compression_common.h"

#include <limits>
#include <vector>

using namespace cv;

namespace enc = sensor_msgs::image_encodings;

namespace compressed_depth_image_transport
{

void CompressedDepthSubscriber::internalCallback(const sensor_msgs::CompressedImageConstPtr& message,
                                            const Callback& user_cb)

{

  cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);

  // Copy message header
  cv_ptr->header = message->header;

  // Assign image encoding
  string image_encoding = message->format.substr(0, message->format.find(';'));
  cv_ptr->encoding = image_encoding;

  // Decode message data
  if (message->data.size() > sizeof(ConfigHeader))
  {

    // Read compression type from stream
    ConfigHeader compressionConfig;
    memcpy(&compressionConfig, &message->data[0], sizeof(compressionConfig));

    // Get compressed image data
    const vector<uint8_t> imageData(message->data.begin() + sizeof(compressionConfig), message->data.end());

    // Depth map decoding

    if (message->data.size() > 8)
    {
      float depthQuantA, depthQuantB;

      // Read quantization parameters
      depthQuantA = compressionConfig.depthParam[0];
      depthQuantB = compressionConfig.depthParam[1];

      // Decode image data
      cv::Mat decompressed = cv::imdecode(imageData, CV_LOAD_IMAGE_UNCHANGED);

      if (enc::bitDepth(image_encoding) == 32)
      {

        // Allocate floating point image
        int rows = decompressed.rows;
        int cols = decompressed.cols;
        cv_ptr->image = Mat(rows, cols, CV_32FC1);

        // Depth depth conversion
        MatIterator_<float> itDepthImg = cv_ptr->image.begin<float>(),
                            itDepthImg_end = cv_ptr->image.end<float>();
        MatConstIterator_<unsigned short> itInvDepthImg = decompressed.begin<unsigned short>(),
                                          itInvDepthImg_end = decompressed.end<unsigned short>();

        for (; (itDepthImg != itDepthImg_end) && (itInvDepthImg != itInvDepthImg_end); ++itDepthImg, ++itInvDepthImg)
        {
          // check for NaN & max depth
          if (*itInvDepthImg)
          {
            *itDepthImg = depthQuantA / ((float)*itInvDepthImg - depthQuantB);
          }
          else
          {
            *itDepthImg = std::numeric_limits<float>::quiet_NaN();
          }
        }
      }
      else
      {
        // Decode raw image
        cv_ptr->image = cv::imdecode(imageData, CV_LOAD_IMAGE_UNCHANGED);
      }
    }

    // Publish message to user callback
    user_cb(cv_ptr->toImageMsg());
  }
}

} //namespace compressed_depth_image_transport
