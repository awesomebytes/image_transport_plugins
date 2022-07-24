/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2012, Willow Garage, Inc.
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#include "compressed_image_transport/compressed_subscriber.h"
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "compressed_image_transport/qoixx.hpp"

#include "compressed_image_transport/compression_common.h"

#include <limits>
#include <vector>

#include <typeinfo>

using namespace cv;

namespace enc = sensor_msgs::image_encodings;

namespace compressed_image_transport
{

void CompressedSubscriber::subscribeImpl(ros::NodeHandle& nh, const std::string& base_topic, uint32_t queue_size,
                             const Callback& callback, const ros::VoidPtr& tracked_object,
                             const image_transport::TransportHints& transport_hints)
{
    typedef image_transport::SimpleSubscriberPlugin<sensor_msgs::CompressedImage> Base;
    Base::subscribeImpl(nh, base_topic, queue_size, callback, tracked_object, transport_hints);

    // Set up reconfigure server for this topic
    reconfigure_server_ = boost::make_shared<ReconfigureServer>(this->nh());
    ReconfigureServer::CallbackType f = boost::bind(&CompressedSubscriber::configCb, this, _1, _2);
    reconfigure_server_->setCallback(f);
}


void CompressedSubscriber::configCb(Config& config, uint32_t level)
{
  config_ = config;
  if (config_.mode == compressed_image_transport::CompressedSubscriber_gray) {
      imdecode_flag_ = cv::IMREAD_GRAYSCALE;
  } else if (config_.mode == compressed_image_transport::CompressedSubscriber_color) {
      imdecode_flag_ = cv::IMREAD_COLOR;
  } else /*if (config_.mode == compressed_image_transport::CompressedSubscriber_unchanged)*/ {
      imdecode_flag_ = cv::IMREAD_UNCHANGED;
  } 
}

void CompressedSubscriber::shutdown()
{
  reconfigure_server_.reset();
  image_transport::SimpleSubscriberPlugin<sensor_msgs::CompressedImage>::shutdown();
}

void CompressedSubscriber::internalCallback(const sensor_msgs::CompressedImageConstPtr& message,
                                            const Callback& user_cb)

{

  cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);
  ROS_INFO_STREAM("Got compressed_subscriber::internal_callback");

  // Copy message header
  cv_ptr->header = message->header;

  // Decode color/mono image
  try
  {
    // Assign image encoding string
    const size_t split_pos = message->format.find(';');
    std::string image_encoding = message->format.substr(0, split_pos);
    std::string compression_format = message->format.substr(split_pos+2, 3);
    ROS_INFO_STREAM("Compression is: '"<< compression_format << "'");
    if (compression_format == "qoi")
    {
      ROS_INFO_STREAM("Processing QOI!");
      // returns std::pair<std::vector<std::byte>, qoixx::qoi::desc>
      auto qoi_decoded = qoixx::qoi::decode<std::vector<uchar>>(message->data);
      ROS_INFO_STREAM("Type of qoi_decoded: " << typeid(qoi_decoded).name());
      // St4pairISt6vectorIhSaIhEEN5qoixx3qoi4descEE
      ROS_INFO_STREAM("Type of qoi_decoded.first: " << typeid(qoi_decoded.first).name());
      // St6vectorIhSaIhEE
      ROS_INFO_STREAM("data size: " << qoi_decoded.first.size());

      int channels = static_cast<int>(qoi_decoded.second.channels);
      int width = qoi_decoded.second.width;
      int height = qoi_decoded.second.height;
      
      // channels can only be 3 (or 4) in QOI
      auto cv_type = CV_8UC3;
      cv_ptr->encoding = enc::BGR8;

      cv::Mat cv_img_from_qoi = cv::Mat(height,
                              width,
                              // TODO: is this correct for all cases? I think QOI deals only with srgb images
                              cv_type,
                              &qoi_decoded.first[0]);
      ROS_INFO_STREAM("cv::Mat step: " << cv_img_from_qoi.step);
      cv_ptr->image = cv_img_from_qoi;
      ROS_INFO_STREAM("Decoded QOI image, cv_ptr->image.rows: "<< cv_ptr->image.rows << " cols: "<< cv_ptr->image.cols
      << "qoi_decoded.second.height: " << qoi_decoded.second.height << " qoi_decoded.second.width: " <<
      qoi_decoded.second.width << 
      " qoi_desc channels: " << static_cast<int>(qoi_decoded.second.channels));

      ROS_INFO_STREAM("total size of cv_img_from_qoi: "<< cv_ptr->image.rows * cv_ptr->image.cols * cv_ptr->image.channels());

    }
    else 
    {

      cv_ptr->image = cv::imdecode(cv::Mat(message->data), imdecode_flag_);

      if (split_pos==std::string::npos)
      {
        // Older version of compressed_image_transport does not signal image format
        switch (cv_ptr->image.channels())
        {
          case 1:
            cv_ptr->encoding = enc::MONO8;
            break;
          case 3:
            cv_ptr->encoding = enc::BGR8;
            break;
          default:
            ROS_ERROR("Unsupported number of channels: %i", cv_ptr->image.channels());
            break;
        }
        } else
        {
          cv_ptr->encoding = image_encoding;

          if ( enc::isColor(image_encoding))
          {
            std::string compressed_encoding = message->format.substr(split_pos);
            bool compressed_bgr_image = (compressed_encoding.find("compressed bgr") != std::string::npos);

            // Revert color transformation
            if (compressed_bgr_image)
            {
              // if necessary convert colors from bgr to rgb
              if ((image_encoding == enc::RGB8) || (image_encoding == enc::RGB16))
                cv::cvtColor(cv_ptr->image, cv_ptr->image, CV_BGR2RGB);

              if ((image_encoding == enc::RGBA8) || (image_encoding == enc::RGBA16))
                cv::cvtColor(cv_ptr->image, cv_ptr->image, CV_BGR2RGBA);

              if ((image_encoding == enc::BGRA8) || (image_encoding == enc::BGRA16))
                cv::cvtColor(cv_ptr->image, cv_ptr->image, CV_BGR2BGRA);
            } else
            {
              // if necessary convert colors from rgb to bgr
              if ((image_encoding == enc::BGR8) || (image_encoding == enc::BGR16))
                cv::cvtColor(cv_ptr->image, cv_ptr->image, CV_RGB2BGR);

              if ((image_encoding == enc::BGRA8) || (image_encoding == enc::BGRA16))
                cv::cvtColor(cv_ptr->image, cv_ptr->image, CV_RGB2BGRA);

              if ((image_encoding == enc::RGBA8) || (image_encoding == enc::RGBA16))
                cv::cvtColor(cv_ptr->image, cv_ptr->image, CV_RGB2RGBA);
            }
          }
        }
      }
  }
  catch (cv::Exception& e)
  {
    ROS_ERROR("%s", e.what());
  }

  size_t rows = cv_ptr->image.rows;
  size_t cols = cv_ptr->image.cols;

  if ((rows > 0) && (cols > 0))
    // Publish message to user callback
    user_cb(cv_ptr->toImageMsg());
}

} //namespace compressed_image_transport
