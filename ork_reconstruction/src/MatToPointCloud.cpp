/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
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
 *   * Neither the name of Willow Garage, Inc. nor the names of its
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
 *
 */

#include <fstream>
#include <iostream>

#include <boost/foreach.hpp>
#include <boost/shared_ptr.hpp>

#include <ecto/ecto.hpp>

#include <opencv2/core/core.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "conversions.hpp"

using ecto::tendrils;

namespace image_pipeline
{
  namespace conversion
  {
    /** Ecto implementation of a module that takes a point cloud as
        an input and stacks it in a matrix of floats:
        - if the point cloud is organized, the return
          a matrix is width by height with 3 channels (for x, y and z)
        - if the point cloud is unorganized, the
          return a matrix is n_point by 1 with 3 channels (for x, y and z)
     */
    struct MatToPointCloudXYZ
    {
      // Get the original keypoints and point cloud
      typedef pcl::PointXYZ PointType;
      typedef pcl::PointCloud<PointType> CloudType;
      typedef CloudType::ConstPtr CloudOutT;


      static void
      declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
      {
        inputs.declare(&MatToPointCloudXYZ::points3d, "points",
                       "The width by height by 3 channels (x, y and z)");
        outputs.declare(&MatToPointCloudXYZ::cloud_out, "point_cloud",
                        "The XYZ point cloud");
      }

      int
      process(const tendrils& inputs, const tendrils& outputs)
      {
        CloudType::Ptr point_cloud(new CloudType);
        cvToCloud(*points3d, *point_cloud);
        *cloud_out = point_cloud;
        return ecto::OK;
      }
      ecto::spore<cv::Mat> points3d;
      ecto::spore<CloudOutT> cloud_out;
    };
    struct MatToPointCloudXYZOrganized
    {
      // Get the original keypoints and point cloud
      typedef pcl::PointXYZ PointType;
      typedef pcl::PointCloud<PointType> CloudType;
      typedef CloudType::ConstPtr CloudOutT;


      static void
      declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
      {
        inputs.declare(&MatToPointCloudXYZOrganized::points3d, "points", "The width by height by 3 channels (x, y and z)");
        outputs.declare(&MatToPointCloudXYZOrganized::cloud_out, "point_cloud", "The XYZ point cloud");
      }

      int
      process(const tendrils& inputs, const tendrils& outputs)
      {
        CloudType::Ptr point_cloud(new CloudType);
        cvToCloudOrganized(*points3d, *point_cloud);
        *cloud_out = point_cloud;
        return ecto::OK;
      }
      ecto::spore<cv::Mat> points3d;
      ecto::spore<CloudOutT> cloud_out;
    };
    struct MatToPointCloudXYZRGB
    {
      typedef pcl::PointXYZRGB PointType;
      // Get the original keypoints and point cloud
      typedef pcl::PointCloud<PointType> CloudType;
      typedef CloudType::ConstPtr CloudOutT;


      static void
      declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
      {
        inputs.declare(&MatToPointCloudXYZRGB::image, "image", "The rgb image.").required(true);
        inputs.declare(&MatToPointCloudXYZRGB::points3d, "points", "The 3d points.").required(true);
        inputs.declare(&MatToPointCloudXYZRGB::mask, "mask", "The binary mask for valid points.");
        outputs.declare(&MatToPointCloudXYZRGB::cloud_out, "point_cloud", "The XYZRGB point cloud");
      }

      int
      process(const tendrils& i, const tendrils& o)
      {
        //extract the cloud
        CloudType::Ptr cloud(new CloudType);
        cvToCloudXYZRGB(*points3d, *cloud, *image, *mask, false);
        *cloud_out = cloud;
        return ecto::OK;
      }
      ecto::spore<cv::Mat> mask, image, points3d;
      ecto::spore<CloudOutT> cloud_out;

    };
    struct MatToPointCloudXYZRGBOrganized
        {
          // Get the original keypoints and point cloud
          typedef pcl::PointXYZRGB PointType;
          typedef pcl::PointCloud<PointType> CloudType;
          typedef CloudType::ConstPtr CloudOutT;


          static void
          declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
          {
            inputs.declare(&MatToPointCloudXYZRGBOrganized::points3d, "points", "The width by height by 3 channels (x, y and z)").required(true);
            inputs.declare(&MatToPointCloudXYZRGBOrganized::image, "image", "The rgb image.").required(true);
            outputs.declare(&MatToPointCloudXYZRGBOrganized::cloud_out, "point_cloud", "The XYZRGB organized point cloud");
          }

          int
          process(const tendrils& inputs, const tendrils& outputs)
          {
            CloudType::Ptr point_cloud(new CloudType);

            cvToCloudRGBOrganized(*points3d, *image, *point_cloud);
            *cloud_out = point_cloud;
            return ecto::OK;
          }
          ecto::spore<cv::Mat> points3d, image;
          ecto::spore<CloudOutT> cloud_out;
        };
  }
}

ECTO_CELL(object_recognition_reconstruction, image_pipeline::conversion::MatToPointCloudXYZ, "MatToPointCloudXYZ",
          "Given a cv::Mat, convert it to pcl::PointCloud<pcl::PointXYZ>.");
ECTO_CELL(object_recognition_reconstruction, image_pipeline::conversion::MatToPointCloudXYZOrganized, "MatToPointCloudXYZOrganized",
          "Given a cv::Mat, convert it to an organized pcl::PointCloud<pcl::PointXYZ>.");
ECTO_CELL(object_recognition_reconstruction, image_pipeline::conversion::MatToPointCloudXYZRGB, "MatToPointCloudXYZRGB",
          "Given a cv::Mat, convert it to pcl::PointCloud<pcl::PointXYZRGB>.");
ECTO_CELL(object_recognition_reconstruction, image_pipeline::conversion::MatToPointCloudXYZRGBOrganized, "MatToPointCloudXYZRGBOrganized",
          "Given a cv::Mat, convert it to an organized pcl::PointCloud<pcl::PointXYZRGB>.");
