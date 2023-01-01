/*
 * <one line to give the program's name and a brief idea of what it does.>
 * Copyright (C) 2016  <copyright holder> <email>
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * 
 */

#ifndef POINTCLOUDMAPPING_H
#define POINTCLOUDMAPPING_H

#include <atomic>
#include "System.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <condition_variable>
#include <opencv2/highgui/highgui.hpp>
#include <KeyFrame.h>
#include "Converter.h"
#include <sys/time.h>


namespace ORB_SLAM3 {
    class PointCloudMapping {
    public:
//        typedef pcl::PointXYZRGBA PointT;
//        typedef pcl::PointCloud<PointT> PointCloud;

        PointCloudMapping(double resolution_, double meanK_, double thresh_);

        void save();

//      插入一个keyframe，会更新一次地图
        void insertKeyFrame(KeyFrame *kf);

        void shutdown();

        void viewer();

        [[maybe_unused]] void inserttu([[maybe_unused]] cv::Mat &color, [[maybe_unused]] cv::Mat &depth, [[maybe_unused]] int idk) {};

        [[maybe_unused]] int mnLoopCount = 0;
        vector<KeyFrame *> cur_vpKFs;
        [[maybe_unused]] bool cloudBusy = false;
        [[maybe_unused]] bool loopBusy = false;

        void updateCloud(Map &curMap);

        void Clear();

        bool bStop = false;

//      关于更新时的变量
        std::atomic<bool> mabIsUpdating;

    protected:
        static void generatePointCloud(KeyFrame *kf);

        std::list<KeyFrame *> mlNewKeyFrames;
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr globalMap;
        shared_ptr<thread> viewerThread;

        bool shutDownFlag = false;
        std::mutex shutDownMutex;

        condition_variable keyFrameUpdated;
        std::mutex mMutexGlobalMap;
//        vector<PointCloud>     pointcloud;
//      data to generate point clouds
        vector<KeyFrame *> keyframes;
        vector<cv::Mat> colorImgs;
        vector<cv::Mat> depthImgs;
        vector<cv::Mat> colorImgks;
        vector<cv::Mat> depthImgks;
        vector<int> ids;
        std::mutex keyframeMutex;
        std::mutex updateMutex;
        [[maybe_unused]] uint16_t lastKeyframeSize = 0;

        double resolution = 0.04;
        double meanK = 50.0;
        double thresh = 1.0;
        pcl::VoxelGrid<pcl::PointXYZRGBA> *voxel;
        pcl::StatisticalOutlierRemoval<pcl::PointXYZRGBA> *statistical_filter;
    };
}
#endif  // POINTCLOUDMAPPING_H
