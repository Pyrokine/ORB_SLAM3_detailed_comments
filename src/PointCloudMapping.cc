/**
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

#include "PointCloudMapping.h"


namespace ORB_SLAM3 {
    PointCloudMapping::PointCloudMapping(double resolution_, double meanK_, double thresh_)
            : mabIsUpdating(false) {
        this->resolution = resolution_;
        this->meanK = meanK_;
        this->thresh = thresh_;
        std::cout << resolution << " " << meanK << " " << thresh << std::endl;
        statistical_filter = new pcl::StatisticalOutlierRemoval<pcl::PointXYZRGBA>(true);
        voxel = new pcl::VoxelGrid<pcl::PointXYZRGBA>();
        statistical_filter->setMeanK(int(meanK));
        statistical_filter->setStddevMulThresh(thresh);
        voxel->setLeafSize(float(resolution), float(resolution), float(resolution));
        globalMap = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBA>);

        viewerThread = make_shared<thread>(bind(&PointCloudMapping::viewer, this));
    }

    void PointCloudMapping::shutdown() {
        {
            unique_lock<mutex> lck(shutDownMutex);
            shutDownFlag = true;
            keyFrameUpdated.notify_one();
        }
        viewerThread->join();
    }

    void PointCloudMapping::Clear() {
        std::cout << "清除稠密地图" << std::endl;
        std::unique_lock<std::mutex> lck(mMutexGlobalMap);
        globalMap.reset(new pcl::PointCloud<pcl::PointXYZRGBA>);
    }

    void PointCloudMapping::insertKeyFrame(KeyFrame *kf) {
//        cout << "receive a keyframe, 第" << kf->mnId << "个" << endl;
        if (kf->imLeftRgb.empty())
            return;
        unique_lock<mutex> lck(keyframeMutex);
        mlNewKeyFrames.emplace_back(kf);
        if (mlNewKeyFrames.size() > 35)
            mlNewKeyFrames.pop_front();

    }

    void PointCloudMapping::generatePointCloud(KeyFrame *kf) {  // ,Eigen::Isometric3d T
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pPointCloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
//      point cloud is null ptr
        for (int m = 0; m < kf->imDepth.rows; m += 3) {
            for (int n = 0; n < kf->imDepth.cols; n += 3) {
                float d = kf->imDepth.ptr<float>(m)[n];
                if (d < 0.05 || d > 10)
                    continue;
                pcl::PointXYZRGBA p;
                p.z = d;
                p.x = (float(n) - kf->cx) * p.z / kf->fx;
                p.y = (float(m) - kf->cy) * p.z / kf->fy;

                p.b = kf->imLeftRgb.ptr<uchar>(m)[n * 3];
                p.g = kf->imLeftRgb.ptr<uchar>(m)[n * 3 + 1];
                p.r = kf->imLeftRgb.ptr<uchar>(m)[n * 3 + 2];

                pPointCloud->points.push_back(p);
            }
        }
        pPointCloud->height = 1;
        pPointCloud->width = pPointCloud->points.size();
        pPointCloud->is_dense = true;
        kf->mptrPointCloud = pPointCloud;
//        cout << pPointCloud->width << endl;
    }

    void PointCloudMapping::viewer() {
        pcl::visualization::CloudViewer viewer("ORB-SLAM3: Dense Map Viewer");
//        KeyFrame *pCurKF;
        while (true) {
            {
                unique_lock<mutex> lck_shutdown(shutDownMutex);
                if (shutDownFlag) {
                    break;
                }
            }

            if (bStop || mabIsUpdating) {
                continue;
            }

            int N;
            std::list<KeyFrame *> lNewKeyFrames;
            {
                unique_lock<mutex> lck(keyframeMutex);
                N = int(mlNewKeyFrames.size());
                lNewKeyFrames = mlNewKeyFrames;
                if (N == 0)
                    continue;
                else {
                    mlNewKeyFrames.clear();
                }
            }

//          初始化
            cout  << "PointCloudMapping - Number of point clouds in queue: " << N << endl;
            double generatePointCloudTime = 0, transformPointCloudTime = 0;
            for (auto pKF: lNewKeyFrames) {
                if (pKF->isBad()) {
                    cout << "PointCloudMapping - New Key Frame is bad" << endl;
                    continue;
                }
                generatePointCloud(pKF);

                pcl::PointCloud<pcl::PointXYZRGBA>::Ptr p(new pcl::PointCloud<pcl::PointXYZRGBA>);
                pcl::transformPointCloud(*(pKF->mptrPointCloud), *(p), pKF->GetPoseInverse().matrix());

                {
                    std::unique_lock<std::mutex> lck(mMutexGlobalMap);
                    *globalMap += *p;
                }
            }
            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZRGBA>);

//          去除孤立点这个比较耗时，用处也不是很大，可以去掉
//            statistical_filter->setInputCloud(globalMap);
//            statistical_filter->filter(*tmp);

//            voxel->setInputCloud(globalMap);
//            voxel->filter(*globalMap);

            viewer.showCloud(globalMap);  // 这个比较费时，建议不需要实时显示的可以屏蔽或改成几次显示一次
        }
    }

//  保存地图的函数，需要的自行调用~
    void PointCloudMapping::save() {
        std::unique_lock<std::mutex> lck(mMutexGlobalMap);
        pcl::io::savePCDFile("result.pcd", *globalMap);
        cout << "globalMap save finished" << endl;
    }

    void PointCloudMapping::updateCloud(Map &curMap) {
        std::unique_lock<std::mutex> lck(updateMutex);

        mabIsUpdating = true;
        cur_vpKFs = curMap.GetAllKeyFrames();
//        loopBusy = true;
        cout << "开始点云更新" << endl;
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr tmpGlobalMap(new pcl::PointCloud<pcl::PointXYZRGBA>);
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr curPointCloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr tmpGlobalMapFilter(new pcl::PointCloud<pcl::PointXYZRGBA>());
        for (auto & cur_vpKF : cur_vpKFs) {
            if (!mabIsUpdating) {
                std::cout << "中断点云更新" << std::endl;
                return;
            }
            if (!cur_vpKF->isBad() && cur_vpKF->mptrPointCloud) {

                pcl::transformPointCloud(
                        *(cur_vpKF->mptrPointCloud), *(curPointCloud),
                        Converter::toMatrix4d(Converter::toCvMat(Converter::toSE3Quat(cur_vpKF->GetPoseInverse()))));
                *tmpGlobalMap += *curPointCloud;

//                voxel->setInputCloud(tmpGlobalMap);
//                voxel->filter(*tmpGlobalMapFilter);
                tmpGlobalMap->swap(*tmpGlobalMapFilter);
            }
        }
        cout << "点云更新完成" << endl;
        {
            std::unique_lock<std::mutex> lck(mMutexGlobalMap);
            globalMap = tmpGlobalMap;
        }
        mabIsUpdating = false;
    }
}