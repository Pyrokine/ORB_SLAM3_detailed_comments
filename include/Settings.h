/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef ORB_SLAM3_SETTINGS_H
#define ORB_SLAM3_SETTINGS_H


// Flag to activate the measurement of time in each process (track, local map, place recognition).
// #define REGISTER_TIMES

#include "CameraModels/GeometricCamera.h"

#include <unistd.h>
#include <cstdio>
#include <cstdlib>
#include <string>

namespace ORB_SLAM3 {
    class System;

//  TODO: change to double instead of float

    class Settings {
    public:

//      Enum for the different camera types implemented
        enum CameraType {
            PinHole = 0,
            Rectified = 1,
            KannalaBrandt = 2
        };

//      Delete default constructor
        Settings() = delete;

//      Constructor from file
        Settings(const std::string &configFile, const int &sensor);

//      Ostream operator overloading to dump settings to the terminal
        friend std::ostream &operator<<(std::ostream &output, const Settings &s);

//      Getter methods
        CameraType cameraType() { return cameraType_; }

        GeometricCamera *camera1() { return calibration1_; }

        GeometricCamera *camera2() { return calibration2_; }

        cv::Mat camera1DistortionCoef() { return cv::Mat(int(vPinHoleDistorsion1_.size()), 1, CV_32F, vPinHoleDistorsion1_.data()); }

        cv::Mat camera2DistortionCoef() { return cv::Mat(int(vPinHoleDistorsion2_.size()), 1, CV_32F, vPinHoleDistorsion1_.data()); }

        Sophus::SE3f Tlr() { return Tlr_; }

        [[nodiscard]] float bf() const { return bf_; }

        [[nodiscard]] float b() const { return b_; }

        [[nodiscard]] float thDepth() const { return thDepth_; }

        [[nodiscard]] bool needToUndistort() const { return bNeedToUndistort_; }

        cv::Size newImSize() { return newImSize_; }

        [[nodiscard]] float fps() const { return fps_; }

        [[nodiscard]] bool rgb() const { return bRGB_; }

        [[nodiscard]] bool needToResize() const { return bNeedToResize1_; }

        [[nodiscard]] bool needToRectify() const { return bNeedToRectify_; }

        [[nodiscard]] float noiseGyro() const { return noiseGyro_; }

        [[nodiscard]] float noiseAcc() const { return noiseAcc_; }

        [[nodiscard]] float gyroWalk() const { return gyroWalk_; }

        [[nodiscard]] float accWalk() const { return accWalk_; }

        [[nodiscard]] float imuFrequency() const { return imuFrequency_; }

        Sophus::SE3f Tbc() { return Tbc_; }

        [[nodiscard]] bool insertKFsWhenLost() const { return insertKFsWhenLost_; }

        [[nodiscard]] float depthMapFactor() const { return depthMapFactor_; }

        [[nodiscard]] int nFeatures() const { return nFeatures_; }

        [[nodiscard]] int nLevels() const { return nLevels_; }

        [[nodiscard]] int initThFAST() const { return initThFAST_; }

        [[nodiscard]] int minThFAST() const { return minThFAST_; }

        [[nodiscard]] float scaleFactor() const { return scaleFactor_; }

        [[nodiscard]] float keyFrameSize() const { return keyFrameSize_; }

        [[nodiscard]] float keyFrameLineWidth() const { return keyFrameLineWidth_; }

        [[nodiscard]] float graphLineWidth() const { return graphLineWidth_; }

        [[nodiscard]] float pointSize() const { return pointSize_; }

        [[nodiscard]] float cameraSize() const { return cameraSize_; }

        [[nodiscard]] float cameraLineWidth() const { return cameraLineWidth_; }

        [[nodiscard]] float viewPointX() const { return viewPointX_; }

        [[nodiscard]] float viewPointY() const { return viewPointY_; }

        [[nodiscard]] float viewPointZ() const { return viewPointZ_; }

        [[nodiscard]] float viewPointF() const { return viewPointF_; }

        [[nodiscard]] float imageViewerScale() const { return imageViewerScale_; }

        std::string atlasLoadFile() { return sLoadFrom_; }

        std::string atlasSaveFile() { return sSaveto_; }

        [[nodiscard]] float thFarPoints() const { return thFarPoints_; }

        cv::Mat M1l() { return M1l_; }

        cv::Mat M2l() { return M2l_; }

        cv::Mat M1r() { return M1r_; }

        cv::Mat M2r() { return M2r_; }

    private:
        template<typename T>
        T readParameter(cv::FileStorage &fSettings, const std::string &name, bool &found, const bool required = true) {
            cv::FileNode node = fSettings[name];
            if (node.empty()) {
                if (required) {
                    std::cerr << name << " required parameter does not exist, aborting..." << std::endl;
                    exit(-1);
                } else {
                    std::cerr << name << " optional parameter does not exist..." << std::endl;
                    found = false;
                    return T();
                }
            } else {
                found = true;
                return (T) node;
            }
        }

        void readCamera1(cv::FileStorage &fSettings);

        void readCamera2(cv::FileStorage &fSettings);

        void readImageInfo(cv::FileStorage &fSettings);

        void readIMU(cv::FileStorage &fSettings);

        void readRGBD(cv::FileStorage &fSettings);

        void readORB(cv::FileStorage &fSettings);

        void readViewer(cv::FileStorage &fSettings);

        void readLoadAndSave(cv::FileStorage &fSettings);

        void readOtherParameters(cv::FileStorage &fSettings);

        void precomputeRectificationMaps();

        int sensor_;
        CameraType cameraType_;  // Camera type

//      Visual stuff
        GeometricCamera *calibration1_{}, *calibration2_{};  // Camera calibration
        GeometricCamera *originalCalib1_{}, *originalCalib2_{};
        std::vector<float> vPinHoleDistorsion1_, vPinHoleDistorsion2_;

        cv::Size originalImSize_, newImSize_;
        float fps_{};
        bool bRGB_{};

        bool bNeedToUndistort_;
        bool bNeedToRectify_;
        bool bNeedToResize1_, bNeedToResize2_;

        Sophus::SE3f Tlr_;
        float thDepth_{};
        float bf_{}, b_{};

//      Rectification stuff

        cv::Mat M1l_, M2l_;
        cv::Mat M1r_, M2r_;

//      Inertial stuff
        float noiseGyro_{}, noiseAcc_{};
        float gyroWalk_{}, accWalk_{};
        float imuFrequency_{};
        Sophus::SE3f Tbc_;
        bool insertKFsWhenLost_{};

//      RGBD stuff
        float depthMapFactor_{};

//      ORB stuff
        int nFeatures_{};
        float scaleFactor_{};
        int nLevels_{};
        int initThFAST_{}, minThFAST_{};

//      Viewer stuff
        float keyFrameSize_{};
        float keyFrameLineWidth_{};
        float graphLineWidth_{};
        float pointSize_{};
        float cameraSize_{};
        float cameraLineWidth_{};
        float viewPointX_{}, viewPointY_{}, viewPointZ_{}, viewPointF_{};
        float imageViewerScale_{};

//      Save & load maps
        std::string sLoadFrom_, sSaveto_;

//      Other stuff
        float thFarPoints_{};
    };
}

#endif  // ORB_SLAM3_SETTINGS_H