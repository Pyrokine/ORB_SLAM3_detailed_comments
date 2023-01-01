/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2020 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
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


#ifndef INITIALIZER_H
#define INITIALIZER_H

#include <opencv2/opencv.hpp>
#include "Frame.h"

#include <unordered_set>

namespace ORB_SLAM3 {
    class Map;

//  THIS IS THE INITIALIZER FOR MONOCULAR SLAM. NOT USED IN THE STEREO OR RGBD CASE.
    class Initializer {
        typedef pair<int, int> Match;

    public:

//      Fix the reference frame
        [[maybe_unused]] explicit Initializer(const Frame &ReferenceFrame, float sigma = 1.0, int iterations = 200);

//      Computes in parallel a fundamental matrix and a homography
//      Selects a model and tries to recover the motion and the structure from motion
        [[maybe_unused]] bool Initialize(const Frame &CurrentFrame, const vector<int> &vMatches12, cv::Mat &R21, cv::Mat &t21, vector<cv::Point3f> &vP3D, vector<bool> &vbTriangulated);

    private:
        [[maybe_unused]] void FindHomography(vector<bool> &vbMatchesInners, float &score, cv::Mat &H21);

        [[maybe_unused]] void FindFundamental(vector<bool> &vbInners, float &score, cv::Mat &F21);

        [[maybe_unused]] static cv::Mat ComputeH21(const vector<cv::Point2f> &vP1, const vector<cv::Point2f> &vP2);

        [[maybe_unused]] static cv::Mat ComputeF21(const vector<cv::Point2f> &vP1, const vector<cv::Point2f> &vP2);

        [[maybe_unused]] float CheckHomography(const cv::Mat &H21, const cv::Mat &H12, vector<bool> &vbMatchesInners, float sigma);

        [[maybe_unused]] float CheckFundamental(const cv::Mat &F21, vector<bool> &vbMatchesInners, float sigma);

        [[maybe_unused]] bool ReconstructF(vector<bool> &vbMatchesInners, cv::Mat &F21, cv::Mat &K,
                          cv::Mat &R21, cv::Mat &t21, vector<cv::Point3f> &vP3D, vector<bool> &vbTriangulated, float minParallax, int minTriangulated);

        [[maybe_unused]] bool ReconstructH(vector<bool> &vbMatchesInners, cv::Mat &H21, cv::Mat &K,
                          cv::Mat &R21, cv::Mat &t21, vector<cv::Point3f> &vP3D, vector<bool> &vbTriangulated, float minParallax, int minTriangulated);

        [[maybe_unused]] static void Triangulate(const cv::KeyPoint &kp1, const cv::KeyPoint &kp2, const cv::Mat &P1, const cv::Mat &P2, cv::Mat &x3D);

        [[maybe_unused]] static void Normalize(const vector<cv::KeyPoint> &vKeys, vector<cv::Point2f> &vNormalizedPoints, cv::Mat &T);
//        void Normalize(const vector<cv::Point2f> &vKeys, vector<cv::Point2f> &vNormalizedPoints, cv::Mat &T);

        [[maybe_unused]] int CheckRT(const cv::Mat &R, const cv::Mat &t, const vector<cv::KeyPoint> &vKeys1, const vector<cv::KeyPoint> &vKeys2,
                    const vector<Match> &vMatches12, vector<bool> &vbInliers,
                    const cv::Mat &K, vector<cv::Point3f> &vP3D, float th2, vector<bool> &vbGood, float &parallax);

        [[maybe_unused]] static void DecomposeE(const cv::Mat &E, cv::Mat &R1, cv::Mat &R2, cv::Mat &t);

//      Key points from Reference Frame (Frame 1)
        [[maybe_unused]] vector<cv::KeyPoint> mvKeys1;

//      Key points from Current Frame (Frame 2)
        [[maybe_unused]] vector<cv::KeyPoint> mvKeys2;

//      Current Matches from Reference to Current
        vector<Match> mvMatches12;
        vector<bool> mvbMatched1;

//      Calibration
        cv::Mat mK;

//      Standard Deviation and Variance
        [[maybe_unused]] float mSigma, mSigma2;

//      Ransac max iterations
        [[maybe_unused]] int mMaxIterations;

//      Ransac sets
        vector<vector<size_t> > mvSets;

        GeometricCamera *mpCamera;
    };
}  // namespace ORB_SLAM

#endif  // INITIALIZER_H
