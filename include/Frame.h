/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef FRAME_H
#define FRAME_H

#include<vector>

#include "MapPoint.h"
#include "Thirdparty/DBoW2/DBoW2/BowVector.h"
#include "Thirdparty/DBoW2/DBoW2/FeatureVector.h"
#include "ORBVocabulary.h"
#include "KeyFrame.h"
#include "ORBextractor.h"

#include <opencv2/opencv.hpp>

namespace ORB_SLAM2
{
#define FRAME_GRID_ROWS 48
#define FRAME_GRID_COLS 64

class MapPoint;
class KeyFrame;

/**
 * @brief: Frame类用于处理单，双，RGBD每一帧图像，完成了特征提取和深度计算等功能
 * 重要变量：mvKeys, mvKeysRight,mDescriptors, mDescriptorsRight,mvDepth,mGrid
 * 该类使用构造函数直接完成包括特征提取和深度计算的所有操作。使用时直接调用相应构造函数即可
 * 存在问题：Undistort函数在对于KeyPoint的去畸变是按照Caltech张正友的方法实现的，去畸变矩阵的意义不清楚
 * 如何去畸变不清楚。
 */
class Frame
{
public:
    Frame();

    // Copy constructor.
    //结构函数--拷贝函数
    Frame(const Frame &frame);

    // Constructor for stereo cameras.
    //构造函数--双目数据
    Frame(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timeStamp, ORBextractor* extractorLeft, ORBextractor* extractorRight, ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth);

    // Constructor for RGB-D cameras.
    //构造函数--RGBD
    Frame(const cv::Mat &imGray, const cv::Mat &imDepth, const double &timeStamp, ORBextractor* extractor,ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth);

    // Constructor for Monocular cameras.
    //构造函数--单目
    Frame(const cv::Mat &imGray, const double &timeStamp, ORBextractor* extractor,ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth);

    // Extract ORB on the image. 0 for left image and 1 for right image.
    //抽特征：flag：0,左图；flag：1,右图
    void ExtractORB(int flag, const cv::Mat &im);

    // Compute Bag of Words representation.
    //计算BoW表示
    void ComputeBoW();

    // Set the camera pose.
    //设定相机姿态矩阵
    void SetPose(cv::Mat Tcw);

    // Computes rotation, translation and camera center matrices from the camera pose.
    //由相机姿态更新状态矩阵
    void UpdatePoseMatrices();

    // Returns the camera center.
    //返回相机中心
    inline cv::Mat GetCameraCenter(){
        return mOw.clone();
    }

    // Returns inverse of rotation
    //返回旋转矩阵的逆
    inline cv::Mat GetRotationInverse(){
        return mRwc.clone();
    }

    // Check if a MapPoint is in the frustum of the camera
    // and fill variables of the MapPoint to be used by the tracking
    //检查一个地图点是不是在相机视野中（椎体）
    bool isInFrustum(MapPoint* pMP, float viewingCosLimit);

    // Compute the cell of a keypoint (return false if outside the grid)
    //计算一个地图点在图像中所处的单元位置
    bool PosInGrid(const cv::KeyPoint &kp, int &posX, int &posY);
    // 检查区域内的特征点，位置(x,y)，半径r
    vector<size_t> GetFeaturesInArea(const float &x, const float  &y, const float  &r, const int minLevel=-1, const int maxLevel=-1) const;

    // Search a match for each keypoint in the left image to a keypoint in the right image.
    // If there is a match, depth is computed and the right coordinate associated to the left keypoint is stored.
    // 搜索左右图的匹配特征点对，计算并存储相应的右坐标值和深度值
    void ComputeStereoMatches();

    // Associate a "right" coordinate to a keypoint if there is valid depth in the depthmap.
    // 由RGBD数据得到立体视觉深度信息和右坐标
    void ComputeStereoFromRGBD(const cv::Mat &imDepth);

    // Backprojects a keypoint (if stereo/depth info available) into 3D world coordinates.
    // 将一个有深度信息的特征点反投影到三维空间
    cv::Mat UnprojectStereo(const int &i);

public:
    // Vocabulary used for relocalization.

    ORBVocabulary* mpORBvocabulary;                                         // 用于relocalization的词汇库

    // Feature extractor. The right is used only in the stereo case.
    ORBextractor* mpORBextractorLeft, *mpORBextractorRight;                 //左右图分别的ORBextractor

    // Frame timestamp.
    double mTimeStamp;                                                      //timestamp

    // Calibration matrix and OpenCV distortion parameters.
    cv::Mat mK;                                                             //标定参数矩阵
    static float fx;
    static float fy;
    static float cx;
    static float cy;
    static float invfx;
    static float invfy;
    cv::Mat mDistCoef;

    // Stereo baseline multiplied by fx.
    float mbf;                                                              //baseline * fx

    // Stereo baseline in meters.
    float mb;                                                               //baseline数值（米）

    // Threshold close/far points. Close points are inserted from 1 view.
    // Far points are inserted as in the monocular case from 2 views.
    float mThDepth;                                                         //区分远点和近点的阈值

    // Number of KeyPoints.
    int N;                                                                  //特征点数量

    // Vector of keypoints (original for visualization) and undistorted (actually used by the system).
    // In the stereo case, mvKeysUn is redundant as images must be rectified.
    // In the RGB-D case, RGB images can be distorted.
    std::vector<cv::KeyPoint> mvKeys, mvKeysRight;                          //原始特征点向量\滤过的特征点向量
    std::vector<cv::KeyPoint> mvKeysUn;                                     //无畸变（或去畸变的）特征点向量

    // Corresponding stereo coordinate and depth for each keypoint.
    // "Monocular" keypoints have a negative value.
    std::vector<float> mvuRight;                                            //双目/rgbd的相应特征点的右图坐标值，单目为负数
    std::vector<float> mvDepth;                                             //双目/rgbd的相应特征点的深度值，单目为负数

    // Bag of Words Vector structures.
    DBoW2::BowVector mBowVec;                                               //BoW向量
    DBoW2::FeatureVector mFeatVec;                                          //特征向量

    // ORB descriptor, each row associated to a keypoint.
    cv::Mat mDescriptors, mDescriptorsRight;                                //ORB特征描述子，每一行对应相应的一个特征点

    // MapPoints associated to keypoints, NULL pointer if no association.
    std::vector<MapPoint*> mvpMapPoints;                                    //与特征点对应的地图点，如果没有为空指针

    // Flag to identify outlier associations.                               //特征点与地图点是否是outlier的表征flag
    std::vector<bool> mvbOutlier;

    // Keypoints are assigned to cells in a grid to reduce matching complexity when projecting MapPoints.
    static float mfGridElementWidthInv;
    static float mfGridElementHeightInv;
    std::vector<std::size_t> mGrid[FRAME_GRID_COLS][FRAME_GRID_ROWS];       //对地图区分grid，每一个cell里面对应的特征点数量

    // Camera pose.
    cv::Mat mTcw;                                                           //相机姿态：public

    // Current and Next Frame id.
    static long unsigned int nNextId;                                       //Frame id
    long unsigned int mnId;

    // Reference Keyframe.
    KeyFrame* mpReferenceKF;                                                //参考的关键帧

    // Scale pyramid info.
    int mnScaleLevels;                                                      //尺度金字塔信息，级数
    float mfScaleFactor;
    float mfLogScaleFactor;
    vector<float> mvScaleFactors;                                           //尺度因子
    vector<float> mvInvScaleFactors;
    vector<float> mvLevelSigma2;
    vector<float> mvInvLevelSigma2;

    // Undistorted Image Bounds (computed once).
    static float mnMinX;                                                    //无畸变区域范围 X-Y（只计算一次）
    static float mnMaxX;
    static float mnMinY;
    static float mnMaxY;

    static bool mbInitialComputations;                                      //是否初始化过计算无畸变区域的flag


private:

    // Undistort keypoints given OpenCV distortion parameters.
    // Only for the RGB-D case. Stereo must be already rectified!
    // (called in the constructor).
    //在RGB-D模式下，对特征点进行去畸变处理，即（x,y）
    void UndistortKeyPoints();

    // Computes image bounds for the undistorted image (called in the constructor).
    void ComputeImageBounds(const cv::Mat &imLeft);

    // Assign keypoints to the grid for speed up feature matching (called in the constructor).
    // 将特征点分配到相应的grid
    void AssignFeaturesToGrid();

    // Rotation, translation and camera center
    cv::Mat mRcw;
    cv::Mat mtcw;
    cv::Mat mRwc;
    cv::Mat mOw; //==mtwc 相机中心的位置
};

}// namespace ORB_SLAM

#endif // FRAME_H
