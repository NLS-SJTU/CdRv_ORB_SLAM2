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


#ifndef TRACKING_H
#define TRACKING_H

#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>

#include "Viewer.h"
#include "FrameDrawer.h"
#include "Map.h"
#include "LocalMapping.h"
#include "LoopClosing.h"
#include "Frame.h"
#include "ORBVocabulary.h"
#include "KeyFrameDatabase.h"
#include "ORBextractor.h"
#include "Initializer.h"
#include "MapDrawer.h"
#include "System.h"

#include <mutex>

namespace ORB_SLAM2
{

class Viewer;
class FrameDrawer;
class Map;
class LocalMapping;
class LoopClosing;
class System;

class Tracking
{  

public:
    // 读入配置文件
    Tracking(System* pSys, ORBVocabulary* pVoc, FrameDrawer* pFrameDrawer, MapDrawer* pMapDrawer, Map* pMap,
             KeyFrameDatabase* pKFDB, const string &strSettingPath, const int sensor);

    // Preprocess the input and call Track(). Extract features and performs stereo matching.
    // 预处理图像并调用主要函数Track()
    cv::Mat GrabImageStereo(const cv::Mat &imRectLeft,const cv::Mat &imRectRight, const double &timestamp);
    cv::Mat GrabImageRGBD(const cv::Mat &imRGB,const cv::Mat &imD, const double &timestamp);
    cv::Mat GrabImageMonocular(const cv::Mat &im, const double &timestamp);

    // 设置指向其他函数(线程)的指针
    void SetLocalMapper(LocalMapping* pLocalMapper);
    void SetLoopClosing(LoopClosing* pLoopClosing);
    void SetViewer(Viewer* pViewer);

    // 对参数变化进行实施校准——（读入新的配置参数）
    // The focal length should be similar or scale prediction will fail when projecting points
    // TODO: 修改 MapPoint::PredictScale() 函数增加对焦距变化导致层数估计的影响
    void ChangeCalibration(const string &strSettingPath);

    // 关闭建图功能时调用此函数
    void InformOnlyTracking(const bool &flag);


public:

    // 枚举变量，表示当前跟踪状态
    enum eTrackingState{
        SYSTEM_NOT_READY=-1,
        NO_IMAGES_YET=0,
        NOT_INITIALIZED=1,
        OK=2,
        LOST=3
    };
    // 系统跟踪状态的记录变量
    eTrackingState mState;
    eTrackingState mLastProcessedState;

    // 输入传感器的种类:MONOCULAR, STEREO, RGBD
    int mSensor;

    // Current Frame
    Frame mCurrentFrame;    // 记录当前帧信息的Frame类
    cv::Mat mImGray;    // 当前左目图的灰度图矩阵

    // Initialization Variables (Monocular)
    // 初始化时前两帧相关变量
    std::vector<int> mvIniLastMatches;
    std::vector<int> mvIniMatches;// 跟踪初始化时前两帧之间的匹配
    std::vector<cv::Point2f> mvbPrevMatched;
    std::vector<cv::Point3f> mvIniP3D;
    Frame mInitialFrame;

    // 在运行最后用于恢复相机位姿的几个List
    // 存储了每一帧的 reference keyframe 和它们相对于Frame 的位姿变换
    list<cv::Mat> mlRelativeFramePoses;     // 位姿列表，记录每一个refKF到CurrF的位姿变换Tcr, 可将refKF的点转换到current的坐标系下
    list<KeyFrame*> mlpReferences;      // 关键帧指针列表，指向各个帧的refKF
    list<double> mlFrameTimes;      // 每一帧的时间戳
    list<bool> mlbLost;     // bool列表，记录每一帧的丢失状态，False表示正常，True表示Lost

    // True表示不建图，只跟踪
    bool mbOnlyTracking;

    // 重置所有进程与数据
    void Reset();

protected:

    // 主进程，包括位姿跟踪，局部地图跟踪和插入关键帧等操作；独立于使用的传感器
    void Track();

    // 双目与RGBD的Map 初始化， 生成MapPoints
    void StereoInitialization();

    // Map initialization for monocular
    void MonocularInitialization();
    void CreateInitialMapMonocular();

    void CheckReplacedInLastFrame();    // 检查并更新上一帧中被Loop Closing替换的MapPoints
    bool TrackReferenceKeyFrame();      // 通过对RefKF对CurrF进行Track，得到当前位姿的估计， 沿用上一帧的位姿作为初值
    void UpdateLastFrame();     // 对上一帧进行投影。找到深度较小的那些三维投影点，以此建立MapPoints（因为MapPointCulling会删掉一些MP）

    bool TrackWithMotionModel();

    bool Relocalization();      // 当Track失败，进行重定位；

    void UpdateLocalMap();      // 更新LocalMap数据(包括哦MP 与 KF)，将mvpLocalMapPoints更新到 mpMap中到mvpReferenceMapPoints中——用于绘图，每次会被覆盖 (被TrackLocalMap()调用)
    void UpdateLocalPoints();   // 更新局部关键点，会清空此前的mvpLocalMapPoints (被UpdateLocalMap()调用)
    void UpdateLocalKeyFrames();    // 更新局部关键帧，即产生新的K1，K2集合与refKF，会清空此前存的KF (被UpdateLocalMap()调用)

    bool TrackLocalMap();       // 跟踪LocalMap数据，根据投影匹配进一步优化位姿，剔除误匹配
    void SearchLocalPoints();       // 寻找LocaMap的LocalMapPoints中 当前帧的可见点，并进行匹配 (被TrackLocalMap()调用)

    bool NeedNewKeyFrame();     // 判断是否需要插入关键帧
    void CreateNewKeyFrame();   // 生成新的关键帧并创建相应的新MapPoints

    // 当前帧与LocalMap匹配的点数很少时为True(表示进行类似VO的工作), 匹配足够多时为False
    bool mbVO;

    // 指向其他线程的指针
    LocalMapping* mpLocalMapper;
    LoopClosing* mpLoopClosing;

    //ORB
    // orb特征提取器，不管单目还是双目，mpORBextractorLeft都要用到
    // 如果是双目，则要用到mpORBextractorRight
    // 如果是单目，在初始化的时候使用mpIniORBextractor而不是mpORBextractorLeft，
    // mpIniORBextractor属性中提取的特征点个数是mpORBextractorLeft的两倍
    ORBextractor* mpORBextractorLeft, *mpORBextractorRight;
    ORBextractor* mpIniORBextractor;

    //BoW
    ORBVocabulary* mpORBVocabulary;     // 指向ORB词汇树的指针
    KeyFrameDatabase* mpKeyFrameDB;     // 指向KeyFrame DataBase实例的指针

    // 单目初始器
    Initializer* mpInitializer;

    //Local Map
    // ORBSLAM的Map就是由KeyFrames与MapPoints组成
    // LocalMap 就是由LocalKeyFrames与LocalMapPoints组成
    KeyFrame* mpReferenceKF;    // 文中提到的参考关键帧，与CurFrame共视程度最高的关键帧
    std::vector<KeyFrame*> mvpLocalKeyFrames;       // 文中K1+K2集合, 是LocalMap中KeyFrames集合
    std::vector<MapPoint*> mvpLocalMapPoints;       // 是LocalMap中的MapPoints集合
    
    // 指向system类用于控制Reset()
    System* mpSystem;
    
    //Drawers
    Viewer* mpViewer;
    FrameDrawer* mpFrameDrawer;
    MapDrawer* mpMapDrawer;

    //Map
    // 指向当前使用的全局地图；ORBSLAM中没有LocalMap类，直接存成mvpLocalKeyFrames和mvpLocalMapPoints
    Map* mpMap;

    //Calibration matrix
    cv::Mat mK;
    cv::Mat mDistCoef;
    float mbf;  // base line/米

    //插入新关键帧的规则参数
    // 帧数间隔参数
    int mMinFrames;
    int mMaxFrames;

    // 远近点的门限参数
    // 近点更为可靠，只要一个KF看到就插入MapPoint;远点要求两个KeyFrames看到才插入 ( ?后者没看到? )
    float mThDepth;

    // For RGB-D inputs only. For some datasets (e.g. TUM) the depthmap values are scaled.
    float mDepthMapFactor;

    //当前帧有效的KP到MP的匹配数
    int mnMatchesInliers;

    //Last Frame, KeyFrame and Relocalisation Info
    KeyFrame* mpLastKeyFrame;   // 产生关键帧时（init和CreateKF时），将新生成的KF赋给了该变量，可能确实并未使用只是提供给大家;或者作者打算开发其他功能
    Frame mLastFrame;       // 记录最近的一个Frame数据，会经常被用到
    unsigned int mnLastKeyFrameId;      // 最近一个KF的 id号（全局id）
    unsigned int mnLastRelocFrameId;    // 最近一次重定位的帧的 id号（全局id）

    //Motion Model
    cv::Mat mVelocity;  // 相邻两帧间的变换矩阵Tcl（T current last）

    //Color order (true RGB, false BGR, ignored if grayscale)
    bool mbRGB;

    list<MapPoint*> mlpTemporalPoints;  // 临时添加的MapPoint，仅仅是为了增加相邻帧Track的可靠性，在创建新KF前会全部删除;
};

} //namespace ORB_SLAM

#endif // TRACKING_H
