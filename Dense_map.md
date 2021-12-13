# 添加稠密建图部分，如何修改，以及是如何做的

添加一个`PointcloudMapping`类, 在类的构造函数里开一个显示稠密点云的线程，和ORB-SLAM的思路一样，多线程进行处理，然后在运行ORB-SLAM2时，将Tracking生成的Keyframe和彩色图，深度图插入到建图队列中，显示稠密点云的线程从队列中获取彩色图，深度图进行构建。
用到的库： PCL，主要进行点云的生成，坐标变换，滤波和显示。

源文件处：添加`PointcloudMapping.h`
```c++
// PointcloudMapping.h
#include <mutex>
#include <condition_variable>
#include <thread>
#include <queue>
#include <boost/make_shared.hpp>

#include <opencv2/opencv.hpp>
#include <Eigen/Core>  // Eigen核心部分
#include <Eigen/Geometry> // 提供了各种旋转和平移的表示
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/transforms.h>
#include <pcl/common/projection_matrix.h>



#include "KeyFrame.h"
#include "Converter.h"

#ifndef POINTCLOUDMAPPING_H
#define POINTCLOUDMAPPING_H


typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef pcl::PointXYZRGBA PointT; // A point structure representing Euclidean xyz coordinates, and the RGB color.
typedef pcl::PointCloud<PointT> PointCloud;

namespace ORB_SLAM2 {

class Converter;
class KeyFrame;

class PointCloudMapping {
    public:
        PointCloudMapping(double resolution=0.01);
        ~PointCloudMapping();
        void insertKeyFrame(KeyFrame* kf, const cv::Mat& color, const cv::Mat& depth); // 传入的深度图像的深度值单位已经是m
        void requestFinish();
        bool isFinished();
        void getGlobalCloudMap(PointCloud::Ptr &outputMap);

    private:
        void showPointCloud();
        void generatePointCloud(const cv::Mat& imRGB, const cv::Mat& imD, const cv::Mat& pose, int nId); 

        double mCx, mCy, mFx, mFy, mResolution;
        
        std::shared_ptr<std::thread>  viewerThread;
  
        std::mutex mKeyFrameMtx;
        std::condition_variable mKeyFrameUpdatedCond;
        std::queue<KeyFrame*> mvKeyFrames;
        std::queue<cv::Mat> mvColorImgs, mvDepthImgs;

        bool mbShutdown;
        bool mbFinish;

        std::mutex mPointCloudMtx;
        PointCloud::Ptr mPointCloud;

        // filter
        pcl::VoxelGrid<PointT> voxel;
        pcl::StatisticalOutlierRemoval<PointT> statistical_filter;
};

}
#endif
```

```c++
// PointcloudMapping.cc
#include "PointcloudMapping.h"

namespace ORB_SLAM2 {

PointCloudMapping::PointCloudMapping(double resolution)
{
    mResolution = resolution;
    mCx = 0;
    mCy = 0;
    mFx = 0;
    mFy = 0;
    mbShutdown = false;
    mbFinish = false;

    voxel.setLeafSize( resolution, resolution, resolution);
    statistical_filter.setMeanK(50);
    statistical_filter.setStddevMulThresh(1.0); // The distance threshold will be equal to: mean + stddev_mult * stddev

    mPointCloud = boost::make_shared<PointCloud>();  // 用boost::make_shared<>

    viewerThread = std::make_shared<std::thread>(&PointCloudMapping::showPointCloud, this);  // make_unique是c++14的
}

PointCloudMapping::~PointCloudMapping()
{
    viewerThread->join();
}

void PointCloudMapping::requestFinish()
{
    {
        unique_lock<mutex> locker(mKeyFrameMtx);
        mbShutdown = true;
    }
    mKeyFrameUpdatedCond.notify_one();
}

bool PointCloudMapping::isFinished()
{
    return mbFinish;
}

void PointCloudMapping::insertKeyFrame(KeyFrame* kf, const cv::Mat& color, const cv::Mat& depth)
{
    unique_lock<mutex> locker(mKeyFrameMtx);
    mvKeyFrames.push(kf);
    mvColorImgs.push( color.clone() );  // clone()函数进行Mat类型的深拷贝，为什幺深拷贝？？
    mvDepthImgs.push( depth.clone() );

    mKeyFrameUpdatedCond.notify_one();
    cout << "receive a keyframe, id = " << kf->mnId << endl;
}

void PointCloudMapping::showPointCloud() 
{
    pcl::visualization::CloudViewer viewer("Dense pointcloud viewer");
    while(true) {   
        KeyFrame* kf;
        cv::Mat colorImg, depthImg;

        {
            std::unique_lock<std::mutex> locker(mKeyFrameMtx);
            while(mvKeyFrames.empty() && !mbShutdown){  // !mbShutdown为了防止所有关键帧映射点云完成后进入无限等待
                mKeyFrameUpdatedCond.wait(locker); 
            }            
            
            if (!(mvDepthImgs.size() == mvColorImgs.size() && mvKeyFrames.size() == mvColorImgs.size())) {
                std::cout << "这是不应该出现的情况！" << std::endl;
                continue;
            }

            if (mbShutdown && mvColorImgs.empty() && mvDepthImgs.empty() && mvKeyFrames.empty()) {
                break;
            }

            kf = mvKeyFrames.front();
            colorImg = mvColorImgs.front();    
            depthImg = mvDepthImgs.front();    
            mvKeyFrames.pop();
            mvColorImgs.pop();
            mvDepthImgs.pop();
        }

        if (mCx==0 || mCy==0 || mFx==0 || mFy==0) {
            mCx = kf->cx;
            mCy = kf->cy;
            mFx = kf->fx;
            mFy = kf->fy;
        }

        
        {
            std::unique_lock<std::mutex> locker(mPointCloudMtx);
            generatePointCloud(colorImg, depthImg, kf->GetPose(), kf->mnId);
            viewer.showCloud(mPointCloud);
        }
        
        std::cout << "show point cloud, size=" << mPointCloud->points.size() << std::endl;
    }

    // 存储点云
    string save_path = "./resultPointCloudFile.pcd";
    pcl::io::savePCDFile(save_path, *mPointCloud);
    cout << "save pcd files to :  " << save_path << endl;
    mbFinish = true;
}

void PointCloudMapping::generatePointCloud(const cv::Mat& imRGB, const cv::Mat& imD, const cv::Mat& pose, int nId)
{ 
    std::cout << "Converting image: " << nId;
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();     
    PointCloud::Ptr current(new PointCloud);
    for(size_t v = 0; v < imRGB.rows ; v+=3){
        for(size_t u = 0; u < imRGB.cols ; u+=3){
            float d = imD.ptr<float>(v)[u];
            if(d <0.01 || d>10){ // 深度值为0 表示测量失败
                continue;
            }

            PointT p;
            p.z = d;
            p.x = ( u - mCx) * p.z / mFx;
            p.y = ( v - mCy) * p.z / mFy;

            p.b = imRGB.ptr<uchar>(v)[u*3];
            p.g = imRGB.ptr<uchar>(v)[u*3+1];
            p.r = imRGB.ptr<uchar>(v)[u*3+2];

            current->points.push_back(p);
        }        
    }

    Eigen::Isometry3d T = Converter::toSE3Quat( pose );
    PointCloud::Ptr tmp(new PointCloud);
    // tmp为转换到世界坐标系下的点云
    pcl::transformPointCloud(*current, *tmp, T.inverse().matrix()); 

    // depth filter and statistical removal，离群点剔除
    statistical_filter.setInputCloud(tmp);  
    statistical_filter.filter(*current);   
    (*mPointCloud) += *current;

    pcl::transformPointCloud(*mPointCloud, *tmp, T.inverse().matrix());
    // 加入新的点云后，对整个点云进行体素滤波
    voxel.setInputCloud(mPointCloud);
    voxel.filter(*tmp);
    mPointCloud->swap(*tmp);
    mPointCloud->is_dense = true; 

    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    double t = std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count(); 
    std::cout << ", Cost = " << t << std::endl;
}


void PointCloudMapping::getGlobalCloudMap(PointCloud::Ptr &outputMap)
{
    std::unique_lock<std::mutex> locker(mPointCloudMtx);
    outputMap = mPointCloud;
}

}

```
## 其他部分修改
1. 系统入口：`system.h`和`system.cc`

`system.h`中

在`system.h`中添加头文件

    ```c++
    #include "PointcloudMapping.h"
    ```
并且在ORB-SLAM2的命名空间中添加声明：
    ```c++
    class PointCloudMapping
    ```
    
和其他线程一样，都是在ORB-SLAM2下的，因此需要进行添加。

添加`private`的成员
```c++
shared_ptr<PointCloudMapping> mpPointCloudMapping; 
```
之后创建PointCloudMapping对象，使用共享指针make_shared保存，并在初始化Tracking线程时，作为输入参数传入。
```C++
   mpPointCloudMapping = make_shared<PointCloudMapping>( 0.01 );
    //Initialize the Tracking thread
    //(it will live in the main thread of execution, the one that called this constructor)
    mpTracker = new Tracking(this, mpVocabulary, mpFrameDrawer, mpMapDrawer,
                             mpMap, mpKeyFrameDatabase, strSettingsFile, mSensor, mpPointCloudMapping);
```

之后是在Shutdown中加入终止的调用。`mpPointCloudMapping->requestFinish()`，之后需要在while循环中判断`!mpPointCloudMapping->isFinished()`，如果点云地图的构建还未结束，则还不能结束系统。

```c++
void System::Shutdown()
{
    mpLocalMapper->RequestFinish();
    mpLoopCloser->RequestFinish();
    if(mpViewer)
    {
        mpViewer->RequestFinish();
        while(!mpViewer->isFinished())
            usleep(5000);
    }
    
    mpPointCloudMapping->requestFinish();
    // Wait until all thread have effectively stopped
    while(!mpPointCloudMapping->isFinished() || !mpLocalMapper->isFinished() || !mpLoopCloser->isFinished() || mpLoopCloser->isRunningGBA())
    {
        usleep(5000);
    }

    if(mpViewer)
        pangolin::BindToContext("ORB-SLAM2: Map Viewer");
}
```
增加获取稠密点云地图的方法，调用的是`tracking`类的`getPointCloudMap()`方法，并由`outputMap`保存（注意传入的是引用）。
```c++
void System::getPointCloudMap(pcl::PointCloud<pcl::PointXYZRGBA> ::Ptr &outputMap)
{
	mpTracker->getPointCloudMap(outputMap);	
}
```

### 跟踪线程中的修改
`Tracking.h`和`Tracking.cc`文件添加相关`pcl`的头文件：
```c++
// 添加稠密建图部分的头文件,主要是pcl的库
#include "PointcloudMapping.h"

#include <condition_variable>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
```

在`Tracking.h`中添加`protected`成员变量,用于存放地图和彩色图，深度图的
```c++
    cv::Mat mImRGB;
    cv::Mat mImDepth;
    shared_ptr<PointCloudMapping>  mpPointCloudMapping;
```

需要在Tracking线程中传入点云的对象，在构造函数中添加参数，`shared_ptr<PointCloudMapping> pPointCloud`,并且在初始化列表中，添加稠密点云地图的对象指针的初始化`mpPointCloudMapping(pPointCloud)`。在`Tracking：：GrabImageRGBD()`中保存RGB和Depth图像。
```c++
mImRGB=imRGB;
mImDepth=imDepth;
```
在`Tracking：：CreateNewKeyFrame()`中将关键帧插入到点云地图
```c++
mpPointCloudMapping->insertKeyFrame( pKF, this->mImRGB, this->mImDepth );
```

增加获取稠密点云地图的方法，调用的是`PointCloudMapping`类的`getGlobalCloudMap()`方法。
```c++
void Tracking::getPointCloudMap(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &outputMap)
{
	mpPointCloudMapping->getGlobalCloudMap(outputMap);
}
```
# ROS中添加稠密建图
见ROS中的RGBD-dense的代码，这部分目前由于RGBD模式在ROS环境下运行有问题，正在想办法解决。
问题如下：

![orbslam_bug](https://github.com/zhuhu00/img/blob/master/orbslam_bug.gif)

感觉是配置不对，或者是哪里出现了问题。
