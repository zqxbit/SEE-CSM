#pragma once

#include <fstream>

#include <std_srvs/Empty.h>
#include <mutex>
#include <thread>


#include <time.h> 
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include "tf2_msgs/TFMessage.h"
#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/TransformStamped.h>

#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>

#include <pcl_ros/point_cloud.h>
#include <geometry_msgs/Pose.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <tf/transform_broadcaster.h>
#include <tf/message_filter.h>
#include <tf/transform_listener.h>

#include <pcl_ros/transforms.h>
#include <pcl/conversions.h>
#include <pcl_ros/impl/transforms.hpp>

#include "seg_point_type.h"
#include "load_matrix.hpp"


class Semanticmulti {
  public:
    Semanticmulti(ros::NodeHandle& nh,
             double resolution, double block_depth,
             double sf2, double ell,
             int num_class, double free_thresh,
             double occupied_thresh, float var_thresh, 
	           double ds_resolution,
             double free_resolution, double max_range,
             float prior, std::string name,
             std::string frientname, std::string cloudTP, 
             std::string cloudTP_gazebo, int color_num)
      : nh_(nh)
      , resolution_(resolution)
      , num_class_(num_class)
      , ds_resolution_(ds_resolution)
      , free_resolution_(free_resolution)
      , max_range_(max_range) 
      , name_(name)
      , frientname_(frientname)
      , color_num_(color_num){
        local_map_ = new semantic_bki::SemanticBKIOctoMap(resolution, block_depth, num_class, sf2, ell, prior, var_thresh, free_thresh, occupied_thresh);
        global_map_ = new semantic_bki::SemanticBKIOctoMap(resolution, block_depth, num_class, sf2, ell, prior, var_thresh, free_thresh, occupied_thresh);
        m_local_pub_ = new semantic_bki::MarkerArrayPub(nh_, name_+"/local_map", resolution);
        m_global_pub_ = new semantic_bki::MarkerArrayPub(nh_, name_+"/global_map", resolution);

        m_localMapPub = nh_.advertise<sensor_msgs::PointCloud2>(name_+"/map_point", 2, true);  // 发布自己的局部地图点云消息
        m_friendMapSub = nh_.subscribe(frientname_+"/map_point", 1, &Semanticmulti::friendMapCallback, this);   //接收周围地图的点云消息

         // 如果用SemanticKITTI的rosbag
        m_pointCloudSub_kitti = new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh_,"/seg_kitti_velo/pointcloud", 1000);    //接收rosbag中的点云消息
        m_tfPointCloudSub_kitti = new tf::MessageFilter<sensor_msgs::PointCloud2>(*m_pointCloudSub_kitti, m_tfListener, "/world", 1000);  //接收tf和点云之后触发接收  world是frameid
        m_tfPointCloudSub_kitti->registerCallback(boost::bind(&Semanticmulti::insertCloudCallback, this, _1));   //回调函数

        //如果用学长的数据集
        m_pointCloudSub_senior = new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh_, cloudTP, 1000);    //接收rosbag中的点云消息
        m_tfPointCloudSub_senior = new tf::MessageFilter<sensor_msgs::PointCloud2>(*m_pointCloudSub_senior, m_tfListener, "/map", 1000);  //接收tf和点云之后触发接收  world是frameid
        m_tfPointCloudSub_senior->registerCallback(boost::bind(&Semanticmulti::insertCloudCallback2, this, _1));   //回调函数

        m_pointCloudSub_senior_2 = new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh_, "/seg_cloud_2", 1000);    //接收rosbag中的点云消息
        m_tfPointCloudSub_senior_2 = new tf::MessageFilter<sensor_msgs::PointCloud2>(*m_pointCloudSub_senior, m_tfListener, "/map", 1000);  //接收tf和点云之后触发接收  world是frameid
        m_tfPointCloudSub_senior_2->registerCallback(boost::bind(&Semanticmulti::insertCloudCallback2, this, _1));   //回调函数

        //如果用gazebo的数据集
        m_pointCloudSub_gazebo = new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh_, cloudTP_gazebo, 1000);    //接收rosbag中的点云消息
        m_tfPointCloudSub_gazebo = new tf::MessageFilter<sensor_msgs::PointCloud2>(*m_pointCloudSub_gazebo, m_tfListener, "/map", 1000);  //接收tf和点云之后触发接收  world是frameid
        m_tfPointCloudSub_gazebo->registerCallback(boost::bind(&Semanticmulti::insertCloudCallback3, this, _1));   //回调函数

        m_manualTrigerSrv = nh_.advertiseService("manual_triger", &Semanticmulti::manualTriger, this);   //接收人工触发消息

        // m_globalMapViewerTimer = nh_.createTimer(ros::Duration(5.0), &Semanticmulti::pubGlobalMap, this);   //全局地图5s一次，定时器
        m_localMapViewerTimer = nh_.createTimer(ros::Duration(2.0), &Semanticmulti::pubLocalMap, this);   //局部地图2s一次

        count_pc = 0;
        // SemanticKitti的标签标号，对应颜色在markerarray_pub.h中
        label2label =  { 0,  1,  10, 11, 13, 15, 16, 18, 20, 30,  31,  32,  40,  44,  48,  49,  50,
                             51, 52, 60, 70, 71, 72, 80, 81, 99, 252, 256, 253, 254, 255, 257, 258, 259 };
        // 学长数据的标签rgb颜色，对应颜色在markerarray_pub.h中
        labelcolor =  { 4605510, 15999976, 8405120, 12491161, 10066329, 14423100, 6710940, 
                            7048739, 14474240, 20580, 10025880, 7801632, 142, 230, 16711680, 16427550,
                            15460, 70, 4620980};
        std::cout << frientname_ << "to" << name_ ;
        load_matrix(nh_, "friendToMe", friendToMe);
      }

//--------------------------------------------------------------------------------------------------------------------------------------------------------------------//
// -----------------------------------------------------------------------分界线：SemanticKitti回调---------------------------------------------------------//
//--------------------------------------------------------------------------------------------------------------------------------------------------------------------//

      void insertCloudCallback(const sensor_msgs::PointCloud2::ConstPtr &cloud)  //接收到点云和tf之后，根据tf转化，然后回调函数
      {
          semantic_bki::point3f origin;
          pcl::PointCloud<pcl::PointXYZL> pc;
          pcl::PointCloud<pcl::PointXYZL> pc_global;
          pcl::fromROSMsg(*cloud, pc);

          count_pc = count_pc + 1;
          tf::StampedTransform sensorToWorldTf;   //定义存放变换关系的变量
          try
          {
              // 监听两个坐标系之间的变换， 其实就是点云坐标系（什么都行，我们的tf有很多）到世界坐标系
              m_tfListener.lookupTransform("/world", cloud->header.frame_id, cloud->header.stamp, sensorToWorldTf);   //需要从cloud->header.frame_id（left_camera）转化到/world
          }
          catch (tf::TransformException &ex)
          {
              ROS_ERROR_STREAM("Transform error of sensor data: " << ex.what() << ", quitting callback");
              return;
          }

          Eigen::Matrix4f sensorToWorld;
          pcl_ros::transformAsMatrix(sensorToWorldTf, sensorToWorld);   //直接得到矩阵
          pcl::transformPointCloud(pc, pc_global, sensorToWorld);   //得到世界坐标系下的点云
          std::cout<< "---------------------------This is the " << name_ <<"  " <<count_pc  << "st------------------------------"<< std::endl;
          for (int i = 0; i <   pc_global.points.size(); i++)
          {
            std::vector<int>::iterator ite = find(label2label.begin(), label2label.end(), pc_global.points[i].label);
            pc_global.points[i].label = std::distance(std::begin(label2label), ite)+1;
          }
          origin.x() =sensorToWorld(0, 3);  // 这一帧的坐标原点位置
          origin.y() = sensorToWorld(1, 3);
          origin.z() = sensorToWorld(2, 3);
          local_map_->insert_pointcloud(pc_global, origin, ds_resolution_, free_resolution_, max_range_); 
      }

//--------------------------------------------------------------------------------------------------------------------------------------------------------------------//
// -----------------------------------------------------------------------分界线：学长回调----------------------------------------------------------------------//
//--------------------------------------------------------------------------------------------------------------------------------------------------------------------//
      void insertCloudCallback2(const sensor_msgs::PointCloud2::ConstPtr &cloud)  //接收到点云和tf之后，根据tf转化，然后回调函数
      {
          semantic_bki::point3f origin;
          pcl::PointCloud<PointXYZRGBSemanticsBayesian> pc;
          pcl::PointCloud<PointXYZRGBSemanticsBayesian> pc_global;
          pcl::PointCloud<pcl::PointXYZL>::Ptr  pc_con_pt (new pcl::PointCloud<pcl::PointXYZL>);   
          pcl::fromROSMsg(*cloud, pc);

          std::lock_guard<std::mutex> local_map_guard(m_localMapMutex);  //给局部点云上锁，要处理它了
          std::lock_guard<std::mutex> global_map_guard(m_globalMapMutex);  //给局部点云上锁，要处理它了

          count_pc = count_pc + 1;
          tf::StampedTransform sensorToWorldTf;   //定义存放变换关系的变量
          try
          {
              // 监听两个坐标系之间的变换， 其实就是点云坐标系（什么都行，我们的tf有很多）到世界坐标系
              m_tfListener.lookupTransform("/map", cloud->header.frame_id, cloud->header.stamp, sensorToWorldTf);   //需要从cloud->header.frame_id（left_camera）转化到/world
          }
          catch (tf::TransformException &ex)
          {
              ROS_ERROR_STREAM("Transform error of sensor data: " << ex.what() << ", quitting callback");
              return;
          }
          Eigen::Matrix4f sensorToWorld;
          pcl_ros::transformAsMatrix(sensorToWorldTf, sensorToWorld);   //直接得到矩阵
          pcl::transformPointCloud(pc, pc_global, sensorToWorld);   //得到世界坐标系下的点云
          std::cout<< "---------------------------This is the " << name_ <<"  "<<  count_pc  << "st------------------------------"<< std::endl;

          clock_t startTime,endTime;
          startTime = ros::Time::now().toNSec();
          float label_raw;
          std::uint32_t label, rgb;

          for (int i = 0; i <   pc_global.points.size(); i++)
          {
            label_raw = pc_global.points[i].semantic_color1;  //不需要比较，第一类就是最好的
            std::memcpy(&rgb, &label_raw, sizeof(uint32_t));  //得到这个点的语义对应的RGB颜色
            std::vector<uint32_t>::iterator ite = find(labelcolor.begin(), labelcolor.end(), rgb);
            label = std::distance(std::begin(labelcolor), ite)+1;

            pcl::PointXYZL p;
            p.x = pc_global.points[i].x;
            p.y = pc_global.points[i].y;
            p.z = pc_global.points[i].z;
            p.label = label;
            pc_con_pt->points.push_back(p);
          }

          origin.x() =sensorToWorld(0, 3);  // 这一帧的坐标原点位置
          origin.y() = sensorToWorld(1, 3);
          origin.z() = sensorToWorld(2, 3);
          local_map_->insert_pointcloud_test(*pc_con_pt, origin, ds_resolution_, free_resolution_, max_range_); 
          // global_map_->insert_pointcloud_test(*pc_con_pt, origin, ds_resolution_, free_resolution_, max_range_); 
          // std::cout<< name_ <<"  "<< "waitting for next scan...................."<<std::endl;
          // endTime = ros::Time::now().toNSec();
          // if (name_ == "/robot1")
          //   std::cout << "The run time is:" << (double)(endTime - startTime) / 10e6 << "ms" << std::endl;
      }

//--------------------------------------------------------------------------------------------------------------------------------------------------------------------//
// -----------------------------------------------------------------------分界线：Gazebo回调----------------------------------------------------------------------//
//--------------------------------------------------------------------------------------------------------------------------------------------------------------------//

      void insertCloudCallback3(const sensor_msgs::PointCloud2::ConstPtr &cloud)  //接收到点云和tf之后，根据tf转化，然后回调函数
      {
          semantic_bki::point3f origin;
          pcl::PointCloud<pcl::PointXYZL> pc;
          pcl::PointCloud<pcl::PointXYZL> pc_global;
          pcl::fromROSMsg(*cloud, pc);

          count_pc = count_pc + 1;
          tf::StampedTransform sensorToWorldTf;   //定义存放变换关系的变量
          try
          {
              // 监听两个坐标系之间的变换， 其实就是点云坐标系（什么都行，我们的tf有很多）到世界坐标系
              m_tfListener.lookupTransform("/map", cloud->header.frame_id, cloud->header.stamp, sensorToWorldTf);   //需要从cloud->header.frame_id（left_camera）转化到/world
          }
          catch (tf::TransformException &ex)
          {
              ROS_ERROR_STREAM("Transform error of sensor data: " << ex.what() << ", quitting callback");
              return;
          }
          Eigen::Matrix4f sensorToWorld;
          pcl_ros::transformAsMatrix(sensorToWorldTf, sensorToWorld);   //直接得到矩阵
          pcl::transformPointCloud(pc, pc_global, sensorToWorld);   //得到世界坐标系下的点云
          std::cout<< "---------------------------This is the " << name_ <<"  " <<count_pc  << "st------------------------------"<< std::endl;
          origin.x() =sensorToWorld(0, 3);  // 这一帧的坐标原点位置
          origin.y() = sensorToWorld(1, 3);
          origin.z() = sensorToWorld(2, 3);
          clock_t startTime,endTime;
          startTime = ros::Time::now().toNSec();
          local_map_->insert_pointcloud_test(pc_global, origin, ds_resolution_, free_resolution_, max_range_); 
          // global_map_->insert_pointcloud(pc_global, origin, ds_resolution_, free_resolution_, max_range_); 
          endTime = ros::Time::now().toNSec();
          // std::cout << "The run time is:" << (double)(endTime - startTime) / 10e6 << "ms" << std::endl;
      }


    void pubLocalMap(const ros::TimerEvent &event)   //定时发布地图
    {
      std::thread pubLocalMapObj(std::bind(&Semanticmulti::pubLocalMapRviz, this));  //新建立一个发送全局地图消息的线程
      if (pubLocalMapObj.joinable())  //如果线程是结合的，分离出来
      {
          pubLocalMapObj.detach();
      }
    }
    void pubLocalMapRviz( void )
    {
      m_local_pub_->clear_map(resolution_);
      std::lock_guard<std::mutex> local_map_guard(m_localMapMutex);  //给局部点云上锁，要处理它了
      // int i = 0;
      // 把bkioctomap所有的叶节点推进去，没有剪枝操作。begin_leaf()是bkioctomap第一个block第一个节点，end_leaf()是最后一个block最后一个节点
      for (auto it = local_map_->begin_leaf(); it != local_map_->end_leaf(); ++it) {
        // if ( it.get_node().get_state() == semantic_bki::State::OCCUPIED|| it.get_node().get_state() == semantic_bki::State::FREE) {  // 被占用的节点才显示  it.get_node().get_state() == semantic_bki::State::OCCUPIED || it.get_node().get_state() == semantic_bki::State::FREE
        if ( it.get_node().get_state() == semantic_bki::State::OCCUPIED) { 
          semantic_bki::point3f p = it.get_loc();
          m_local_pub_->insert_point3d_semantics(p.x(), p.y(), p.z(), it.get_size(), it.get_node().get_semantics(), color_num_);  //学长为4，kitti为3
          // i++;
        }
      }
      // std::cout << name_ <<" has node num:" << i<<std::endl;
      m_local_pub_->publish();
    }


    void pubGlobalMap(const ros::TimerEvent &event)   //定时发布全局地图
    {
      std::thread pubGlobalMapObj(std::bind(&Semanticmulti::pubGlobalMapRviz, this));  //新建立一个发送全局地图消息的线程
      if (pubGlobalMapObj.joinable())  //如果线程是结合的，分离出来
      {
          pubGlobalMapObj.detach();
      }
    }
    void pubGlobalMapRviz( void )
    {
      m_global_pub_->clear_map(resolution_);
      std::lock_guard<std::mutex> global_map_guard(m_globalMapMutex);  //给全局点云上锁，要处理它了
      int i = 0;
      for (auto it = global_map_->begin_leaf(); it != global_map_->end_leaf(); ++it) {
        // if ( it.get_node().get_state() == semantic_bki::State::OCCUPIED|| it.get_node().get_state() == semantic_bki::State::FREE) {  // 被占用的节点才显示  it.get_node().get_state() == semantic_bki::State::OCCUPIED || it.get_node().get_state() == semantic_bki::State::FREE
        if ( it.get_node().get_state() == semantic_bki::State::OCCUPIED) { 
          semantic_bki::point3f p = it.get_loc();
          m_global_pub_->insert_point3d_semantics(p.x(), p.y(), p.z(), it.get_size(), it.get_node().get_semantics(), color_num_);
          i++;
        }
      }
      m_global_pub_->publish();
      // std::cout << name_ <<" has node num:" << i<<std::endl;
    }


    void pubLocalMapPoint(void)
    {
      pcl::PointCloud<MapPoint>::Ptr  map_point_pt (new pcl::PointCloud<MapPoint>);   
      MapPoint p;
      sensor_msgs::PointCloud2 map_point_msg;
      // 把局部地图转化为点云格式发布出来
      std::lock_guard<std::mutex> local_map_guard(m_localMapMutex);  //给局部点云上锁，要处理它了
      for (auto it = local_map_->begin_leaf(); it != local_map_->end_leaf(); ++it) {
        if ( it.get_node().get_state() == semantic_bki::State::OCCUPIED|| it.get_node().get_state() == semantic_bki::State::FREE) {  
          semantic_bki::point3f p_raw = it.get_loc();
          p.x = p_raw.x();
          p.y = p_raw.y();
          p.z = p_raw.z();
          std::vector<float> scs(num_class_);
          it.get_node().get_ms(scs);

          // 找最大的
          p.semantic_label1 = std::distance(scs.begin(), std::max_element(scs.begin(), scs.end()));  //找到里面最大的那个
          p.prob1 = *std::max_element(scs.begin(), scs.end());
          scs[p.semantic_label1] = 0.0;
          // 找第二大的
          p.semantic_label2 = std::distance(scs.begin(), std::max_element(scs.begin(), scs.end()));  //找到里面最大的那个
          p.prob2 = *std::max_element(scs.begin(), scs.end());
          scs[p.semantic_label2] = 0.0;
          // 找第三大的
          p.semantic_label3= std::distance(scs.begin(), std::max_element(scs.begin(), scs.end()));  //找到里面最大的那个
          p.prob3 = *std::max_element(scs.begin(), scs.end());
          scs[p.semantic_label3] = 0.0;

          std::vector<float> prob(num_class_);
          it.get_node().get_probs(prob);
          p.true_prob = prob[p.semantic_label1];

          map_point_pt->points.push_back(p);
        }
      }
      map_point_pt->width = 1;
      map_point_pt->height = map_point_pt->points.size();
      pcl::toROSMsg( *map_point_pt, map_point_msg);  //将点云转化为消息才能发布
      map_point_msg.header.frame_id = "map"; //帧id改成和velodyne一样的
      m_localMapPub.publish(map_point_msg); //发布调整之后的点云数据，主题为/adjustd_cloud
      std::cout << name_ << " has finshed share local map" <<std::endl;
    }

    // 人工触发的消息回调
    bool manualTriger(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response)
    {
      std::thread PubLocalMapObj(std::bind(&Semanticmulti::pubLocalMapPoint, this));
      if (PubLocalMapObj.joinable())
      {
        PubLocalMapObj.detach();
      }
      return true;
    }

    // 相邻地图的回调函数
    void friendMapCallback(const sensor_msgs::PointCloud2::ConstPtr &cloud_msg)  
    {
      pcl::PointCloud<MapPoint>::Ptr map_point_pt (new pcl::PointCloud<MapPoint>);  
      pcl::fromROSMsg(*cloud_msg, *map_point_pt);   //得到点云的地图
      std::lock_guard<std::mutex> global_map_guard(m_globalMapMutex);  //给全局点云上锁，要处理它了
      insertFriendMapToGlobal(*map_point_pt);   //把这个加入全局地图
      std::cout << name_ <<" has finished BKI for its Gloabl Map" <<std::endl;
    }

    void insertFriendMapToGlobal(pcl::PointCloud<MapPoint> &map_cells)  //得到的点云形式的地图加入进来
    {
      std::cout << name_ <<" has recesived frient map"<<std::endl;
      pcl::PointCloud<MapPoint>::Ptr transformed_cells(new pcl::PointCloud<MapPoint>);
      pcl::transformPointCloud(map_cells, *transformed_cells, friendToMe);  //得到自己坐标下的点云
      global_map_->insert_map_point_single(transformed_cells);    //一对一
      // global_map_->insert_map_point_multi(transformed_cells);    //多对多
       std::thread pubGlobalMapObj(std::bind(&Semanticmulti::pubGlobalMapRviz, this));  //新建立一个发送全局地图消息的线程
      if (pubGlobalMapObj.joinable())  //如果线程是结合的，分离出来
      {
          pubGlobalMapObj.detach();
      }
    }

  private:
    ros::NodeHandle nh_;
    double resolution_;
    double ds_resolution_;
    double free_resolution_;
    double max_range_;
    int num_class_;
    std::string name_;
    std::string frientname_;
    int color_num_;
    semantic_bki::SemanticBKIOctoMap* local_map_;
    semantic_bki::SemanticBKIOctoMap* global_map_;
    semantic_bki::MarkerArrayPub* m_local_pub_;
    semantic_bki::MarkerArrayPub* m_global_pub_;

    ros::Publisher m_localMapPub;  //发布局部地图点云
    ros::Subscriber m_friendMapSub;   //接收周围地图

    // std::vector<float> label_vec;   // 学长的语义颜色float类型
    std::vector<int> label2label;  // semantickitti的标签类别
    std::vector<uint32_t> labelcolor;  // 学长的语义颜色uint32_t类型
    int count_pc;
    // ros::Publisher m_globalcloudPub;  //发布局部地图点云
    message_filters::Subscriber<sensor_msgs::PointCloud2> *m_pointCloudSub_kitti;  //接收点云
    message_filters::Subscriber<sensor_msgs::PointCloud2> *m_pointCloudSub_senior;  //接收点云
    message_filters::Subscriber<sensor_msgs::PointCloud2> *m_pointCloudSub_senior_2;  //接收点云
    message_filters::Subscriber<sensor_msgs::PointCloud2> *m_pointCloudSub_gazebo;  //接收点云
    tf::MessageFilter<sensor_msgs::PointCloud2> *m_tfPointCloudSub_kitti;  //接收/tf消息的过滤器，应该是接收点云和tf同步化
    tf::MessageFilter<sensor_msgs::PointCloud2> *m_tfPointCloudSub_senior;  //接收/tf消息的过滤器，应该是接收点云和tf同步化
    tf::MessageFilter<sensor_msgs::PointCloud2> *m_tfPointCloudSub_senior_2;  //接收/tf消息的过滤器，应该是接收点云和tf同步化
    tf::MessageFilter<sensor_msgs::PointCloud2> *m_tfPointCloudSub_gazebo;  //接收/tf消息的过滤器，应该是接收点云和tf同步化
    tf::TransformListener m_tfListener;  // 转化坐标系

    ros::ServiceServer m_manualTrigerSrv;   //接收人工触发

    std::mutex m_globalMapMutex;    //全局地图互斥锁，防止资源访问冲突，好像没必要？
    std::mutex m_localMapMutex;  //局部地图互斥锁

    Eigen::Matrix4f friendToMe;   //地图之间的转化矩阵

    ros::Timer m_globalMapViewerTimer;   //全局地图定时器
    ros::Timer m_localMapViewerTimer;   //局部地图定时器

};
