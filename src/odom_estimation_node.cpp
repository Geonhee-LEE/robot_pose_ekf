/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2008, Willow Garage, Inc.
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

/* Author: Wim Meeussen */
/* Modified by Geonhee */

#include <robot_pose_ekf/odom_estimation_node.h>

using namespace MatrixWrapper;
using namespace std;
using namespace ros;
using namespace tf;


static const double EPS = 1e-5;


//#define __EKF_DEBUG_FILE__

namespace estimation
{
  // constructor
  OdomEstimationNode::OdomEstimationNode()
    : odom_active_(false),
      imu_active_(false),
      vo_active_(false),
      gps_active_(false),
      odom_initializing_(false),
      imu_initializing_(false),
      vo_initializing_(false),
      gps_initializing_(false),
      odom_covariance_(6),
      imu_covariance_(3),
      vo_covariance_(6),
      gps_covariance_(3),
      odom_callback_counter_(0),
      imu_callback_counter_(0),
      vo_callback_counter_(0),
      gps_callback_counter_(0),
      ekf_sent_counter_(0)
  {
    ros::NodeHandle nh_private("~");
    ros::NodeHandle nh;

    // paramters
    nh_private.param("output_frame", output_frame_, std::string("output_frame"));
    nh_private.param("base_footprint_frame", base_footprint_frame_, std::string("base_footprint_frame"));
    nh_private.param("sensor_timeout", timeout_, 1.0);
    nh_private.param("odom_used", odom_used_, true);
    nh_private.param("imu_used",  imu_used_, true);
    nh_private.param("vo_used",   vo_used_, true);
    nh_private.param("gps_used",   gps_used_, false);
    nh_private.param("debug",   debug_, false);
    nh_private.param("self_diagnose",  self_diagnose_, false);
    double freq;
    nh_private.param("freq", freq, 30.0);

    tf_prefix_ = tf::getPrefixParam(nh_private);
    output_frame_ = tf::resolve(tf_prefix_, output_frame_);
    base_footprint_frame_ = tf::resolve(tf_prefix_, base_footprint_frame_);

    ROS_INFO_STREAM("output frame: " << output_frame_);
    ROS_INFO_STREAM("base frame: " << base_footprint_frame_);

    // set output frame and base frame names in OdomEstimation filter so that user-defined tf frames are respected
    my_filter_.setOutputFrame(output_frame_);
    my_filter_.setBaseFootprintFrame(base_footprint_frame_);

    timer_ = nh_private.createTimer(ros::Duration(1.0/max(freq,1.0)), &OdomEstimationNode::spin, this);

    // advertise our estimation
    pose_pub_ = nh_private.advertise<geometry_msgs::PoseWithCovarianceStamped>("odom_combined", 10);

    // initialize
    filter_stamp_ = Time::now();

    // subscribe to odom messages
    if (odom_used_){
      ROS_DEBUG("Odom sensor can be used");
      odom_sub_ = nh.subscribe("odom", 10, &OdomEstimationNode::odomCallback, this);
    }
    else ROS_DEBUG("Odom sensor will NOT be used");

    // subscribe to imu messages
    if (imu_used_){
      ROS_DEBUG("Imu sensor can be used");
      imu_sub_ = nh.subscribe("imu_data", 10,  &OdomEstimationNode::imuCallback, this);
    }
    else ROS_DEBUG("Imu sensor will NOT be used");

    // subscribe to vo messages
    if (vo_used_){
      ROS_DEBUG("VO sensor can be used");
      vo_sub_ = nh.subscribe("vo", 10, &OdomEstimationNode::voCallback, this);
    }
    else ROS_DEBUG("VO sensor will NOT be used");

    if (gps_used_){
      ROS_DEBUG("GPS sensor can be used");
      gps_sub_ = nh.subscribe("gps", 10, &OdomEstimationNode::gpsCallback, this);
    }
    else ROS_DEBUG("GPS sensor will NOT be used");


    // publish state service
    state_srv_ = nh_private.advertiseService("get_status", &OdomEstimationNode::getStatus, this);

    if (debug_){
      // open files for debugging
      odom_file_.open("/tmp/odom_file.txt");
      imu_file_.open("/tmp/imu_file.txt");
      vo_file_.open("/tmp/vo_file.txt");
      gps_file_.open("/tmp/gps_file.txt");
      corr_file_.open("/tmp/corr_file.txt");

  
    }
  };




  // destructor
  OdomEstimationNode::~OdomEstimationNode(){

    if (debug_){
      // close files for debugging
      odom_file_.close();
      imu_file_.close();
      gps_file_.close();
      vo_file_.close();
      corr_file_.close();
    }
  };





  // callback function for odom data
  void OdomEstimationNode::odomCallback(const OdomConstPtr& odom)
  {
    odom_callback_counter_++;

    ROS_DEBUG("Odom callback at time %f ", ros::Time::now().toSec());
    assert(odom_used_);

    // receive data 
    odom_stamp_ = odom->header.stamp;
    odom_time_  = Time::now();
    Quaternion q;
    tf::quaternionMsgToTF(odom->pose.pose.orientation, q);	//convert Quaternion msg to Quaternion 
    odom_meas_  = Transform(q, Vector3(odom->pose.pose.position.x, odom->pose.pose.position.y, 0));  // Constructor from Quaternion (optional Vector3 ) 
    for (unsigned int i=0; i<6; i++)
      for (unsigned int j=0; j<6; j++)
        odom_covariance_(i+1, j+1) = odom->pose.covariance[6*i+j];

    //StampedTransform (const tf::Transform &input, const ros::Time &timestamp, const std::string &frame_id, const std::string &child_frame_id)
    my_filter_.addMeasurement(StampedTransform(odom_meas_.inverse(), odom_stamp_, base_footprint_frame_, "odom_meas"), odom_covariance_);
    
    // activate odom
    if (!odom_active_) {
      if (!odom_initializing_){
	odom_initializing_ = true;
	odom_init_stamp_ = odom_stamp_;
	ROS_INFO("Initializing Odom sensor");      
      }
      if ( filter_stamp_ >= odom_init_stamp_){
	odom_active_ = true;
	odom_initializing_ = false;
	ROS_INFO("Odom sensor activated");      
      }
      else ROS_DEBUG("Waiting to activate Odom, because Odom measurements are still %f sec in the future.", 
		    (odom_init_stamp_ - filter_stamp_).toSec());
    }
    
    if (debug_){
      // write to file
      double tmp, yaw;
      odom_meas_.getBasis().getEulerYPR(yaw, tmp, tmp);
      odom_file_<< fixed <<setprecision(5) << ros::Time::now().toSec() << " " << odom_meas_.getOrigin().x() << " " << odom_meas_.getOrigin().y() << "  " << yaw << "  " << endl;
    }
  };




  // callback function for imu data
  void OdomEstimationNode::imuCallback(const ImuConstPtr& imu)
  {
    imu_callback_counter_++;

    assert(imu_used_);

    // receive data 
    imu_stamp_ = imu->header.stamp;
    tf::Quaternion orientation;
    
    quaternionMsgToTF(imu->orientation, orientation);//convert Quaternion msg to Quaternion 
    imu_meas_ = tf::Transform(orientation, tf::Vector3(0,0,0)); //Constructor from Quaternion(optional Vector3)  param: const Quaternion&  q, const Vector3&  c=Vector3(tfScalar(0),tfScalar(0),tfScalar(0)) 
    for (unsigned int i=0; i<3; i++)
      for (unsigned int j=0; j<3; j++)
        imu_covariance_(i+1, j+1) = imu->orientation_covariance[3*i+j];

    // Transforms imu data to base_footprint frame
    if (!robot_state_.waitForTransform(base_footprint_frame_, imu->header.frame_id, imu_stamp_, ros::Duration(0.5))){
      // warn when imu was already activated, not when imu is not active yet
      if (imu_active_)
        ROS_ERROR("Could not transform imu message from %s to %s", imu->header.frame_id.c_str(), base_footprint_frame_.c_str());
      else if (my_filter_.isInitialized())
        ROS_WARN("Could not transform imu message from %s to %s. Imu will not be activated yet.", imu->header.frame_id.c_str(), base_footprint_frame_.c_str());
      else 
        ROS_DEBUG("Could not transform imu message from %s to %s. Imu will not be activated yet.", imu->header.frame_id.c_str(), base_footprint_frame_.c_str());
      return;
    }
    StampedTransform base_imu_offset;
    robot_state_.lookupTransform(base_footprint_frame_, imu->header.frame_id, imu_stamp_, base_imu_offset);
    imu_meas_ = imu_meas_ * base_imu_offset;

    imu_time_  = Time::now();

    // manually set covariance untile imu sends covariance

      SymmetricMatrix measNoiseImu_Cov(3);  measNoiseImu_Cov = 0;
      measNoiseImu_Cov(1,1) = pow(0.00017,2);  // = 0.01 degrees / sec
      measNoiseImu_Cov(2,2) = pow(0.00017,2);  // = 0.01 degrees / sec
      measNoiseImu_Cov(3,3) = pow(0.00017,2);  // = 0.01 degrees / sec
      imu_covariance_ = measNoiseImu_Cov;
    

    my_filter_.addMeasurement(StampedTransform(imu_meas_.inverse(), imu_stamp_, base_footprint_frame_, "imu_meas"), imu_covariance_);
    
    // activate imu
    if (!imu_active_) {
      if (!imu_initializing_){
	imu_initializing_ = true;
	imu_init_stamp_ = imu_stamp_;
	ROS_INFO("Initializing Imu sensor");      
      }
      if ( filter_stamp_ >= imu_init_stamp_){
	imu_active_ = true;
	imu_initializing_ = false;
	ROS_INFO("Imu sensor activated");      
      }
      else ROS_DEBUG("Waiting to activate IMU, because IMU measurements are still %f sec in the future.", 
		    (imu_init_stamp_ - filter_stamp_).toSec());
    }
    
    if (debug_){
      // write to file
      double tmp, yaw;
      imu_meas_.getBasis().getEulerYPR(yaw, tmp, tmp); 
      imu_file_ <<fixed<<setprecision(5)<<ros::Time::now().toSec()<<" "<< yaw << endl;
    }
  };




  // callback function for VO data
  void OdomEstimationNode::voCallback(const VoConstPtr& vo)
  {
    vo_callback_counter_++;

    assert(vo_used_);

    // get data & convert 3D into 2D
    nav_msgs::Odometry vo_2d;
    vo_2d = *vo;
    vo_stamp_ = vo->header.stamp;
    vo_2d.pose.pose.position.z = 0;
    tf2::Quaternion q;
    q.setRPY(0, 0, tf2::getYaw(vo->pose.pose.orientation));
    tf2::convert(q, vo_2d.pose.pose.orientation);
    
    poseMsgToTF(vo_2d.pose.pose, vo_meas_);	//convert Pose msg to TF, tf::Transform vo_meas_ 


    for (unsigned int i=0; i<6; i++)
      for (unsigned int j=0; j<6; j++)
        vo_covariance_(i+1, j+1) = vo->pose.covariance[6*i+j];

    
    // Transforms VO data to base_footprint frame
    if (!robot_state_.waitForTransform(base_footprint_frame_, vo->child_frame_id, vo_stamp_, ros::Duration(0.5))){
     // warn when VO was already activated, not when VO is not active yet
	if (vo_active_)
          ROS_ERROR("Could not transform VO message from %s to %s", vo->child_frame_id.c_str(), base_footprint_frame_.c_str());
        else if (my_filter_.isInitialized())
          ROS_WARN("Could not transform VO message from %s to %s. VO will not be activated yet.", vo->child_frame_id.c_str(), base_footprint_frame_.c_str());
        else 
          ROS_DEBUG("Could not transform VO message from %s to %s. VO will not be activated yet.", vo->child_frame_id.c_str(), base_footprint_frame_.c_str());
        return;
    }

  
    StampedTransform base_vo_offset; 
    robot_state_.lookupTransform( vo->child_frame_id,base_footprint_frame_, vo_stamp_, base_vo_offset);
    vo_meas_ = vo_meas_ * base_vo_offset ;
    vo_time_  = Time::now();


    my_filter_.addMeasurement(StampedTransform(vo_meas_.inverse(), vo_stamp_, base_footprint_frame_, "vo_meas"), vo_covariance_);
    
    // activate vo
    if (!vo_active_) {
      if (!vo_initializing_){
	vo_initializing_ = true;
	vo_init_stamp_ = vo_stamp_;
	ROS_INFO("Initializing Vo sensor");      
      }
      if (filter_stamp_ >= vo_init_stamp_){
	vo_active_ = true;
	vo_initializing_ = false;
	ROS_INFO("Vo sensor activated");      
      }
      else ROS_DEBUG("Waiting to activate VO, because VO measurements are still %f sec in the future.", 
		    (vo_init_stamp_ - filter_stamp_).toSec());
    }
    
    if (debug_){
      // write to file
      double Rx, Ry, Rz;
      vo_meas_.getBasis().getEulerYPR(Rz, Ry, Rx);
      vo_file_ <<fixed<<setprecision(5)<<ros::Time::now().toSec()<<" "<< vo_meas_.getOrigin().x() << " " << vo_meas_.getOrigin().y() << " " << vo_meas_.getOrigin().z() << " "
               << Rx << " " << Ry << " " << Rz << endl;
    }

    /*
      // Transforms VO data wrt base_footprint frame
      tf::Transform transformed_odom;
      geometry_msgs::Pose pose_wrt_base = vo->pose.pose;
      geometry_msgs::Twist twist_wrt_base = vo->twist.twist;

      tf::poseMsgToTF(pose_wrt_base, transformed_odom);
      //quaternionMsgToTF(imu->orientation, orientation);//convert Quaternion msg to Quaternion 
      //transformed_odom  = Transform(q, Vector3(odom->pose.pose.position.x, odom->pose.pose.position.y, 0));  // Constructor from Quaternion (optional Vector3 ) 

      
      tf::StampedTransform vo_wrt_base_offset; 
      robot_state_.lookupTransform(vo->child_frame_id, base_footprint_frame_, vo_stamp_, vo_wrt_base_offset);
      transformed_odom = transformed_odom * vo_wrt_base_offset;

      geometry_msgs::Transform transformed_pose;
      tf::transformTFToMsg(transformed_odom, transformed_pose);
      //ROS_INFO_STREAM("Transform: " << transformed_pose);

      // Twist VO data wrt base_footprint frame
      robot_state_.lookupTwist(vo->child_frame_id, base_footprint_frame_, ros::Time(0), ros::Duration(0.1), twist_wrt_base);
      ROS_INFO_STREAM("Twist: " << twist_wrt_base);
    */
    
    
  };

	/* void Transformer::lookupTwist(const std::string& tracking_frame, const std::string& observation_frame, const ros::Time& time, const ros::Duration& averaging_interval, geometry_msgs::Twist& twist) const
    { 
    // ref point is origin of tracking_frame, ref_frame = obs_frame
    lookupTwist(tracking_frame, observation_frame, observation_frame, tf::Point(0,0,0), tracking_frame, time, averaging_interval, twist);
    }
	*/

	/* void Transformer::lookupTwist(const std::string& tracking_frame, const std::string& observation_frame, const std::string& reference_frame,
		            const tf::Point & reference_point, const std::string& reference_point_frame, 
		            const ros::Time& time, const ros::Duration& averaging_interval, geometry_msgs::Twist& twist) const
	   {
	     ros::Time latest_time, target_time;
	     getLatestCommonTime(observation_frame, tracking_frame, latest_time, NULL); 
	   
	     if (ros::Time() == time)
	       target_time = latest_time;
	     else
	       target_time = time;
	   
	     ros::Time end_time = std::min(target_time + averaging_interval *0.5 , latest_time);

	     ros::Time start_time = std::max(ros::Time().fromSec(.00001) + averaging_interval, end_time) - averaging_interval;  // don't collide with zero
	     ros::Duration corrected_averaging_interval = end_time - start_time; //correct for the possiblity that start time was truncated above.
	     StampedTransform start, end;
	     lookupTransform(observation_frame, tracking_frame, start_time, start);
	     lookupTransform(observation_frame, tracking_frame, end_time, end); 
	   
	     tf::Matrix3x3 temp = start.getBasis().inverse() * end.getBasis();
	     tf::Quaternion quat_temp;
	     temp.getRotation(quat_temp);
	     tf::Vector3 o = start.getBasis() * quat_temp.getAxis();
	     tfScalar ang = quat_temp.getAngle();
	     
	     double delta_x = end.getOrigin().getX() - start.getOrigin().getX();
	     double delta_y = end.getOrigin().getY() - start.getOrigin().getY();
	     double delta_z = end.getOrigin().getZ() - start.getOrigin().getZ();	   
	   
	     tf::Vector3 twist_vel ((delta_x)/corrected_averaging_interval.toSec(), (delta_y)/corrected_averaging_interval.toSec(), (delta_z)/corrected_averaging_interval.toSec());
	     tf::Vector3 twist_rot = o * (ang / corrected_averaging_interval.toSec());
	   	   
	     // This is a twist w/ reference frame in observation_frame  and reference point is in the tracking_frame at the origin (at start_time)	   
	     //correct for the position of the reference frame
	     tf::StampedTransform inverse;
	     lookupTransform(reference_frame,tracking_frame,  target_time, inverse);
	     tf::Vector3 out_rot = inverse.getBasis() * twist_rot;
	     tf::Vector3 out_vel = inverse.getBasis()* twist_vel + inverse.getOrigin().cross(out_rot);
	   	   
	     //Rereference the twist about a new reference point
	     // Start by computing the original reference point in the reference frame:
	     tf::Stamped<tf::Point> rp_orig(tf::Point(0,0,0), target_time, tracking_frame);
	     transformPoint(reference_frame, rp_orig, rp_orig);
	     // convert the requrested reference point into the right frame
	     tf::Stamped<tf::Point> rp_desired(reference_point, target_time, reference_point_frame);
	     transformPoint(reference_frame, rp_desired, rp_desired);
	     // compute the delta
	     tf::Point delta = rp_desired - rp_orig;
	     // Correct for the change in reference point 
	     out_vel = out_vel + out_rot * delta;
	     // out_rot unchanged   
	     twist.linear.x =  out_vel.x();
	     twist.linear.y =  out_vel.y();
	     twist.linear.z =  out_vel.z();
	     twist.angular.x =  out_rot.x();
	     twist.angular.y =  out_rot.y();
	     twist.angular.z =  out_rot.z();
	};*/

  void OdomEstimationNode::gpsCallback(const GpsConstPtr& gps)
  {
    gps_callback_counter_++;

    assert(gps_used_);

    // get data
    gps_stamp_ = gps->header.stamp;
    gps_time_  = Time::now();
    poseMsgToTF(gps->pose.pose, gps_meas_);
    for (unsigned int i=0; i<3; i++)
      for (unsigned int j=0; j<3; j++)
        gps_covariance_(i+1, j+1) = gps->pose.covariance[6*i+j];
    my_filter_.addMeasurement(StampedTransform(gps_meas_.inverse(), gps_stamp_, base_footprint_frame_, "gps_meas"), gps_covariance_);
    
    // activate gps
    if (!gps_active_) {
      if (!gps_initializing_){
	    gps_initializing_ = true;
	    gps_init_stamp_ = gps_stamp_;
	    ROS_INFO("Initializing GPS sensor");      
      }
      if (filter_stamp_ >= gps_init_stamp_){
	    gps_active_ = true;
	    gps_initializing_ = false;
	    ROS_INFO("GPS sensor activated");      
      }
      else ROS_DEBUG("Waiting to activate GPS, because GPS measurements are still %f sec in the future.", 
		    (gps_init_stamp_ - filter_stamp_).toSec());
    }
    
    if (debug_){
      // write to file
      gps_file_ <<fixed<<setprecision(5)<<ros::Time::now().toSec()<<" "<< gps_meas_.getOrigin().x() << " " << gps_meas_.getOrigin().y() << " " << gps_meas_.getOrigin().z() <<endl;
    }
  };



  // filter loop
  void OdomEstimationNode::spin(const ros::TimerEvent& e)
  {
    ROS_DEBUG("Spin function at time %f", ros::Time::now().toSec());

    // check for timing problems
    if ( (odom_initializing_ || odom_active_) && (imu_initializing_ || imu_active_) ){
      double diff = fabs( Duration(odom_stamp_ - imu_stamp_).toSec() );
      if (diff > 1.0) ROS_ERROR("Timestamps of odometry and imu are %f seconds apart.", diff);
    }
    
    // initial value for filter stamp; keep this stamp when no sensors are active
    filter_stamp_ = Time::now();
    
    // check which sensors are still active
    if ((odom_active_ || odom_initializing_) && 
        (Time::now() - odom_time_).toSec() > timeout_){
      odom_active_ = false; odom_initializing_ = false;
      ROS_INFO("Odom sensor not active any more");
    }
    if ((imu_active_ || imu_initializing_) && 
        (Time::now() - imu_time_).toSec() > timeout_){
      imu_active_ = false;  imu_initializing_ = false;
      ROS_INFO("Imu sensor not active any more");
    }
    if ((vo_active_ || vo_initializing_) && 
        (Time::now() - vo_time_).toSec() > timeout_){
      vo_active_ = false;  vo_initializing_ = false;
      ROS_INFO("VO sensor not active any more");
    }

    if ((gps_active_ || gps_initializing_) && 
        (Time::now() - gps_time_).toSec() > timeout_){
      gps_active_ = false;  gps_initializing_ = false;
      ROS_INFO("GPS sensor not active any more");
    }

    
    // only update filter when one of the sensors is active
    if (odom_active_ || imu_active_ || vo_active_ || gps_active_){
      
      // update filter at time where all sensor measurements are available
      if (odom_active_)  filter_stamp_ = min(filter_stamp_, odom_stamp_);
      if (imu_active_)   filter_stamp_ = min(filter_stamp_, imu_stamp_);
      if (vo_active_)    filter_stamp_ = min(filter_stamp_, vo_stamp_);
      if (gps_active_)  filter_stamp_ = min(filter_stamp_, gps_stamp_);

      
      // update filter
      if ( my_filter_.isInitialized() )  {
        bool diagnostics = true;
        if (my_filter_.update(odom_active_, imu_active_,gps_active_, vo_active_,  filter_stamp_, diagnostics)){
          
          // output most recent estimate and relative covariance
          my_filter_.getEstimate(output_);
          pose_pub_.publish(output_);
          ekf_sent_counter_++;
          
          // broadcast most recent estimate to TransformArray
          StampedTransform tmp;
          my_filter_.getEstimate(ros::Time(), tmp);
          if(!vo_active_ && !gps_active_)
            tmp.getOrigin().setZ(0.0);
          odom_broadcaster_.sendTransform(StampedTransform(tmp, tmp.stamp_, output_frame_, base_footprint_frame_));
          
          if (debug_){
            // write to file
            ColumnVector estimate; 
            my_filter_.getEstimate(estimate);
            corr_file_ << fixed << setprecision(5)<<ros::Time::now().toSec()<<" ";
            
            for (unsigned int i=1; i<=6; i++)
            corr_file_ << estimate(i) << " ";
            corr_file_ << endl;
          }
        }
        if (self_diagnose_ && !diagnostics)
          ROS_WARN("Robot pose ekf diagnostics discovered a potential problem");
      }


      // initialize filer with odometry frame
      if (imu_active_ && gps_active_ && !my_filter_.isInitialized()) {
	Quaternion q = imu_meas_.getRotation();
        Vector3 p = gps_meas_.getOrigin();
        Transform init_meas_ = Transform(q, p);
        my_filter_.initialize(init_meas_, gps_stamp_);
        ROS_INFO("Kalman filter initialized with gps and imu measurement");
      }	
      else if ( odom_active_ && gps_active_ && !my_filter_.isInitialized()) {
	Quaternion q = odom_meas_.getRotation();
        Vector3 p = gps_meas_.getOrigin();
        Transform init_meas_ = Transform(q, p);
        my_filter_.initialize(init_meas_, gps_stamp_);
        ROS_INFO("Kalman filter initialized with gps and odometry measurement");
      }
      else if ( vo_active_ && gps_active_ && !my_filter_.isInitialized()) {
	Quaternion q = vo_meas_.getRotation();
        Vector3 p = gps_meas_.getOrigin();
        Transform init_meas_ = Transform(q, p);
        my_filter_.initialize(init_meas_, gps_stamp_);
        ROS_INFO("Kalman filter initialized with gps and visual odometry measurement");
      }
      else if ( odom_active_  && vo_active_ && !my_filter_.isInitialized()){
        my_filter_.initialize(odom_meas_, odom_stamp_);
        ROS_INFO("Kalman filter initialized with odom measurement and visual odometry measurement");
      }
      else if ( odom_active_  && !gps_used_ && !my_filter_.isInitialized()){
        my_filter_.initialize(odom_meas_, odom_stamp_);
        ROS_INFO("Kalman filter initialized with odom measurement");
      }
      else if ( vo_active_ && !gps_used_ && !my_filter_.isInitialized()){
        my_filter_.initialize(vo_meas_, vo_stamp_);
        ROS_INFO("Kalman filter initialized with vo measurement");
      }
    }
  };


bool OdomEstimationNode::getStatus(robot_pose_ekf::GetStatus::Request& req, robot_pose_ekf::GetStatus::Response& resp)
{
  stringstream ss;
  ss << "Input:" << endl;
  ss << " * Odometry sensor" << endl;
  ss << "   - is "; if (!odom_used_) ss << "NOT "; ss << "used" << endl;
  ss << "   - is "; if (!odom_active_) ss << "NOT "; ss << "active" << endl;
  ss << "   - received " << odom_callback_counter_ << " messages" << endl;
  ss << "   - listens to topic " << odom_sub_.getTopic() << endl;
  ss << " * IMU sensor" << endl;
  ss << "   - is "; if (!imu_used_) ss << "NOT "; ss << "used" << endl;
  ss << "   - is "; if (!imu_active_) ss << "NOT "; ss << "active" << endl;
  ss << "   - received " << imu_callback_counter_ << " messages" << endl;
  ss << "   - listens to topic " << imu_sub_.getTopic() << endl;
  ss << " * Visual Odometry sensor" << endl;
  ss << "   - is "; if (!vo_used_) ss << "NOT "; ss << "used" << endl;
  ss << "   - is "; if (!vo_active_) ss << "NOT "; ss << "active" << endl;
  ss << "   - received " << vo_callback_counter_ << " messages" << endl;
  ss << "   - listens to topic " << vo_sub_.getTopic() << endl;
  ss << " * GPS sensor" << endl;
  ss << "   - is "; if (!gps_used_) ss << "NOT "; ss << "used" << endl;
  ss << "   - is "; if (!gps_active_) ss << "NOT "; ss << "active" << endl;
  ss << "   - received " << gps_callback_counter_ << " messages" << endl;
  ss << "   - listens to topic " << gps_sub_.getTopic() << endl;
  ss << "Output:" << endl;
  ss << " * Robot pose ekf filter" << endl;
  ss << "   - is "; if (!my_filter_.isInitialized()) ss << "NOT "; ss << "active" << endl;
  ss << "   - sent " << ekf_sent_counter_ << " messages" << endl;
  ss << "   - pulishes on topics " << pose_pub_.getTopic() << " and /tf" << endl;
  resp.status = ss.str();
  return true;
}

}; // namespace






// ----------
// -- MAIN --
// ----------
using namespace estimation;
int main(int argc, char **argv)
{
  // Initialize ROS
  ros::init(argc, argv, "robot_pose_ekf");

  // create filter class
  OdomEstimationNode my_filter_node;

  ros::spin();
  
  return 0;
}
