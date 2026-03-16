/*

A bare-bones example node demonstrating the use of the Monocular mode in ORB-SLAM3

Author: Azmyin Md. Kamal
Date: 01/01/24

REQUIREMENTS
* Make sure to set path to your workspace in common.hpp file

*/

//* Includes
#include "ros2_orb_slam3/common.hpp"

//* Constructor
MonocularMode::MonocularMode() :Node("mono_node_cpp")
{
    // Declare parameters to be passsed from command line
    // https://roboticsbackend.com/rclcpp-params-tutorial-get-set-ros2-params-with-cpp/
    
    //* Find path to home directory
    homeDir = getenv("HOME");
    
    //* Resolve package path at runtime using ament_index
    try {
        packagePath = ament_index_cpp::get_package_share_directory("ros2_orb_slam3") + "/";
    } catch (const std::exception& e) {
        RCLCPP_WARN(this->get_logger(), "Could not find package share dir, falling back to source path");
        packagePath = std::string(homeDir) + "/Workspaces/DroneCrawler/src/ros2_orb_slam3/";
    }
    
    // std::cout<<"VLSAM NODE STARTED\n\n";
    RCLCPP_INFO(this->get_logger(), "\nORB-SLAM3-V1 NODE STARTED");

    this->declare_parameter("node_name_arg", "not_given"); // Name of this agent 
    this->declare_parameter("voc_file_arg", "file_not_set"); // Needs to be overriden with appropriate name  
    this->declare_parameter("settings_file_path_arg", "file_path_not_set"); // path to settings file  
    
    //* Watchdog, populate default values
    nodeName = "not_set";
    vocFilePath = "file_not_set";
    settingsFilePath = "file_not_set";

    //* Populate parameter values
    rclcpp::Parameter param1 = this->get_parameter("node_name_arg");
    nodeName = param1.as_string();
    
    rclcpp::Parameter param2 = this->get_parameter("voc_file_arg");
    vocFilePath = param2.as_string();

    rclcpp::Parameter param3 = this->get_parameter("settings_file_path_arg");
    settingsFilePath = param3.as_string();

    // rclcpp::Parameter param4 = this->get_parameter("settings_file_name_arg");
    
  
    //* HARDCODED, set paths
    if (vocFilePath == "file_not_set" || settingsFilePath == "file_not_set")
    {
        pass;
        vocFilePath = packagePath + "orb_slam3/Vocabulary/ORBvoc.txt.bin";
        settingsFilePath = packagePath + "orb_slam3/config/Monocular/";
    }

    // std::cout<<"vocFilePath: "<<vocFilePath<<std::endl;
    // std::cout<<"settingsFilePath: "<<settingsFilePath<<std::endl;
    
    
    //* DEBUG print
    RCLCPP_INFO(this->get_logger(), "nodeName %s", nodeName.c_str());
    RCLCPP_INFO(this->get_logger(), "voc_file %s", vocFilePath.c_str());
    // RCLCPP_INFO(this->get_logger(), "settings_file_path %s", settingsFilePath.c_str());
    
    subexperimentconfigName = "/mono_py_driver/experiment_settings"; // topic that sends out some configuration parameters to the cpp ndoe
    pubconfigackName = "/mono_py_driver/exp_settings_ack"; // send an acknowledgement to the python node
    subImgMsgName = "/mono_py_driver/img_msg"; // topic to receive RGB image messages
    subTimestepMsgName = "/mono_py_driver/timestep_msg"; // topic to receive RGB image messages

    //* subscribe to python node to receive settings
    expConfig_subscription_ = this->create_subscription<std_msgs::msg::String>(subexperimentconfigName, 1, std::bind(&MonocularMode::experimentSetting_callback, this, _1));

    //* publisher to send out acknowledgement
    configAck_publisher_ = this->create_publisher<std_msgs::msg::String>(pubconfigackName, 10);

    //* subscrbite to the image messages coming from the Python driver node
    subImgMsg_subscription_= this->create_subscription<sensor_msgs::msg::Image>(subImgMsgName, 1, std::bind(&MonocularMode::Img_callback, this, _1));

    //* subscribe to receive the timestep
    subTimestepMsg_subscription_= this->create_subscription<std_msgs::msg::Float64>(subTimestepMsgName, 1, std::bind(&MonocularMode::Timestep_callback, this, _1));

    //* Publisher for camera pose (world frame)
    pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/orb_slam/pose", 10);
    odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/orb_slam/odom", 10);

    
    RCLCPP_INFO(this->get_logger(), "Waiting to finish handshake ......");
    
}

//* Destructor
MonocularMode::~MonocularMode()
{   
    
    // Stop all threads
    // Call method to write the trajectory file
    // Release resources and cleanly shutdown
    pAgent->Shutdown();
    pass;

}

//* Callback which accepts experiment parameters from the Python node
void MonocularMode::experimentSetting_callback(const std_msgs::msg::String& msg){
    
    // Guard: only process the first configuration message to prevent
    // settingsFilePath being appended to multiple times (bridge sends
    // config repeatedly on a timer until it receives ACK)
    if (bSettingsFromPython) {
        return;
    }
    bSettingsFromPython = true;
    experimentConfig = msg.data.c_str();
    
    RCLCPP_INFO(this->get_logger(), "Configuration YAML file name: %s", experimentConfig.c_str());

    //* Publish acknowledgement
    auto message = std_msgs::msg::String();
    message.data = "ACK";
    
    std::cout<<"Sent response: "<<message.data.c_str()<<std::endl;
    configAck_publisher_->publish(message);

    //* Wait to complete VSLAM initialization
    initializeVSLAM(experimentConfig);

}

//* Method to bind an initialized VSLAM framework to this node
void MonocularMode::initializeVSLAM(std::string& configString){
    
    // Watchdog, if the paths to vocabular and settings files are still not set
    if (vocFilePath == "file_not_set" || settingsFilePath == "file_not_set")
    {
        RCLCPP_ERROR(get_logger(), "Please provide valid voc_file and settings_file paths");       
        rclcpp::shutdown();
    } 
    
    //* Build .yaml`s file path
    
    settingsFilePath = settingsFilePath.append(configString);
    settingsFilePath = settingsFilePath.append(".yaml"); // Example ros2_ws/src/orb_slam3_ros2/orb_slam3/config/Monocular/TUM2.yaml

    RCLCPP_INFO(this->get_logger(), "Path to settings file: %s", settingsFilePath.c_str());
    
    // NOTE if you plan on passing other configuration parameters to ORB SLAM3 Systems class, do it here
    // NOTE you may also use a .yaml file here to set these values
    sensorType = ORB_SLAM3::System::MONOCULAR; 
    enablePangolinWindow = true; // Shows Pangolin window output
    enableOpenCVWindow = true; // Shows OpenCV window output
    
    pAgent = new ORB_SLAM3::System(vocFilePath, settingsFilePath, sensorType, enablePangolinWindow);
    std::cout << "MonocularMode node initialized" << std::endl; // TODO needs a better message
}

//* Callback that processes timestep sent over ROS
void MonocularMode::Timestep_callback(const std_msgs::msg::Float64& time_msg){
    // timeStep = 0; // Initialize
    timeStep = time_msg.data;
}

//* Callback to process image message and run SLAM node
void MonocularMode::Img_callback(const sensor_msgs::msg::Image& msg)
{
    // Initialize
    cv_bridge::CvImagePtr cv_ptr; //* Does not create a copy, memory efficient
    
    static int img_count = 0;
    img_count++;
    if (img_count == 1 || img_count % 50 == 0) {
        RCLCPP_INFO(this->get_logger(), "Img_callback invoked (frame %d, %dx%d, encoding=%s)",
                     img_count, msg.width, msg.height, msg.encoding.c_str());
    }
    
    //* Convert ROS image to openCV image
    try
    {
        //cv::Mat im =  cv_bridge::toCvShare(msg.img, msg)->image;
        cv_ptr = cv_bridge::toCvCopy(msg); // Local scope
        
        // DEBUGGING, Show image
        // Update GUI Window
        // cv::imshow("test_window", cv_ptr->image);
        // cv::waitKey(3);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(),"Error reading image");
        return;
    }
    
    // std::cout<<std::fixed<<"Timestep: "<<timeStep<<std::endl; // Debug
    
    //* Perform all ORB-SLAM3 operations in Monocular mode
    //! Pose with respect to the camera coordinate frame not the world coordinate frame
    Sophus::SE3f Tcw = pAgent->TrackMonocular(cv_ptr->image, timeStep); 
    
    // Only publish when tracking is active (OK=2 or OK_KLT=5).
    // Publishing identity/stale poses during tracking failure corrupts
    // FUEL's occupancy map and confuses the controller.
    int tracking_state = pAgent->GetTrackingState();
    static int last_logged_state = -1;
    static int pose_pub_count = 0;
    if (tracking_state != last_logged_state) {
        RCLCPP_INFO(this->get_logger(), "Tracking state changed: %d -> %d (published %d poses so far)",
                     last_logged_state, tracking_state, pose_pub_count);
        last_logged_state = tracking_state;
    }
    if (tracking_state != 2 && tracking_state != 5) {
        return;
    }
    pose_pub_count++;
    if (pose_pub_count == 1 || pose_pub_count % 50 == 0) {
        RCLCPP_INFO(this->get_logger(), "Publishing pose #%d (frame %d)", pose_pub_count, img_count);
    }

    //* Convert camera-frame pose to world-frame and publish
    Sophus::SE3f Twc = Tcw.inverse();
    Eigen::Vector3f translation = Twc.translation();
    Eigen::Quaternionf quaternion = Twc.unit_quaternion();

    auto pose_msg = geometry_msgs::msg::PoseStamped();
    // Use the source image timestamp so FUEL's ApproximateTimeSynchronizer
    // can match this pose with the depth image from the same camera frame.
    pose_msg.header.stamp = msg.header.stamp;
    pose_msg.header.frame_id = "world";
    pose_msg.pose.position.x = static_cast<double>(translation.x());
    pose_msg.pose.position.y = static_cast<double>(translation.y());
    pose_msg.pose.position.z = static_cast<double>(translation.z());
    pose_msg.pose.orientation.x = static_cast<double>(quaternion.x());
    pose_msg.pose.orientation.y = static_cast<double>(quaternion.y());
    pose_msg.pose.orientation.z = static_cast<double>(quaternion.z());
    pose_msg.pose.orientation.w = static_cast<double>(quaternion.w());

    pose_publisher_->publish(pose_msg);

    //* Also publish as Odometry for FUEL exploration planner
    auto odom_msg = nav_msgs::msg::Odometry();
    odom_msg.header = pose_msg.header;
    odom_msg.child_frame_id = "base_link";
    odom_msg.pose.pose = pose_msg.pose;
    odom_publisher_->publish(odom_msg);

}


