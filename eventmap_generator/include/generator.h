#pragma once 

#include "ros/ros.h"
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <dvs_msgs/Event.h>
#include <dvs_msgs/EventArray.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Time.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <eventmap_generator/eventmap.h>

#include <thread>
#include <iostream>
#include <chrono>

namespace generator{

using EventQueue = std::deque<dvs_msgs::Event>;

class EventQueueMat 
{
public:
    EventQueueMat(int width, int height, int queueLen){
        width_ = width;
        height_ = height;
        queueLen_ = queueLen;
        eqMat_ = std::vector<EventQueue>(width_ * height_, EventQueue());
    }

    void insertEvent(const dvs_msgs::Event& e){
        if(!insideImage(e.x, e.y))
            return;
        else{
            EventQueue& eq = getEventQueue(e.x, e.y);
            eq.push_back(e);
            while(eq.size() > queueLen_)
                eq.pop_front();
        }
    }

    void clear(){
      eqMat_.clear();
    }

    bool insideImage(const size_t x, const size_t y){
      return !(x < 0 || x >= width_ || y < 0 || y >= height_);
    }

    inline EventQueue& getEventQueue(const size_t x, const size_t y){
      return eqMat_[x + width_ * y];
    }

    size_t width_;
    size_t height_;
    size_t queueLen_;
    std::vector<EventQueue> eqMat_;

private:
    
};

class EventMap {
public:
    EventMap(ros::NodeHandle& nh, ros::NodeHandle& nh_private);
    ~EventMap(){event_sub_.shutdown();};

private:
    struct Job{
        size_t start_col_, end_col_;
        size_t start_row_, end_row_;
        size_t i_thread_;
        ros::Time trig_time;
        cv::Mat* event_no_polarity_;
        cv::Mat* event_positive_;
        cv::Mat* event_negative_;
    };

    void eventsCallback(const dvs_msgs::EventArray::ConstPtr& msg);
    void init(int width, int height);
    void readBag(std::string bagName);
    void clearEventQueue();
    void thread(Job &job);
    void generateEventMap(const ros::Time& triggered_time);
    void generateEventMap_hyperthread(const ros::Time& triggered_time);

    ros::NodeHandle nh_;
    ros::Subscriber event_sub_;
    ros::Publisher eventmap_pub_;

    rosbag::Bag read_bag, write_bag;
    std::string bagName, store_path;

    bool bSensorInitialized_ = false;
    int max_event_queue_length_ = 10;
    int output_event_num_;
    ros::Time trig_time_, last_time_;
    cv::Size sensor_size_;

    // containers
    EventQueue events_;
    std::shared_ptr<EventQueueMat> pEventQueueMat_;

    std::vector<ros::Time> timestamps;

    int count = 0;
};  


};