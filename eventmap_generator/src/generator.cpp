#include "generator.h"

namespace generator{

EventMap::EventMap(ros::NodeHandle& nh, ros::NodeHandle& nh_private) : nh_(nh){
    nh_.param("BagName", bagName, std::string("record.bag"));
    nh_.param("StorePath", store_path, std::string("intermediateBags"));

    event_sub_ = nh_.subscribe("/dvs/events", 10, &EventMap::eventsCallback, this);
    eventmap_pub_ = nh_.advertise<eventmap_generator::eventmap>("/eventmap", 1);

    // Judge if event queue is empty
    bSensorInitialized_ = false;
    if(pEventQueueMat_)
        pEventQueueMat_->clear();
    sensor_size_ = cv::Size(0,0);
    
    readBag(bagName);
}

void EventMap::readBag(std::string bagName){
    read_bag.open(bagName, rosbag::bagmode::Read);
    write_bag.open(store_path + "/eventmap.bag", rosbag::bagmode::Write);

    rosbag::View view(read_bag, rosbag::TopicQuery("/dvs/events"));

    for (const rosbag::MessageInstance& m : view) {
        dvs_msgs::EventArray::ConstPtr msg = m.instantiate<dvs_msgs::EventArray>();

        if (!bSensorInitialized_) {
            init(msg->width, msg->height);
        }   
        for(const dvs_msgs::Event& e : msg->events){
            static bool first_input = true;
            if (first_input){
                first_input = false;
                trig_time_ = e.ts;
                last_time_ = e.ts;
            }
            events_.push_back(e);
            pEventQueueMat_->insertEvent(e);
            
            ros::Duration duration = e.ts - last_time_;
            double time_diff_ms = duration.toSec() * 1000;
            if (time_diff_ms > 3){
                ROS_ERROR("Missing event alert!");
                trig_time_ = e.ts;
                last_time_ = e.ts;
                events_.clear();
                continue;
            }
            last_time_ = e.ts;

            duration = e.ts - trig_time_;
            time_diff_ms = duration.toSec() * 1000;
        
            if (time_diff_ms > 30 && events_.size() > 10000){
                trig_time_ = events_[events_.size() - 10000].ts;
                ROS_INFO("The timestamp is %d.%d:", trig_time_.sec, trig_time_.nsec);
                generateEventMap(trig_time_);
                
                trig_time_ = e.ts;
                events_.clear();
            }
        }
    }

    read_bag.close();
    write_bag.close();

    ros::shutdown();
}

void EventMap::eventsCallback(const dvs_msgs::EventArray::ConstPtr& msg){
    if(!bSensorInitialized_){
        init(msg->width, msg->height);
    }     

    for(const dvs_msgs::Event& e : msg->events){
        static bool first_input = true;
        if (first_input){
            first_input = false;
            trig_time_ = e.ts;
        }
        events_.push_back(e);
        pEventQueueMat_->insertEvent(e);
        
        ros::Duration duration = e.ts - trig_time_;
        double time_diff_ms = duration.toSec() * 1000;

        if (time_diff_ms > 30){
            //clearEventQueue();
            if (events_.size() > 1000){
                trig_time_ = events_[events_.size() - 1000].ts;
            }

            ROS_INFO("The timestamp is %d.%d:", trig_time_.sec, trig_time_.nsec);
            generateEventMap(trig_time_);
            
            trig_time_ = e.ts;
            events_.clear();
        }
    }
}

void EventMap::init(int width, int height){
    sensor_size_ = cv::Size(width, height);
    bSensorInitialized_ = true;
    pEventQueueMat_.reset(new EventQueueMat(width, height, max_event_queue_length_));
    timestamps.resize(width * height);
    ROS_INFO("Sensor size: (%d x %d)", sensor_size_.width, sensor_size_.height);
}

void EventMap::clearEventQueue()
{
    static constexpr size_t MAX_EVENT_QUEUE_LENGTH = 1000000;
    if (events_.size() > MAX_EVENT_QUEUE_LENGTH)
    {
        size_t remove_events = events_.size() - MAX_EVENT_QUEUE_LENGTH;
        events_.erase(events_.begin(), events_.begin() + remove_events);
    }
}

void EventMap::generateEventMap(const ros::Time& triggered_time){
    cv::Mat event_map_no_polarity = cv::Mat::zeros(cv::Size(sensor_size_.width, sensor_size_.height), CV_32F);
    cv::Mat plot = cv::Mat::zeros(cv::Size(sensor_size_.width, sensor_size_.height), CV_32FC3);
    int count = 0;
    for(size_t x = 0; x < sensor_size_.width; ++x){
        for(size_t y = 0; y < sensor_size_.height; ++y){
            EventQueue& eq = pEventQueueMat_->getEventQueue(x, y);
            if (!eq.empty() && eq.back().ts >= triggered_time){
                count++;
                if (eq.back().polarity){
                    event_map_no_polarity.at<float>(y, x) = 1;
                    cv::circle(plot, cvPoint(x, y), 0.5, cv::Scalar(1, 1, 1), -1);
                    timestamps[y * sensor_size_.width + x] = eq.back().ts;
                }else{
                    event_map_no_polarity.at<float>(y, x) = -1;
                    cv::circle(plot, cvPoint(x, y), 0.5, cv::Scalar(1, 1, 1), -1);
                    timestamps[y * sensor_size_.width + x] = eq.back().ts;
                }
            }
        }
    }
    event_map_no_polarity = (event_map_no_polarity + 1) / 2;
    ROS_INFO("Event map generated");
    
    // Publish eventmap
    eventmap_generator::eventmap eventmap_msg;
    cv_bridge::CvImage cv_image;
    cv_image.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
    cv_image.image = event_map_no_polarity;
    cv_image.header.stamp = triggered_time;
    static sensor_msgs::Image ros_image;
    cv_image.toImageMsg(ros_image);
    cv::imshow("event", plot);
    cv::waitKey(1);

    eventmap_msg.eventmap = ros_image;
    eventmap_msg.timestamps = timestamps;
    if(eventmap_pub_.getNumSubscribers() > 0)
    {
        eventmap_pub_.publish(eventmap_msg);
    }

    write_bag.write("/eventmap", ros::Time::now(), eventmap_msg);
}

void EventMap::generateEventMap_hyperthread(const ros::Time& triggered_time){
    ROS_INFO("Event map begin");
    cv::Mat event_map_no_polarity = cv::Mat::zeros(cv::Size(sensor_size_.width, sensor_size_.height), CV_32F);

    // distribute jobs
    int NUM_THREAD_TS = 8;
    std::vector<Job> jobs(NUM_THREAD_TS);
    size_t num_col_per_thread = sensor_size_.width / NUM_THREAD_TS;
    size_t res_col = sensor_size_.width % NUM_THREAD_TS;
    for(size_t i = 0; i < NUM_THREAD_TS; i++)
    {
        jobs[i].i_thread_ = i;
        jobs[i].start_col_ = num_col_per_thread * i;
        if(i == NUM_THREAD_TS - 1)
            jobs[i].end_col_ = jobs[i].start_col_ + num_col_per_thread - 1 + res_col;
        else
            jobs[i].end_col_ = jobs[i].start_col_ + num_col_per_thread - 1;
        jobs[i].start_row_ = 0;
        jobs[i].end_row_ = sensor_size_.height - 1;
        jobs[i].trig_time = triggered_time;
        jobs[i].event_no_polarity_ = &event_map_no_polarity;
    }

    // hyper thread processing
    std::vector<std::thread> threads;
    threads.reserve(NUM_THREAD_TS);
    for(size_t i = 0; i < NUM_THREAD_TS; i++)
        threads.emplace_back(std::bind(&EventMap::thread, this, jobs[i]));
    for(auto& thread:threads)
        if(thread.joinable())
            thread.join();

    event_map_no_polarity = (event_map_no_polarity + 1) / 2;
    ROS_INFO("Event map generated");
    
    // Publish eventmap
    eventmap_generator::eventmap eventmap_msg;
    cv_bridge::CvImage cv_image;
    cv_image.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
    cv_image.image = event_map_no_polarity;
    cv_image.header.stamp = triggered_time;
    static sensor_msgs::Image ros_image;
    cv_image.toImageMsg(ros_image);
    cv::imshow("event", event_map_no_polarity);
    cv::waitKey(1);

    eventmap_msg.eventmap = ros_image;
    eventmap_msg.timestamps = timestamps;
    if(eventmap_pub_.getNumSubscribers() > 0)
    {
        eventmap_pub_.publish(eventmap_msg);
    }
}

void EventMap::thread(Job &job){
    size_t start_col = job.start_col_;
    size_t end_col = job.end_col_;
    size_t start_row = job.start_row_;
    size_t end_row = job.end_row_;
    size_t i_thread = job.i_thread_;
    ros::Time trig_time = job.trig_time;
    cv::Mat& event_map_no_polarity = *job.event_no_polarity_;

    for(size_t y = start_row; y <= end_row; y++){
        for(size_t x = start_col; x <= end_col; x++){
            EventQueue& eq = pEventQueueMat_->getEventQueue(x, y);
            if (!eq.empty() && eq.back().ts > trig_time){
                if (eq.back().polarity){
                    event_map_no_polarity.at<float>(y, x) = 1;
                }else{
                    event_map_no_polarity.at<float>(y, x) = -1;
                }
            }
        }
    }
}

};