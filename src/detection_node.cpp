#include <ros/ros.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/CameraInfo.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>

class ImageProcessor {
public:
    ImageProcessor(ros::NodeHandle& nh) : it(nh),is_info_received(false) {
        is_info_received = false; 

        // 1. 使用普通的 Subscriber 订阅带 /compressed 的话题
        sub = nh.subscribe("/hk_camera/image_raw/compressed", 10, &ImageProcessor::imageCallback, this);

        // 2. 发布者依然可以用 image_transport
        pub = it.advertise("/ft/debug/image", 10);

        // 3. 订阅 CameraInfo 话题
        info_sub = nh.subscribe("/hk_camera/camera_info", 1, &ImageProcessor::cameraInfoCallback, this);
    }

    bool isInfoReceived() {
        return is_info_received;
    }

private:
    // 【专门处理内参的回调函数】
    void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& info_msg)
    {
        if (is_info_received) return; 

        cameraMatrix = (cv::Mat_<double>(3, 3) << 
            info_msg->K[0], info_msg->K[1], info_msg->K[2],
            info_msg->K[3], info_msg->K[4], info_msg->K[5],
            info_msg->K[6], info_msg->K[7], info_msg->K[8]);

        // 安全地转换 D
        if (!info_msg->D.empty()) {
            distCoeffs = cv::Mat(1, info_msg->D.size(), CV_64F, (void*)info_msg->D.data()).clone();
        }

        int width = info_msg->width;
        int height = info_msg->height;

        ROS_INFO("收到内参: fx=%.2f, fy=%.2f, cx=%.2f, cy=%.2f", 
                 info_msg->K[0], info_msg->K[4], info_msg->K[2], info_msg->K[5]);
        ROS_INFO("畸变系数个数: %zu", info_msg->D.size());

        is_info_received = true;
    }

    // 【回调函数：接收 CompressedImage】
    void imageCallback(const sensor_msgs::CompressedImageConstPtr& msg)
    {
        if (!is_info_received) return;

        try {
            // ==========================================
            // 【重点修改】：使用 toCvShare
            // ==========================================
            // 1. 先解压成普通 Image 消息
            sensor_msgs::ImagePtr raw_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->toImageMsg();

            // 2. 使用 toCvShare 共享内存
            cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(raw_image, sensor_msgs::image_encodings::BGR8);
            
            // 3. 拿到只读的 cv::Mat
            cv::Mat image_const = cv_ptr->image;

            // ==========================================
            // ⚠️ 注意：image_const 是只读的！
            // 如果你想在上面画线、画圆，必须先 clone 一份！
            // ==========================================
            cv::Mat image = image_const.clone(); 

            // 现在你可以随意在 image 上画图了
            // cv::circle(image, cv::Point(100, 100), 50, cv::Scalar(0, 255, 0), -1);

            // 如果你想发布出去
            cv_bridge::CvImage out_msg;
            out_msg.header = msg->header; 
            out_msg.encoding = sensor_msgs::image_encodings::BGR8;
            out_msg.image = image;
            pub.publish(out_msg.toImageMsg());

        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }
    }

    // --- 你的变量 ---
    cv::Mat cameraMatrix; 
    cv::Mat distCoeffs;
    bool is_info_received; 

    // --- ROS 对象 ---
    image_transport::ImageTransport it;
    ros::Subscriber sub;
    image_transport::Publisher pub;
    ros::Subscriber info_sub;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "my_node");
    ros::NodeHandle nh;

    ImageProcessor processor(nh);

    ros::Rate rate(10);
    while (ros::ok() && !processor.isInfoReceived()) {
        ros::spinOnce();
        ROS_WARN_THROTTLE(1.0, "等待 CameraInfo 数据中...");
        rate.sleep();
    }

    ros::spin();
    return 0;
}