#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <vector>
#include <iostream>

class ImageSubscriber : public rclcpp::Node
{
public:
	ImageSubscriber() : Node("image_subscriber")
	{
		RCLCPP_INFO(this->get_logger(), "Initializing ImageSubscriber node");

		RCLCPP_INFO(this->get_logger(), "Starting camera subscription");

		camera_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
			"/image_raw",
			10,
			std::bind(&ImageSubscriber::onImageMsg, this, std::placeholders::_1));
	}

private:
	rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_subscription_;

	void onImageMsg(const sensor_msgs::msg::Image::SharedPtr msg)
	{
		// RCLCPP_INFO(this->get_logger(), "Received image!");

		cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
		cv::Mat img = cv_ptr->image;
		cv::Mat img_g;
		cv::cvtColor(img, img, cv::COLOR_BGR2RGB);
		cv::cvtColor(img, img_g, cv::COLOR_RGB2GRAY);

		// cv::imshow("Image", img);
		// cv::imshow("Image1", img_g);
		
		int iTop = img_g.rows;
		int jTop = img_g.cols;
		

		int64_t count = 0;	
		for (int i = 0; i < iTop; i++)
		{
		
			for (int j = 0; j < jTop; j++)
			{
				// b = input[img_g.step * j + i];
				int pixel_value = img_g.at<uchar>(i, j);
				//RCLCPP_INFO(this->get_logger(), "Image Data: %d", pixel_value);	

				// send to the common ram
				
				// count ++;

			}
		}

		//RCLCPP_INFO(this->get_logger(), "DOOOOOOOONEEEEE!");

		cv::waitKey(34);
	}
};

int main(int argc, char *argv[])
{
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);

	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<ImageSubscriber>());

	rclcpp::shutdown();
	return 0;
}
