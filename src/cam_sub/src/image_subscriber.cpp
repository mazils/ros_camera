#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include "std_msgs/msg/int32_multi_array.hpp"

#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <vector>
#include <iostream>
#include "reserved_mem.hpp"
#include "ximage_processing.h"


#define BUFFER_LENGTH 100

#define P_START 0x70000000
#define P_OFFSET 0

class ImageSubscriber : public rclcpp::Node
{
	uint32_t *u_buff;
	uint32_t *read_mem;
	rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_subscription_;
	rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr mem_pub_;

	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr cam_pub_red_res;
	Reserved_Mem res_mem;

	public:

	ImageSubscriber()
		: Node("minimal_subscriber")
	{
		// RCLCPP_INFO(this->get_logger(), "Initializing ImageSubscriber node");

		RCLCPP_INFO(this->get_logger(), "Starting camera subscription");

		camera_subscription_ = this->create_subscription<sensor_msgs::msg::Image>("/image_raw",10,std::bind(&ImageSubscriber::onImageMsg, this, std::placeholders::_1));

		cam_pub_red_res = this->create_publisher<sensor_msgs::msg::Image>("/image_raw_scaled_down",10);

		mem_pub_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("/memory", 10);
		//allocate_memory();

		res_mem=Reserved_Mem();
    	int buffer[BUFFER_LENGTH];
	}
	//  ~ImageSubscriber()
    // {
    //     if (u_buff != nullptr)
    //     {
    //         free(u_buff);
    //     }
    // }

private:
	// void allocate_memory(){
	// 	u_buff = (uint32_t *)malloc(LENGTH);
	// 	if (u_buff == NULL)
	// 	{
	// 		RCLCPP_INFO(this->get_logger(), "could not allocate user buffer ");
	// 	}
	// 	else{
	// 		RCLCPP_INFO(this->get_logger(), "mem is allocated");
	// 	}
	// }
	

	void onImageMsg(const sensor_msgs::msg::Image::SharedPtr msg)
	{
		std_msgs::msg::Int32MultiArray message;
		// RCLCPP_INFO(this->get_logger(), "Received image!");
		// RCLCPP_INFO(this->get_logger(), "My log message ");
		cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
		cv::Mat img = cv_ptr->image;


		cv::Mat img_g;
		cv::cvtColor(img, img, cv::COLOR_BGR2RGB);
		cv::cvtColor(img, img_g, cv::COLOR_RGB2GRAY);
		// Allocate memory for the array
		
		
	//////////////image resising////////////////////

		cv::Mat img_resized;
		cv::resize(img_g, img_resized, cv::Size(10, 10), 0, 0, cv::INTER_AREA);
		// Optional: Normalize pixel values to 0-255
		img_resized.convertTo(img_resized, CV_8U);


//////////////////////////////////////////////////

/////////////////creating message and publish/////////////////////////
		cv_bridge::CvImage cv_bridge_img;
		cv_bridge_img.image = img_resized;
		cv_bridge_img.encoding = sensor_msgs::image_encodings::MONO8; // For grayscale images
		sensor_msgs::msg::Image::SharedPtr msg_reduced = cv_bridge_img.toImageMsg();
		if (img_resized.empty()) {
				RCLCPP_ERROR(this->get_logger(), "Resized image is empty, cannot publish.");
				return;
		}
		RCLCPP_INFO(this->get_logger(), "Publishing reduced image.");
		cam_pub_red_res-> publish(*msg_reduced);
		

//////////////picture to array///////////
		std::vector<uchar> array;
		
		if (img_resized.isContinuous())
		{
			array.assign((uchar*)img_resized.datastart, (uchar*)img_resized.dataend);
		} 
		else 
		{
			for (int i = 0; i < img_resized.rows; ++i) 
			{
				array.insert(array.end(), img_resized.ptr<uchar>(i), img_resized.ptr<uchar>(i)+img_resized.cols);
			}
		}
///////////////////////////////////////////////////////
		
		uint32_t arr[array.size()];

    	// Copy elements from vector to array
    	std::copy(array.begin(), array.end(), arr);
		
		uint32_t u_array[BUFFER_LENGTH];
		for (int i = 0; i < 100; i++) {
        u_array[i] = i + 100; 
    	}
		
		u_buff = u_array;
		// u_buff = arr;
		//WORKS
		int mem_ret = res_mem.transfer(u_buff, P_OFFSET, sizeof(array));
	
		XImage_processing xip;
    	int status=XImage_processing_Initialize(&xip,"image_processing");
    	if (status!= XST_SUCCESS){
        	std::cout << "Cannot Init IP" << std::endl;
        	exit(1);
    	}

    	while(!XImage_processing_IsReady(&xip)){}

    	XImage_processing_Set_in_r(&xip,0x70000000);
    	XImage_processing_Set_out_r(&xip,0x70000000+BUFFER_LENGTH*sizeof(uint32_t));
    	XImage_processing_Start(&xip);

    	while(!XImage_processing_IsDone(&xip)){}
		int buffer[BUFFER_LENGTH];
    	res_mem.gather<int>(buffer,BUFFER_LENGTH,BUFFER_LENGTH);

		for(int i=0;i<100;i++){
        // std::cout << std::to_string(i) << ": "<<std::to_string(buffer[i]) << std::endl;
			RCLCPP_INFO(this->get_logger(), "memory read  %d",buffer[i]);
		}
		// std::cout << "I'd say it works" << std::endl;

		// message.data = *read_mem;
		// mem_pub_->publish(message);
		// RCLCPP_INFO(this->get_logger(), "memory written  %d",mem_ret);






	// 	    // Allocate memory to read back the data
    // read_mem = (uint32_t *)malloc(sizeof(u_array));
    // if (read_mem == NULL)
	// 	{
	// 		RCLCPP_ERROR(this->get_logger(), "Failed to allocate memory for reading back.");
	// 		return;
	// 	}

    // // Gather data from reserved memory
    // int gather_ret = pmem.gather(read_mem, P_OFFSET, sizeof(u_array));
    // if (gather_ret < 0)
	// 	{
	// 		RCLCPP_ERROR(this->get_logger(), "Error in gathering data from reserved memory.");
	// 		free(read_mem);
	// 		return;
	// 	}

	// 	// RCLCPP_INFO(this->get_logger(), "My asdasd message ");



	// RCLCPP_INFO(this->get_logger(), "Data read back from memory:");
    // for (size_t i = 0; i < sizeof(u_array) / sizeof(uint32_t); ++i)
    // {
    //     RCLCPP_INFO(this->get_logger(), "read_mem[%ld] = %u", i, read_mem[i]);
    // }

    // // Free allocated memory for reading back
    // free(read_mem);
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