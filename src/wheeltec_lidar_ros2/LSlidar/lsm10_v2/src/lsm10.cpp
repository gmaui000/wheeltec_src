/*************************************************************************************
@company: Copyright (C) 2021, Leishen Intelligent System, WHEELTEC (Dongguan) Co., Ltd
@product: LSM10
@filename: lsm10.cpp
@brief:
@version:       date:       author:            comments:
@v2.0           22-4-12      yao,Tues          ROS2
*************************************************************************************/
#include "lsm10_v2/lsm10.h"
#include <stdio.h>
#include <signal.h>
#include "lsm10_v2/msg/difop.hpp" 
using std::placeholders::_1;
using namespace std;
rclcpp::Node::SharedPtr nh_=nullptr;
namespace ls{
LSM10 * LSM10::instance()
{
  static LSM10 obj;
  return &obj;
}

LSM10::LSM10():rclcpp::Node ("m10lidar")
{
  int code = 0;
  initParam();
  pub_ = create_publisher<sensor_msgs::msg::LaserScan>("scan", 3);
  device_pub = create_publisher<lsm10_v2::msg::Difop>("difop_information", 100);
  serial_ = LSIOSR::instance("/dev/wheeltec_laser", 460800);
  code = serial_->init();
  if(code != 0)
  {
    RCLCPP_INFO(this->get_logger(),"rclcpp is down.");
	  rclcpp::shutdown();
	  exit(0);
  }

  recv_thread_ = new boost::thread(boost::bind(&LSM10::recvThread, this));
  //test_thread_.reset( new boost::thread(boost::bind(&LSM10::recvThread, this)));
  
 // testscan_thread_.reset( new boost::thread(boost::bind(&LSM10::pubScanThread, this)));

   pubscan_thread_ = new boost::thread(boost::bind(&LSM10::pubScanThread, this));

}

LSM10::~LSM10()
{
  printf("start LSM10::~LSM10()\n");

  is_shutdown_ = true;
/*
  test_thread_->interrupt();
  test_thread_->join();
  
  test_thread_ = NULL;
  //delete pubscan_thread_;
  testscan_thread_->interrupt();
  testscan_thread_->join();
  
  testscan_thread_ = NULL;
  //delete pubscan_thread_;
  */
  pubscan_thread_->interrupt();
  pubscan_thread_->join();
  //printf("1end LSM10::~LSM1000()\n");

  pubscan_thread_ = NULL;
  delete pubscan_thread_;
  //printf("2end LSM10::~LSM1000()\n");

  recv_thread_->interrupt();
  recv_thread_->join();
  //printf("3end LSM10::~LSM1000()\n");

  recv_thread_ = NULL;
  delete recv_thread_;
  //printf("4end LSM10::~LSM1000()\n");

  serial_->close();
  serial_ = NULL;
  delete serial_;
  printf("end LSM10::~LSM10()\n");
}
void LSM10::thread_tmp()
{
  int test_rmp=0;

  printf("test thread\n");

}
void LSM10::initParam()
{
  pre_time_ = rclcpp::Node::now();
  time_ = rclcpp::Node::now();
  std::string scan_topic_ = "/scan";
  std::string frame_id_ = "laser";
  std::string serial_port_ = "/dev/wheeltec_laser";
  
  this->declare_parameter<std::string>("scan_topic", scan_topic_);
  this->get_parameter<std::string>("scan", scan_topic_);

  this->declare_parameter<std::string>("laser", "laser");
  this->get_parameter<std::string>("laser", frame_id_);

  this->declare_parameter<std::string>("serial_port", "/dev/wheeltec_laser");
  this->get_parameter<std::string>("serial_port", serial_port_);

  this->declare_parameter<int>("baud_rate", 406800);
  this->get_parameter<int>("baud_rate", baud_rate_);

  this->declare_parameter<int>("rpm", 600);
  this->get_parameter<int>("rpm", rpm_); 
  
  is_shutdown_ = false;

  data_len_ = 180;
  points_size_ = 360 * 42 / 15;
  scan_points_.resize(points_size_);
}

int LSM10::getScan(std::vector<ScanPoint> &points, rclcpp::Time & scan_time, float &scan_duration)
{
  boost::unique_lock<boost::mutex> lock(mutex_);
  points.assign(scan_points_bak_.begin(), scan_points_bak_.end());
  scan_time = pre_time_;
  scan_duration = (time_ - pre_time_).seconds();
}

int LSM10::getVersion(std::string &version)
{
  version = "lsm10_v1_0";
  return 0;
}

double LSM10::getRPM()
{
  return real_rpm_;
}

void LSM10::recvThread()
{
  char * packet_bytes = new char[data_len_];
  int idx = 0;
  double degree;
  
  boost::posix_time::ptime t1,t2;
  t1 = boost::posix_time::microsec_clock::universal_time();
  //printf("rec\n");
  while(!is_shutdown_&&rclcpp::ok()){
	int count = serial_->read(packet_bytes, 90);
  //printf("rec2\n");

	for (int i = 0; i < count; i++)
	{
		int k = packet_bytes[i];
		k < 0 ? k += 256 : k;
		int y = packet_bytes[i + 1];
		y < 0 ? y += 256 : y;

		int k_1 = packet_bytes[i + 2];
		k_1 < 0 ? k_1 += 256 : k_1;
		int y_1 = packet_bytes[i + 3];
		y_1 < 0 ? y_1 += 256 : y_1;
		
		if (k == 0xA5 && y == 0x5A)					 //应答距离
		{

			if(i != 0)
			{
				serial_->read(packet_bytes + 90 - i, i);
			}
			
			int s = packet_bytes[i + 2];
			s < 0 ? s += 256 : s;
			int z = packet_bytes[i + 3];
			z < 0 ? z += 256 : z;
			
			boost::unique_lock<boost::mutex> lock(mutex_);

			//if ((s * 256 + z) / 100.f > 360)
			//	degree = 0;
			//else
			degree = (s * 256 + z) / 100.f;
			//转速
			lsm10_v2::msg::Difop Difop_data;
			s = packet_bytes[i + 4];
			s < 0 ? s += 256 : s;
			z = packet_bytes[i + 5];
			z < 0 ? z += 256 : z;
			
			Difop_data.motorspeed = float(2500000.0 / (s * 256 + z));
			
			device_pub->publish(Difop_data);

			int invalidValue = 0;
			for (size_t num = 2; num < 86; num+=2)
			{
				int s = packet_bytes[i + num + 4];
				s < 0 ? s += 256 : s;
				int z = packet_bytes[i + num + 5];
				z < 0 ? z += 256 : z;
				
				if ((s * 256 + z) != 0xFFFF)
				{
					scan_points_[idx].range = double(s * 256 + (z)) / 1000.f;
					scan_points_[idx].intensity = 0;
					idx++;
				}
				else
				{
					invalidValue++;
				}
			}

			invalidValue = 42 - invalidValue;

			for (size_t i = 0; i < invalidValue; i++)
			{
				if ((degree + (15.0 / invalidValue * i)) > 360)
					scan_points_[idx-invalidValue+i].degree = degree + (15.0 / invalidValue * i) - 360;
				else
					scan_points_[idx-invalidValue+i].degree = degree + (15.0 / invalidValue * i);
			}
			
			lock.unlock();

			if (degree > 359.5) 
			{
				idx = 0;
				t2 = boost::posix_time::microsec_clock::universal_time();
				boost::posix_time::millisec_posix_time_system_config::time_duration_type t_elapse;
				t_elapse = t2 - t1;
				real_rpm_ = 1000000.0 / t_elapse.ticks();
				t1 = t2;

				boost::unique_lock<boost::mutex> lock(mutex_);
				scan_points_bak_.resize(scan_points_.size());
				scan_points_bak_.assign(scan_points_.begin(), scan_points_.end());
				for(int k=0; k<scan_points_.size(); k++)
				{
					scan_points_[k].range = 0;
					scan_points_[k].degree = 0;
				}
				pre_time_ = time_;
				lock.unlock();
				pubscan_cond_.notify_one();
				time_ = this->now();
			}
			
		}	
		else if (k == 0xA5 && y == 0xFF && k_1 == 00 && y_1 == 0x5A)
		{
			printf("elseif");
			/*lsm10_v2::difopPtr Difop_data = lsm10_v2::difopPtr(
						new lsm10_v2::difop());
 
			//温度
			int s = packet_bytes[i + 12];
			s < 0 ? s += 256 : s;
			int z = packet_bytes[i + 13];
			z < 0 ? z += 256 : z;
			Difop_data->Temperature = (float(s * 256 + z) / 4096 * 330 - 50);

			//高压
			s = packet_bytes[i + 14];
			s < 0 ? s += 256 : s;
			z = packet_bytes[i + 15];
			z < 0 ? z += 256 : z;
			Difop_data->HighPressure = (float(s * 256 + z) / 4096 * 3.3 * 101);

			//转速
			s = packet_bytes[i + 16];
			s < 0 ? s += 256 : s;
			z = packet_bytes[i + 17];
			z < 0 ? z += 256 : z;
			Difop_data->MotorSpeed = (float(1000000 / (s * 256 + z) / 24));
			
			device_pub.publish(Difop_data);*/
		}
	}
  }
  if (packet_bytes)
  {
    packet_bytes = NULL;
    delete packet_bytes;
  }
}

void LSM10::pubScanThread()
{

  //printf("pubscan0");

  bool wait_for_wake = true;
  boost::unique_lock<boost::mutex> lock(pubscan_mutex_);

  while (!is_shutdown_&&rclcpp::ok())
  {
    while (wait_for_wake)
    {
      pubscan_cond_.wait(lock);
      wait_for_wake = false;
    }
     //printf("pubscan1");
    std::vector<ScanPoint> points;
    rclcpp::Time start_time;
    float scan_time;
    this->getScan(points, start_time, scan_time);
    int count = points.size();
    if (count <= 0)
      continue;

    sensor_msgs::msg::LaserScan msg;
    msg.header.frame_id = "laser";
    msg.header.stamp = start_time;
    msg.angle_min = 0.0;
    msg.angle_max = 2 * M_PI;
    msg.angle_increment = (msg.angle_max - msg.angle_min) / count;
    msg.range_min = 0.02;
    msg.range_max = 50;
    msg.ranges.resize(count);
    msg.intensities.resize(count);
    msg.scan_time = scan_time;
    msg.time_increment = scan_time / (double)(count - 1);
	
	for(int k=0; k<count; k++)
	{
		msg.ranges[k] = std::numeric_limits<float>::infinity();
        msg.intensities[k] = 0;
	}
	
	for (int i = 0; i < count; i++) {
		int point_idx = 1008 - points[i].degree * 42 / 15;

		if(point_idx < 0 || point_idx > 1007) 
		{
			continue;
		}

      if (points[i].range == 0.0) {
        msg.ranges[point_idx] = std::numeric_limits<float>::infinity();
        msg.intensities[point_idx] = 0;
     // printf("pubscan2");
      }
      else {
        double dist = points[i].range;
        msg.ranges[point_idx] = (float) dist;
        msg.intensities[point_idx] = points[i].intensity;
     // printf("pubscan3");
      }
    }
    pub_->publish(msg);
     // printf("pubscan4");
    wait_for_wake = true;
  } 
}
}

void handleSig(int signo)
{
  rclcpp::shutdown();
  exit(0);
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  signal(SIGINT, handleSig);
  signal(SIGTERM, handleSig);
  ls::LSM10* lsm10 = ls::LSM10::instance();
  //auto node=std::make_shared<ls::LSM10>();
  auto node = rclcpp::Node::make_shared("lidar10");
  rclcpp::spin(node);
  return 0;
}
