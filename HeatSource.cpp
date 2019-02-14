#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

#include <iostream>

using namespace std;
using namespace cv;
using namespace image_transport;
using namespace ros;

static const string OPENCV_WINDOW_1 = "Thermal camera";
static const string OPENCV_WINDOW_2 = "Vision camera";

int x = 0; 
int y = 0;

bool InRange(int value, int min, int max){
	if(value > min and value < max)	return true;
	else return false;
}

bool Neighbor(Mat image, int r, int k, int z){
	int dx[] = {-2,2,2,-2,-1,1,1,-1};
	int dy[] = {-2,-2,2,2,-1,-1,1,1};
	
	int i = r + dy[z];
	int j = k + dx[z];
	if(i >= 0 and i < image.rows and j >= 0 and j < image.cols){
	uchar* tmp = &((uchar*)(image.data + image.step*i))[j*3];
	if(InRange((int)tmp[2],200,256))
		if(InRange((int)tmp[1],200,256))
			if(InRange((int)tmp[0],0,200)) return true;
			else return false;	
	}
}
void FindHeatSource(Mat &image_t){
	int ma = 0, mb = 0, mc = 0;
	//bool flaga = true;
	for(int i = 0; i < image_t.rows; i++)
		for(int j = 0; j < image_t.cols; j++){
			uchar* tmp_ptr = &((uchar*)(image_t.data + image_t.step*i))[j*3];
			if(InRange((int)tmp_ptr[2],200,256))
			if(InRange((int)tmp_ptr[1],200,256))
			if(InRange((int)tmp_ptr[0],0,200))	
			if((int)tmp_ptr[0] > ma and (int)tmp_ptr[1] > mb and (int)tmp_ptr[2] > mc){				
				ma = (int)tmp_ptr[0];
				mb = (int)tmp_ptr[1];
				mc = (int)tmp_ptr[2];
				
				for(int z = 0; z < 8; z++){
					if(Neighbor(image_t,i,j,z)){ 
						x = j;	y = i;
					}
					else{
						ma = 0; mb = 0; mc = 0;
					}
				}
			}
		}
	//circle(image_t, Point(x,y), 3, CV_RGB(255,0,0), 1, 8, 0);
}

void ImageModify(Mat &image_v){
	x *= 8;
	y *= 8;
	rectangle(image_v, Point(x-40,y-25), Point(x+40,y+25), CV_RGB(0,255,0), 3, 8, 0);
	line(image_v, Point(x-40,y-25), Point(x+40,y+25), CV_RGB(0,255,0), 2, 8, 0);
	line(image_v, Point(x-40,y+25), Point(x+40,y-25), CV_RGB(0,255,0), 2, 8, 0);
	circle(image_v, Point(x,y), 8, CV_RGB(0,0,255), 2, 8, 0);
		cout << "Vision: "<< "[" << x << "," << y << "]" << endl;
}

class ImageConverter{
	ros::NodeHandle nh;
	image_transport::ImageTransport it;
	image_transport::Subscriber image_sub_t;
	image_transport::Subscriber image_sub_v;
	image_transport::Publisher image_pub_t;
	image_transport::Publisher image_pub_v;
	
	public:
		ImageConverter(): it(nh){
			
			image_sub_t = it.subscribe("/thermal_camera/image_raw", 1, &ImageConverter::imageCb_t, this);
			image_sub_v = it.subscribe("/vision_camera/image_raw", 1, &ImageConverter::imageCb_v, this);
			image_pub_t = it.advertise("/image_converter/output_thermal", 1);
			image_pub_v = it.advertise("/image_converter/output_video", 1);
			
			namedWindow(OPENCV_WINDOW_1,WINDOW_NORMAL);
			namedWindow(OPENCV_WINDOW_2,WINDOW_NORMAL);
			}
		
		~ImageConverter(){
			destroyWindow(OPENCV_WINDOW_1);
			destroyWindow(OPENCV_WINDOW_2);
		}
		
		void imageCb_t(const sensor_msgs::ImageConstPtr& msg){
			cv_bridge::CvImagePtr cv_ptr_t;

			try{
				cv_ptr_t = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
			}
			
			catch(cv_bridge::Exception& e){
				ROS_ERROR("cv_bridge exception: %s", e.what());
				return;
			}
			
			FindHeatSource(cv_ptr_t->image);
			
			imshow(OPENCV_WINDOW_1, cv_ptr_t->image);
			waitKey(20);
			
			image_pub_t.publish(cv_ptr_t->toImageMsg());
			
		}
		void imageCb_v(const sensor_msgs::ImageConstPtr& msg){
			cv_bridge::CvImagePtr cv_ptr_v;

			try{
				cv_ptr_v = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
			}
			
			catch(cv_bridge::Exception& e){
				ROS_ERROR("cv_bridge exception: %s", e.what());
				return;
			}
			
			ImageModify(cv_ptr_v->image);
			imshow(OPENCV_WINDOW_2, cv_ptr_v->image);
			waitKey(20);
			
			image_pub_v.publish(cv_ptr_v->toImageMsg());
		}
			
		
};

 
int main(int argc, char ** argv){

	ros::init(argc, argv, "heat_source_finder");

	ImageConverter ic;
		
	ros::spin();

return 0;
}	
