
//#define ROS
#define CAMERA_SHOW
#define CAMERA_SHOW_MORE

#ifdef ROS
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sys/wait.h>
#include <termio.h>
#include <unistd.h>
#include "std_msgs/Int16.h"

#endif

#include<opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
//#include <cv.hpp>
#include <iostream>
#include <stdio.h>
#include <fcntl.h>
#include <time.h>

#define PI 3.1415926

using namespace std;
using namespace cv;

const int Width = 320;
const int Height = 240;
const int XHalf = (Width/2);
const int YHalf = (Height/2);
const int YPoint = 60;
const float slope_threshold = 0.5;
const Scalar Red = Scalar(0, 0, 255);
const Scalar Blue = Scalar(255, 0, 0);
const Scalar Yellow = Scalar(50, 250, 250);
const Scalar Sky = Scalar(215, 200, 60);
const Scalar Pink = Scalar(220, 110, 230);
int lap_cnt = 0;
int flag = 1;

void show_lines(Mat &img, vector<Vec4i> &lines, Scalar color = Scalar(0, 0, 0),int thickness=2)

{
    bool color_gen = false;
    
    if (color == Scalar(0, 0, 0))
        color_gen = true;
    
    for (int i = 0; i < lines.size(); i++)
    {
        if (color_gen == true)
            
            color = Scalar(rand() % 256, rand() % 256, rand() % 256);
        
        line(img, Point(lines[i][0], lines[i][1]), Point(lines[i][2], lines[i][3]), color, thickness);
        
    }
    
}

void find_start_lines(Mat &img, vector<Vec4i> &lines)
{
    int pre_flag = flag;
    for (int i = 0; i < lines.size(); i++)
    {
        int x1 = lines[i][0];
        int y1 = lines[i][1];
        int x2 = lines[i][2];
        int y2 = lines[i][3];
        
        float slope;
        //Calculate slope
        if (x2 - x1 == 0) //corner case, avoiding division by 0
            slope = 999.0; //practically infinite slope
        else
            slope = (y2 - y1) / (float)(x2 - x1);
        
        //Filter lines based on slope
        if (abs(slope) < 0.065) {
            flag = -flag;
            //lap_cnt++;
            //if(lap_cnt > 1)
            putText(img, format("Find START LINE: %d\n", lap_cnt), Point(50,50), FONT_HERSHEY_SIMPLEX,0.5,Yellow,2);
            printf("lap_cnt: %d\n", lap_cnt);
            
        }
    }
    
    if(pre_flag != flag)
    {
        lap_cnt++;
    }
    
}

void  split_left_right(vector<Vec4i> lines, vector<Vec4i>&left_lines, vector<Vec4i> &right_lines)
{
    vector<float> slopes;
    vector<Vec4i> new_lines;
    
    for (int i = 0; i < lines.size(); i++)
    {
        int x1 = lines[i][0];
        int y1 = lines[i][1];
        int x2 = lines[i][2];
        int y2 = lines[i][3];
        
        float slope;
        //Calculate slope
        if (x2 - x1 == 0) //corner case, avoiding division by 0
            slope = 999.0; //practically infinite slope
        else
            slope = (y2 - y1) / (float)(x2 - x1);
        
        //Filter lines based on slope
        if (abs(slope) > slope_threshold) {
            slopes.push_back(slope);
            new_lines.push_back(lines[i]);
        }
        
    }
    
    for (int i = 0; i < new_lines.size(); i++)
    {
        Vec4i line = new_lines[i];
        float slope = slopes[i];
        
        int x1 = line[0];
        int y1 = line[1];
        int x2 = line[2];
        int y2 = line[3];
        
        if (slope > 0 && x1 > XHalf && x2 > XHalf)//
            right_lines.push_back(line);
        else if (slope < 0 && x1 < XHalf && x2 < XHalf)//slope < 0 && x1 < cx && x2 < cx
            left_lines.push_back(line);
    }
}

bool find_line_params(vector<Vec4i> &left_lines, float *left_m, float *left_b)
{
    
    float  left_avg_x = 0, left_avg_y = 0, left_avg_slope = 0;
    
    if(left_lines.size() == 0)
        return false;
    
    for (int i = 0; i < left_lines.size(); i++)//calculate right avg of x and y and slope
    {
        //line(roi, Point(left_lines[i][0],left_lines[i][1]), Point(left_lines[i][2],left_lines[i][3]), color, 3);
        left_avg_x += left_lines[i][0];
        left_avg_x += left_lines[i][2];
        left_avg_y += left_lines[i][1];
        left_avg_y += left_lines[i][3];
        left_avg_slope += (left_lines[i][3] - left_lines[i][1]) / (float)(left_lines[i][2] - left_lines[i][0]);
    }
    left_avg_x = left_avg_x / (left_lines.size() * 2);
    left_avg_y = left_avg_y / (left_lines.size() * 2);
    left_avg_slope = left_avg_slope / left_lines.size();
    
    // return values
    *left_m = left_avg_slope;
    //b=y-mx //find Y intercet
    *left_b = left_avg_y - left_avg_slope * left_avg_x;
    
    return true;
}


void find_lines(Mat &img, vector<cv::Vec4i> &left_lines, vector<Vec4i>& right_lines, float *rdistance, float *ldistance)
{
    static float left_slope_mem = 1, right_slope_mem = 1, right_b_mem = 0, left_b_mem = 0;
    
    float left_b, right_b, left_m, right_m;
    
    bool draw_left = find_line_params(left_lines, &left_m, &left_b);
    if (draw_left) {
        float left_x0 = (-left_b) / left_m;
        float left_x120 = (YHalf - left_b) / left_m;
        left_slope_mem = left_m;
        left_b_mem = left_b;
#ifdef CAMERA_SHOW
        line(img, Point(left_x0, 0), Point(left_x120, YHalf), Blue, 3);
        cout << left_lines.size() << " left lines,";
#endif
    }
    else {
        cout << "\tNo Left Line,";
    }
    
    bool draw_right = find_line_params(right_lines, &right_m, &right_b);
    if (draw_right) {
        float right_x0 = (-right_b) / right_m;
        float right_x120 = (YHalf - right_b) / right_m;
        right_slope_mem = right_m;
        right_b_mem = right_b;
#ifdef CAMERA_SHOW
        line(img, Point(right_x0, 0), Point(right_x120, YHalf), Red, 3);
#endif
        cout << right_lines.size() << " right lines" <<endl;
    }
    else {
        cout << "\tNo RIght Line" << endl;
    }
    // y = mx + b ==> x0 = (y0-b)/m
    float left_xPt = ((YPoint - left_b_mem) / left_slope_mem);
    float right_xPt = ((YPoint - right_b_mem) / right_slope_mem);
    *ldistance = XHalf - left_xPt;
    *rdistance = right_xPt - XHalf;
}

int img_process(Mat &frame)
{
    Mat grayframe, edge_frame, roi_gray_ch3;
    Mat roi, start_roi; // [출발선]
    cvtColor(frame, grayframe, COLOR_BGR2GRAY);
    Rect rect_roi(0,YHalf,Width,YHalf);
    roi = frame(rect_roi);
    
    cvtColor(roi,grayframe,COLOR_BGR2GRAY) ;
    GaussianBlur(grayframe,grayframe,Size(3,3),1.5);
    cvtColor(grayframe, roi_gray_ch3, COLOR_GRAY2BGR);
    cvtColor(grayframe, start_roi, COLOR_GRAY2BGR); // [출발선]
    Canny(grayframe,edge_frame,70,150,3); //min_val, max val , filter size
    
    
    //============
    // convert to HSV color space
    Mat hsvImage;
    cvtColor(roi, hsvImage, COLOR_BGR2HSV);
    
    // split the channels
    std::vector<cv::Mat> hsvChannels;
    cv::split(hsvImage, hsvChannels);
    
    // hue channels tells you the color tone, if saturation and value aren't too low.
    
    // red color is a special case, because the hue space is circular and red is exactly at the beginning/end of the circle.
    // in literature, hue space goes from 0 to 360 degrees, but OpenCV rescales the range to 0 up to 180, because 360 does not fit in a single byte. Alternatively there is another mode where 0..360 is rescaled to 0..255 but this isn't as common.
    int hueValue = 0; // red color
    int hueRange = 12; // how much difference from the desired color we want to include to the result If you increase this value, for example a red color would detect some orange values, too.
    
    int minSaturation = 120; // I'm not sure which value is good here...
    int minValue = 134; // not sure whether 50 is a good min value here...
    
    Mat hueImage = hsvChannels[0]; // [hue, saturation, value]
    
    // is the color within the lower hue range?
    Mat hueMask;
    inRange(hueImage, hueValue - hueRange, hueValue + hueRange, hueMask);
    
    // if the desired color is near the border of the hue space, check the other side too:
    // TODO: this won't work if "hueValue + hueRange > 180" - maybe use two different if-cases instead... with int lowerHueValue = hueValue - 180
    if (hueValue - hueRange < 0 || hueValue + hueRange > 180)
    {
        Mat hueMaskUpper;
        int upperHueValue = hueValue + 180; // in reality this would be + 360 instead
        inRange(hueImage, upperHueValue - hueRange, upperHueValue + hueRange, hueMaskUpper);
        
        // add this mask to the other one
        hueMask = hueMask | hueMaskUpper;
    }
    // now we have to filter out all the pixels where saturation and value do not fit the limits:
    Mat saturationMask = hsvChannels[1] > minSaturation;
    Mat valueMask = hsvChannels[2] > minValue;
    
    hueMask = (hueMask & saturationMask) & valueMask;
    vector<cv::Vec4i> start_lines;
    HoughLinesP(hueMask, start_lines, 1, CV_PI / 360, 50, 50, 10);
    find_start_lines(hueMask, start_lines);

    //=============
    
    vector<cv::Vec4i> lines_set;
    cv::HoughLinesP(edge_frame,lines_set,1,PI/180,30,30,10);
#ifdef CAMERA_SHOW_MORE
    show_lines(roi_gray_ch3, lines_set);
#endif
    vector<Vec4i> right_lines;
    vector<Vec4i> left_lines;
    split_left_right(lines_set, left_lines, right_lines);
#ifdef CAMERA_SHOW
    show_lines(roi, left_lines, Sky, 2);
    show_lines(roi, right_lines, Pink, 2);
#endif
    float ldistance, rdistance;
    find_lines(roi, left_lines, right_lines, &rdistance, &ldistance);
    
    int differ=rdistance-ldistance;
#ifdef CAMERA_SHOW
    circle (roi, Point(XHalf, YPoint), 5, Scalar(250,250,250),-1);
    circle(roi, Point(XHalf + differ, YPoint), 5, Scalar(0, 0, 255), 2);
    putText(roi,format("%3d - %3d = %3d",(int)rdistance, (int)ldistance, differ),Point(XHalf-100,YHalf/2),FONT_HERSHEY_SIMPLEX,0.5,Yellow,2);
    imshow("roi",roi);
    imshow("edgeframe",edge_frame);
#endif
#ifdef CAMERA_SHOW_MORE
    imshow("frame", frame);
    imshow("roi_gray_ch3", roi_gray_ch3);
    imshow("start_roi", start_roi);
    imshow("desired color", hueMask);
#endif
    return differ;
}

int main(int argc, char**argv)
{
    //VideoCapture cap(0);
    //VideoCapture cap("/Users/k0ldbrew/Desktop/start_line_detection.mp4");
    //VideoCapture cap("/Users/k0ldbrew/Desktop/clockwise.mp4");
    VideoCapture cap("/Users/k0ldbrew/Desktop/anticlockwise.mp4");
    Mat frame;
    
    if(!cap.isOpened()){
        std::cout<<"no camera!"<< std::endl;
        return -1;
    }
    
#ifdef ROS
    ros::init(argc, argv, "cam_msg_publisher");
    ros::NodeHandle nh;
    std_msgs::Int16 cam_msg;
    ros::Publisher pub = nh.advertise<std_msgs::Int16>("cam_msg",100);
    
    int init_past=1;
    //--------------------------------------------------
    ros::Rate loop_rate(50);
    cout<<"start"<<endl;
#endif
    
    int differ, key, fr_no = 0;
    bool capture = true;
    //clock_t tStart = clock();
    for(;;){
        if (capture) {
            cap >> frame;
            if (frame.empty())
                break;
        }
        
        
        if ((key = waitKey(30)) >= 0) {
            if (key == 27)
                break;
            else if (key = ' ') {
                capture = !capture;
            }
        }
        
        if (capture == false)
            continue;
        
        fr_no++;
        resize(frame,frame,Size(Width,Height));
        printf("Width: %d\n", frame.rows);
        differ = img_process(frame);
        
#ifdef ROS
        cam_msg.data = differ;
        pub.publish(cam_msg);
        loop_rate.sleep();
#else
        cout << fr_no << ":" << differ << endl;
#endif
    }
    
    cout<<"Camera off"<<endl;
    return 0;
}


