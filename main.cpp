//
// Created by yangjq on 22-8-4.
//
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

struct userdata{
    Mat im;
    vector<Point2f> points;
};

void mouseHandler(int event, int x, int y, int flags, void* data_ptr)
{
    if  ( event == EVENT_LBUTTONDOWN )
    {
        userdata *data = ((userdata *) data_ptr);
        circle(data->im, Point(x,y),3,Scalar(0,255,255), 5, cv::LINE_AA);
        imshow("Image", data->im);
        data->points.push_back(Point2f(x,y));
    }
}

int main( int argc, char** argv)
{

    // Read in the image.导入需要被映射的图片
    Mat im_src = imread("/home/yangjq/Videos/radar3/test1_camera_img/60.bmp");

    Mat im_temp = im_src.clone();
    userdata data;
    data.im = im_temp;

    Mat mask = Mat::zeros(im_src.size(), CV_8UC1);
    vector<vector<Point2i>> contours;
    Mat im_src_cut;

    cout << "Starting from the upper left corner, take points clockwise!" << endl;
    cout << "Click on N corners of a billboard and then press ENTER!" << endl;
    namedWindow("Image", 1);
    //set the callback function for any mouse event
    setMouseCallback("Image", mouseHandler, &data);
    //show the image
    imshow("Image", im_temp);
    waitKey(0);

    vector<Point2i> contour;
    for(int i = 0; i < data.points.size(); i++)
    {
        contour.push_back(Point2i(data.points[i].x,data.points[i].y));
    }
    contours.push_back(contour);
    //contours必须是vector<vector<Point2i>>
    drawContours(mask, contours, 0, Scalar::all(255), -1);
    im_src.copyTo(im_src_cut, mask);
    cout << "If OK! Please press ENTER!" << endl;
    imshow("Image", im_src_cut);
    waitKey(0);

    // Destination image.导入背景图片
    Mat im_dst = imread("/home/yangjq/CLionProjects/ComputeHomo/1.png");

    // Set data for mouse handler
    im_temp = im_dst.clone();
    userdata data2;
    data2.im = im_temp;

    //show the image
    imshow("Image", im_temp);

    cout<< "Select the same number of points according to the previous corresponding points!" << endl;
    cout << "Click on N corners of a billboard and then press ENTER!" << endl;
    //set the callback function for any mouse event
    setMouseCallback("Image", mouseHandler, &data2);
    waitKey(0);

    // Calculate Homography between source and destination points
    Mat h = findHomography(data.points, data2.points);

    // Warp source image
    warpPerspective(im_src_cut, im_temp, h, im_temp.size());

    // Extract four points from mouse data
    const int N = data.points.size(); //选取的匹配点的个数
    Point pts_dst[N];
    for( int i = 0; i < N; i++)
    {
        pts_dst[i] = data2.points[i];
    }

    // Black out polygonal area in destination image.
    fillConvexPoly(im_dst, pts_dst, N, Scalar(0), cv::LINE_AA);

    // Add warped source image to destination image.
    im_dst = im_dst + im_temp;

    // Display image.
    imshow("Image", im_dst);
    waitKey(0);

    return 0;
}
