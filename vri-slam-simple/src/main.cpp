#include <stdio.h>
#include <stdlib.h>
#include <ufr.h>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <vector>

#define LIDAR_SIZE 360

using namespace std;
using namespace cv;





float BoxIntegral(Mat& img, int row, int col, int rows, int cols)  {
    // The subtraction by one for row/col is because row/col is inclusive.
    int r1 = std::min(row,          img.rows) - 1;
    int c1 = std::min(col,          img.cols)  - 1;
    int r2 = std::min(row + rows,   img.rows) - 1;
    int c2 = std::min(col + cols,   img.cols)  - 1;

    float A(0.0f), B(0.0f), C(0.0f), D(0.0f);
    if (r1 >= 0 && c1 >= 0) A = img.at<float>(r1, c1);
    if (r1 >= 0 && c2 >= 0) B = img.at<float>(r1, c2);
    if (r2 >= 0 && c1 >= 0) C = img.at<float>(r2, c1);
    if (r2 >= 0 && c2 >= 0) D = img.at<float>(r2, c2);

    return std::max(0.f, A - B - C + D);
}

void hessian(Mat& img, Mat& res, int rl_filter, float threshold, float value) {

	const int rl_step = 1;
	const int step = rl_step;                      // step size for this filter
	const int b = (rl_filter - 1) / 2 + 1;         // border for this filter
	const int l = rl_filter / 3;                   // lobe for this filter (filter size / 3)
	const int w = rl_filter;                       // filter size
	const float inverse_area = 1.f/(w*w);           // normalisation factor

	for(int iy = 0; iy < img.rows; ++iy) {
		for(int ix = 0; ix < img.cols; ++ix) {
			// get the image coordinates
			int r = iy * step;
			int c = ix * step; 

			// Compute response components
			float Dxx = BoxIntegral(img, r - l + 1, c - b, 2*l - 1, w)
				- BoxIntegral(img, r - l + 1, c - l / 2, 2*l - 1, l)*3;
			
			float Dyy = BoxIntegral(img, r - b, c - l + 1, w, 2*l - 1)
				- BoxIntegral(img, r - l / 2, c - l + 1, l, 2*l - 1)*3;
	
			float Dxy = 
				+ BoxIntegral(img, r - l, c + 1, l, l)
				+ BoxIntegral(img, r + 1, c - l, l, l)
				- BoxIntegral(img, r - l, c - l, l, l)
				- BoxIntegral(img, r + 1, c + 1, l, l);

			// Normalise the filter responses with respect to their size
			Dxx *= inverse_area;
			Dyy *= inverse_area;
			Dxy *= inverse_area;
			
			// Get the determinant of hessian response & laplacian sign
			// responses[index] = (Dxx * Dyy - 0.81f * Dxy * Dxy);
			// laplacian[index] = (Dxx + Dyy >= 0 ? 1 : 0);
			float val = (Dxx * Dyy - 0.81f * Dxy * Dxy);//  * Dyy - 0.81f * Dxy * Dxy;

			res.at<float>(iy,ix) = val;
		}
	}

}

void filter(Mat& img, Mat& res) {
    res = Mat(img.size(), CV_32F, Scalar(0));

    for(int iy = 1; iy < img.rows-1; ++iy) {
		for(int ix = 1; ix < img.cols-1; ++ix) {
            float val = img.at<float>(iy,ix);
            if ( val <= img.at<float>(iy,ix-1) ) {
                res.at<float>(iy,ix) = 0.0;
            } else if ( val <= img.at<float>(iy,ix+1) ) {
                res.at<float>(iy,ix) = 0.0;
            } else if ( val <= img.at<float>(iy-1,ix) ) {
                res.at<float>(iy,ix) = 0.0;
            } else if ( val <= img.at<float>(iy+1,ix) ) {
                res.at<float>(iy,ix) = 0.0;
            } else {
                res.at<float>(iy,ix) = img.at<float>(iy,ix);
            }
        }
    }
}




int main() {
    link_t lidar_dev = ufr_sys_open("lidar", "@new zmq:topic @host 192.168.43.141 @port 5002 @coder msgpack:obj");
    lt_start_subscriber(&lidar_dev, NULL);

    link_t pose_dev = ufr_sys_open("pose", "@new zmq:topic @host 192.168.43.141 @port 5004 @coder msgpack:obj");
    lt_start_subscriber(&pose_dev, NULL);

    Mat map(600, 600, CV_8UC1);

    int world_r_y = 300;
    int world_r_x = 300;

    while (1) {
        map.setTo(0);
        float lidar[LIDAR_SIZE];
        lt_get(&lidar_dev, "^af", LIDAR_SIZE, lidar);
        
        float robot_x, robot_y, robot_th;
        lt_get(&pose_dev, "^fff", &robot_x, &robot_y, &robot_th);
        float robot_th_rad = robot_th * M_PI / 180.0;

        printf("%f %f %f\n", robot_x, robot_y, robot_th );

        float angulo = -M_PI/2.0;
        float diff_angulo = 3.141592*2 / LIDAR_SIZE;
        for (int i=0;   i<LIDAR_SIZE;   i++, angulo += diff_angulo) {
            float dist = lidar[i] / 50.0;
            // printf("%f ", dist);
            uint16_t y = world_r_y - ( sin(angulo-robot_th_rad) * dist * 1.0 );
            uint16_t x = world_r_x + ( cos(angulo-robot_th_rad) * dist * 1.0 );
            

            map.at<uint8_t>(y,x) = 255;

            // circle(map,Point(x,y), 2, (255), -1);
        }
        // printf("\n");

/*
        vector<Vec2f> lines; // will hold the results of the detection
        HoughLines(map, lines, 1, CV_PI/45, 20, 0, 0 );

        printf("linhas: %d\n", lines.size());
        for( size_t i = 0; i < lines.size(); i++ )
        {
            float rho = lines[i][0], theta = lines[i][1];
            Point pt1, pt2;
            double a = cos(theta), b = sin(theta);
            double x0 = a*rho, y0 = b*rho;
            pt1.x = cvRound(x0 + 1000*(-b));
            pt1.y = cvRound(y0 + 1000*(a));
            pt2.x = cvRound(x0 - 1000*(-b));
            pt2.y = cvRound(y0 - 1000*(a));
            line( map, pt1, pt2, 255, 2, LINE_AA);
        }
*/

/*        
        Mat isum, res;
        integral(map,isum,CV_32F);
		Mat haar(isum.size(), CV_32F, Scalar(0));
        hessian(isum, haar, 7, 5.0, 0.0125);
        // hessian(isum, haar, 21, 5.0, 0.0125);
        // hessian(isum, haar, 21, 5.0, 0.0125);
        // hessian(isum, haar, 33, 5.0, 0.0125);
        // filter(haar, res);
        imshow("surf", haar);
*/
        
        imshow("map", map);
        waitKey(10);
    }
    return 0;
}