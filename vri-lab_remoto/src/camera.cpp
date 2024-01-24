#include <stdio.h>
#include <stdlib.h>
#include <ufr.h>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

using namespace std;
using namespace cv;


int main() {
    link_t camera_pub = ufr_sys_open("lidar", "@new mqtt:topic @host 185.209.160.8 @topic teste @coder msgpack:obj");
    lt_start_publisher(&camera_pub, NULL);

    Mat image;
    VideoCapture video(0);
    FILE* fd = fopen("imagem.jpg", "wb+");

    static uint8_t data[1024*1024*2];

    while (1) {
        video >> image;

        // convert to JPEG
        imwrite("imagem.jpg", image);
        fseek(fd, 0, SEEK_SET);
        size_t data_size = fread(data, 1, sizeof(data), fd);

        // send JPG to topic
        lt_write(&camera_pub, (const char*) data, data_size);
        imshow("map", image);
        waitKey(100);
    }
    return 0;
}