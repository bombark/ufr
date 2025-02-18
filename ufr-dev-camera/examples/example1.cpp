// ============================================================================
//  Header
// ============================================================================

#include <ufr.h>
#include "opencv2/opencv.hpp"

using namespace std;
using namespace cv;


// ============================================================================
//  Test
// ============================================================================

int main() {
    // link_t video = ufr_subscriber("@new video:topic @@new zmq:topic @@coder msgpack @@port 3000");
    link_t video = ufr_subscriber("@new video:topic @id 0");
    
    while(1) {
        printf("opa\n");
        if ( ufr_recv(&video) != UFR_OK ) {
            break;
        }

        // int size = ufr_get_size(&video);
        void* data = (void*) ufr_get_raw_ptr(&video);
        int size[2] = {480, 640};
        Mat image(2, size, CV_8UC3, data, 0);
        imshow("janela", image);
        waitKey(1);
    }

    ufr_close(&video);
    return 0;
}