// ============================================================================
//  Header
// ============================================================================

#include <ufr.h>

extern ufr_gtw_api_t ufr_gtw_link_api;
extern ufr_gtw_api_t ufr_gtw_opencv_api;


int  ufr_gtw_video_boot(link_t* link, const ufr_args_t* args) {
    int id = ufr_args_geti(args, "@id", -1);
    if ( id >= 0 ) {
        link->gtw_api = &ufr_gtw_opencv_api;
    } else {
        link->gtw_api = &ufr_gtw_link_api;
    }
    return ufr_boot_gtw(link, args);
   
}


ufr_gtw_api_t ufr_gtw_video_api = {
	.type = NULL,
	.state = NULL,
	.size = NULL,

	.boot = ufr_gtw_video_boot,
	.start = NULL,
	.stop = NULL,
	.copy = NULL,

	.read = NULL,
	.write = NULL,

	.recv = NULL,
	.recv_async = NULL,

    .accept = NULL,

    .test_args = NULL,
};


// ============================================================================
//  Public Function
// ============================================================================

int ufr_gtw_video_new_topic(link_t* link, int type) {
    link->gtw_api = &ufr_gtw_video_api;
    link->dcr_api = NULL;
    link->enc_api = NULL;
    return UFR_OK;
}