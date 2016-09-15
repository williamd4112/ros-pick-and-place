#include "rs_util.hpp"

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

void rs_color_to_cvmat(rs::device & dev, cv::Mat & img)
{
	const rs::intrinsics color_intrin = dev.get_stream_intrinsics(rs::stream::rectified_color);

	const uint8_t * color_frame = reinterpret_cast<const uint8_t *>(dev.get_frame_data(rs::stream::color));
	
	img = cv::Mat(color_intrin.height, color_intrin.width, CV_8UC3, (void*) color_frame);
        cv::cvtColor(img, img, cv::COLOR_RGB2BGR);

}
