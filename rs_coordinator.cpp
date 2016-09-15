#include "rs_coordinator.hpp"

RsCoordinator(rs::device & dev)
{
}

RsCoordinator::~RsCoordinator()
{
}

void RsCoordinator::update(const uint16_t * depth_frame)
{
	float scale = dev.get_depth_scale();
	for (int dy = 0; dy < m_intrinsic.height; ++dy) {
            for (int dx = 0; dx < m_intrinsic.width; ++dx) {
                uint16_t depth_value = depth_frame[dy * m_intrinsic.width + dx];
                float depth_in_meters = depth_value * scale;
                if(depth_value >= m_max_depth * one_meter 
				|| depth_value == 0) continue;

                rs::float2 depth_pixel = {(float)dx, (float)dy};
                rs::float3 depth_point = depth_intrin.deproject(depth_pixel, depth_in_meters);
                rs::float3 color_point = depth_to_color.transform(depth_point);
                rs::float2 color_pixel = color_intrin.project(color_point);

                const int cx = (int)std::round(color_pixel.x), cy = (int)std::round(color_pixel.y); 
                g_pointcloud[cy * FRAME_WIDTH + cx] = depth_point;
            }
        }
 
}

