#ifndef _RS_COORDINATOR_HPP_
#define _RS_COORDINATOR_HPP_

#include <librealsense/rs.hpp>

#include <mutex>

class RsCoordinator
{
public:
	RsCoordinator(rs::device & dev);
	~RsCoordinator();

	void update(const uint16_t * depth_frame);
private:
	rs::extrinsic m_extrinsic;
	rs::intrinsic m_intrinsic;
	rs::float3 * m_pointcloud;
	float m_max_depth;
};

#endif
