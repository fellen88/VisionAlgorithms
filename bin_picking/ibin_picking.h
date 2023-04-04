#ifndef I_BIN_PICKING_H_
#define I_BIN_PICKING_H_

#include <vector>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#ifdef BINPICKING_EXPORTS
#define BINPICKING_API __declspec(dllexport)
#else
#define BINPICKING_API __declspec(dllimport)
#endif

namespace val  //vision algorithm library
{
	class IBinPicking
	{
	public:
		enum Method
		{
			ModelBased,
			ModelBasedDL,
			ModelFree,
		};
		virtual ~IBinPicking() = 0;
		virtual bool Compute(const pcl::PointCloud<pcl::PointXYZRGBNormal>& object_points, std::vector<double>* object_pose) = 0;
	};
}

BINPICKING_API  val::IBinPicking * GetPtr(char algorithm_vision, std::string config_file);

#endif



