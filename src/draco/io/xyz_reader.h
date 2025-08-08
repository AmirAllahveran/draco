#ifndef DRACO_IO_XYZ_READER_H_
#define DRACO_IO_XYZ_READER_H_

#include <string>

#include "draco/core/status.h"
#include "draco/point_cloud/point_cloud.h"

namespace draco {

// Reads an XYZ point cloud file into the provided PointCloud.
// Each line of the file should contain three floating point values representing
// X, Y and Z coordinates. Optionally, three additional integer values can be
// provided to specify RGB color in the range [0,255]. Lines starting with '#'
// are treated as comments.
Status ReadXyzPointCloudFromFile(const std::string &file_name,
                                 PointCloud *out_point_cloud);

}  // namespace draco

#endif  // DRACO_IO_XYZ_READER_H_
