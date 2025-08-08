#ifndef DRACO_IO_XYZ_WRITER_H_
#define DRACO_IO_XYZ_WRITER_H_

#include <string>

#include "draco/core/status.h"
#include "draco/point_cloud/point_cloud.h"

namespace draco {

// Writes the given PointCloud into an XYZ file.
// Each line of the file will contain three floating point values representing
// X, Y and Z coordinates. If the point cloud contains a COLOR attribute, three
// additional integer values (R G B in [0,255]) will be written per line.
Status WriteXyzPointCloudToFile(const PointCloud &point_cloud,
                                const std::string &file_name);

}  // namespace draco

#endif  // DRACO_IO_XYZ_WRITER_H_
