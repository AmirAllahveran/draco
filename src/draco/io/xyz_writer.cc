#include "draco/io/xyz_writer.h"

#include <array>
#include <fstream>

#include "draco/attributes/geometry_attribute.h"
#include "draco/attributes/point_attribute.h"

namespace draco {

Status WriteXyzPointCloudToFile(const PointCloud &point_cloud,
                                const std::string &file_name) {
  const PointAttribute *pos_att =
      point_cloud.GetNamedAttribute(GeometryAttribute::POSITION);
  if (pos_att == nullptr) {
    return Status(Status::DRACO_ERROR, "Point cloud has no positions.");
  }
  std::ofstream out(file_name);
  if (!out) {
    return Status(Status::DRACO_ERROR, "Unable to open output file.");
  }
  const int num_points = point_cloud.num_points();
  std::array<float, 3> pos;
  for (int i = 0; i < num_points; ++i) {
    pos_att->GetMappedValue(PointIndex(i), pos.data());
    out << pos[0] << " " << pos[1] << " " << pos[2] << "\n";
  }
  if (!out) {
    return Status(Status::DRACO_ERROR, "Failed while writing to file.");
  }
  return OkStatus();
}

}  // namespace draco
