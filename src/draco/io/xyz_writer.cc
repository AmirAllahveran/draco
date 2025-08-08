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

  const PointAttribute *color_att =
      point_cloud.GetNamedAttribute(GeometryAttribute::COLOR);
  const bool has_color =
      color_att != nullptr && color_att->num_components() >= 3;

  const int num_points = point_cloud.num_points();
  std::array<float, 3> pos;
  std::array<uint8_t, 4> color;
  for (int i = 0; i < num_points; ++i) {
    pos_att->GetMappedValue(PointIndex(i), pos.data());
    out << pos[0] << " " << pos[1] << " " << pos[2];
    if (has_color) {
      color_att->ConvertValue(AttributeValueIndex(i), 3, color.data());
      out << " " << static_cast<int>(color[0]) << " "
          << static_cast<int>(color[1]) << " " << static_cast<int>(color[2]);
    }
    out << "\n";
  }
  if (!out) {
    return Status(Status::DRACO_ERROR, "Failed while writing to file.");
  }
  return OkStatus();
}

}  // namespace draco
