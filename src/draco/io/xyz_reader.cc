#include "draco/io/xyz_reader.h"

#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "draco/attributes/geometry_attribute.h"
#include "draco/attributes/point_attribute.h"

namespace draco {

Status ReadXyzPointCloudFromFile(const std::string &file_name,
                                 PointCloud *out_point_cloud) {
  std::ifstream in(file_name);
  if (!in) {
    return Status(Status::DRACO_ERROR, "Unable to read input file.");
  }
  std::string line;
  std::vector<float> coords;
  while (std::getline(in, line)) {
    if (line.empty() || line[0] == '#') {
      continue;
    }
    std::istringstream iss(line);
    float x, y, z;
    if (!(iss >> x >> y >> z)) {
      return Status(Status::DRACO_ERROR, "Invalid line in .xyz file.");
    }
    coords.push_back(x);
    coords.push_back(y);
    coords.push_back(z);
  }
  const int num_points = coords.size() / 3;
  out_point_cloud->set_num_points(num_points);
  GeometryAttribute va;
  va.Init(GeometryAttribute::POSITION, nullptr, 3, DT_FLOAT32, false,
          sizeof(float) * 3, 0);
  const int att_id = out_point_cloud->AddAttribute(va, true, num_points);
  PointAttribute *pos_att = out_point_cloud->attribute(att_id);
  for (int i = 0; i < num_points; ++i) {
    pos_att->SetAttributeValue(AttributeValueIndex(i), &coords[i * 3]);
  }
  return OkStatus();
}

}  // namespace draco
