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

  std::vector<uint8_t> colors;
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

    int r, g, b;
    if (iss >> r >> g >> b) {
      colors.push_back(static_cast<uint8_t>(r));
      colors.push_back(static_cast<uint8_t>(g));
      colors.push_back(static_cast<uint8_t>(b));
    } else {
      // If there are remaining tokens but we failed to parse all three color
      // components, treat it as an error.
      if (!(iss >> std::ws).eof()) {
        return Status(Status::DRACO_ERROR, "Invalid color data in .xyz file.");
      }
    }
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

  if (!colors.empty()) {
    if (colors.size() / 3 != num_points) {
      return Status(Status::DRACO_ERROR,
                    "Color count does not match point count in .xyz file.");
    }
    GeometryAttribute ca;
    ca.Init(GeometryAttribute::COLOR, nullptr, 3, DT_UINT8, true,
            sizeof(uint8_t) * 3, 0);
    const int c_att_id = out_point_cloud->AddAttribute(ca, true, num_points);
    PointAttribute *color_att = out_point_cloud->attribute(c_att_id);
    for (int i = 0; i < num_points; ++i) {
      color_att->SetAttributeValue(AttributeValueIndex(i), &colors[i * 3]);
    }
  }
  return OkStatus();
}

}  // namespace draco
