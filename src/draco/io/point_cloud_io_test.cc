// Copyright 2016 The Draco Authors.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
#include "draco/io/point_cloud_io.h"

#include <sstream>

#include "draco/core/draco_test_base.h"
#include "draco/core/draco_test_utils.h"
#include "draco/io/obj_decoder.h"
#include "draco/io/xyz_writer.h"

namespace draco {

class IoPointCloudIoTest : public ::testing::Test {
 protected:
  void test_compression_method(PointCloudEncodingMethod method,
                               int expected_num_attributes,
                               const std::string &file_name) {
    const std::unique_ptr<PointCloud> encoded_pc =
        ReadPointCloudFromTestFile(file_name);
    ASSERT_NE(encoded_pc, nullptr) << "Failed to load test model " << file_name;
    ASSERT_GE(encoded_pc->num_attributes(), expected_num_attributes)
        << "Failed to load test model: " << file_name
        << " wrong number of attributes" << std::endl;

    // Set quantization.
    EncoderOptions options = EncoderOptions::CreateDefaultOptions();
    for (int i = 0; i <= GeometryAttribute::NAMED_ATTRIBUTES_COUNT; i++) {
      options.SetAttributeInt(GeometryAttribute::Type(i), "quantization_bits",
                              14);
    }

    std::stringstream ss;
    WritePointCloudIntoStream(encoded_pc.get(), ss, method, options);
    ASSERT_TRUE(ss.good());

    std::unique_ptr<PointCloud> decoded_pc;
    ReadPointCloudFromStream(&decoded_pc, ss);
    ASSERT_TRUE(ss.good());

    for (int i = 0; i <= GeometryAttribute::NAMED_ATTRIBUTES_COUNT; i++) {
      ASSERT_EQ(encoded_pc->NumNamedAttributes(GeometryAttribute::Type(i)),
                decoded_pc->NumNamedAttributes(GeometryAttribute::Type(i)));
    }

    ASSERT_EQ(encoded_pc->num_points(), decoded_pc->num_points());
  }
};

TEST_F(IoPointCloudIoTest, EncodeSequentialPointCloudTestNmObj) {
  test_compression_method(POINT_CLOUD_SEQUENTIAL_ENCODING, 2, "test_nm.obj");
}
TEST_F(IoPointCloudIoTest, EncodeSequentialPointCloudTestPosObj) {
  test_compression_method(POINT_CLOUD_SEQUENTIAL_ENCODING, 1,
                          "point_cloud_test_pos.obj");
}
TEST_F(IoPointCloudIoTest, EncodeSequentialPointCloudTestPosPly) {
  test_compression_method(POINT_CLOUD_SEQUENTIAL_ENCODING, 1,
                          "point_cloud_test_pos.ply");
}
TEST_F(IoPointCloudIoTest, EncodeSequentialPointCloudTestPosXyz) {
  test_compression_method(POINT_CLOUD_SEQUENTIAL_ENCODING, 1,
                          "point_cloud_test_pos.xyz");
}
TEST_F(IoPointCloudIoTest, EncodeSequentialPointCloudTestPosNormObj) {
  test_compression_method(POINT_CLOUD_SEQUENTIAL_ENCODING, 2,
                          "point_cloud_test_pos_norm.obj");
}
TEST_F(IoPointCloudIoTest, EncodeSequentialPointCloudTestPosNormPly) {
  test_compression_method(POINT_CLOUD_SEQUENTIAL_ENCODING, 2,
                          "point_cloud_test_pos_norm.ply");
}

TEST_F(IoPointCloudIoTest, EncodeKdTreePointCloudTestPosObj) {
  test_compression_method(POINT_CLOUD_KD_TREE_ENCODING, 1,
                          "point_cloud_test_pos.obj");
}
TEST_F(IoPointCloudIoTest, EncodeKdTreePointCloudTestPosPly) {
  test_compression_method(POINT_CLOUD_KD_TREE_ENCODING, 1,
                          "point_cloud_test_pos.ply");
}
TEST_F(IoPointCloudIoTest, EncodeKdTreePointCloudTestPosXyz) {
  test_compression_method(POINT_CLOUD_KD_TREE_ENCODING, 1,
                          "point_cloud_test_pos.xyz");
}

TEST_F(IoPointCloudIoTest, ObjFileInput) {
  // Tests whether loading obj point clouds from files works as expected.
  const std::unique_ptr<PointCloud> pc =
      ReadPointCloudFromTestFile("test_nm.obj");
  ASSERT_NE(pc, nullptr) << "Failed to load the obj point cloud.";
  EXPECT_EQ(pc->num_points(), 97) << "Obj point cloud not loaded properly.";
}
TEST_F(IoPointCloudIoTest, XyzFileInput) {
  const std::unique_ptr<PointCloud> pc =
      ReadPointCloudFromTestFile("point_cloud_test_pos.xyz");
  ASSERT_NE(pc, nullptr) << "Failed to load the xyz point cloud.";
  EXPECT_EQ(pc->num_points(), 4) << "Xyz point cloud not loaded properly.";
}

// Test if we handle wrong input for all file extensions.
TEST_F(IoPointCloudIoTest, WrongFileObj) {
  const std::unique_ptr<PointCloud> pc =
      ReadPointCloudFromTestFile("wrong_file_name.obj");
  ASSERT_EQ(pc, nullptr);
}
TEST_F(IoPointCloudIoTest, WrongFilePly) {
  const std::unique_ptr<PointCloud> pc =
      ReadPointCloudFromTestFile("wrong_file_name.ply");
  ASSERT_EQ(pc, nullptr);
}
TEST_F(IoPointCloudIoTest, WrongFileXyz) {
  const std::unique_ptr<PointCloud> pc =
      ReadPointCloudFromTestFile("wrong_file_name.xyz");
  ASSERT_EQ(pc, nullptr);
}
TEST_F(IoPointCloudIoTest, WrongFile) {
  const std::unique_ptr<PointCloud> pc =
      ReadPointCloudFromTestFile("wrong_file_name");
  ASSERT_EQ(pc, nullptr);
}

TEST_F(IoPointCloudIoTest, XyzFileOutput) {
  PointCloud pc;
  pc.set_num_points(2);
  GeometryAttribute va;
  va.Init(GeometryAttribute::POSITION, nullptr, 3, DT_FLOAT32, false,
          sizeof(float) * 3, 0);
  const int att_id = pc.AddAttribute(va, true, 2);
  PointAttribute *pos_att = pc.attribute(att_id);
  const float coords[6] = {0.f, 0.f, 0.f, 1.f, 1.f, 1.f};
  for (int i = 0; i < 2; ++i) {
    pos_att->SetAttributeValue(AttributeValueIndex(i), &coords[i * 3]);
  }
  const std::string file_name =
      GetTestTempFileFullPath("point_cloud_output.xyz");
  ASSERT_TRUE(WriteXyzPointCloudToFile(pc, file_name).ok());
  auto status_or = ReadPointCloudFromFile(file_name);
  ASSERT_TRUE(status_or.ok());
  std::unique_ptr<PointCloud> read_pc = std::move(status_or).value();
  ASSERT_EQ(read_pc->num_points(), 2);
  const PointAttribute *read_pos =
      read_pc->GetNamedAttribute(GeometryAttribute::POSITION);
  ASSERT_NE(read_pos, nullptr);
  float tmp[3];
  read_pos->GetMappedValue(PointIndex(0), tmp);
  EXPECT_FLOAT_EQ(tmp[0], 0.f);
  EXPECT_FLOAT_EQ(tmp[1], 0.f);
  EXPECT_FLOAT_EQ(tmp[2], 0.f);
  read_pos->GetMappedValue(PointIndex(1), tmp);
  EXPECT_FLOAT_EQ(tmp[0], 1.f);
  EXPECT_FLOAT_EQ(tmp[1], 1.f);
  EXPECT_FLOAT_EQ(tmp[2], 1.f);
  std::remove(file_name.c_str());
}

}  // namespace draco
