#include <gtest/gtest.h>
#include <ayanami/utility/config.hpp>
#include <Eigen/Dense>

#include <filesystem>
#include <fstream>
#include <vector>

namespace {

std::filesystem::path makeTempConfigDir() {
  std::filesystem::path dir =
      std::filesystem::temp_directory_path() / "ayanami_config_test";
  std::filesystem::remove_all(dir);
  std::filesystem::create_directories(dir);
  return dir;
}

void writeJson(const std::filesystem::path& path,
                const std::string& content) {
  std::ofstream ofs(path);
  ofs << content;
}

} // namespace

TEST(ConfigTest, emptyFileGracefulReturn) {
  // File cannot be opened -> spdlog::error + empty json, param falls back to default
  auto dir = makeTempConfigDir();
  ayanami::Config cfg((dir / "nonexistent.json").string());
  EXPECT_EQ(cfg.param<int>("any", "any", 42), 42);
}

TEST(ConfigTest, emptyOrInvalidJsonGracefulReturn) {
  // File exists but contains invalid JSON -> the nlohmann parse_error is caught -> spdlog::error +
  // empty json, param falls back to default (no abort)
  auto dir = makeTempConfigDir();
  writeJson(dir / "invalid_config.json", R"({ "unclosed": )");
  ayanami::Config cfg((dir / "invalid_config.json").string());
  EXPECT_EQ(cfg.param<int>("any", "any", 42), 42);
  EXPECT_FALSE(cfg.hasParam("any", "any"));
}

TEST(ConfigTest, eigenIsometry3dVector) {
  // Two TUM 7-element entries: [x,y,z,qx,qy,qz,qw] * 2
  auto dir = makeTempConfigDir();
  writeJson(dir / "test.json",
             R"({ "p": { "vec": [1,2,3,0,0,0,1, 4,5,6,0,0,0,1] } })");
  ayanami::Config cfg((dir / "test.json").string());
  auto vec = cfg.param<std::vector<Eigen::Isometry3d>>(
      "p", "vec", std::vector<Eigen::Isometry3d>{});
  ASSERT_EQ(vec.size(), 2u);
  EXPECT_NEAR(vec[0].translation().x(), 1.0, 1e-6);
  EXPECT_NEAR(vec[1].translation().x(), 4.0, 1e-6);
  EXPECT_NEAR(vec[1].translation().y(), 5.0, 1e-6);
}

TEST(ConfigTest, loadAndParamScalar) {
  auto dir = makeTempConfigDir();
  writeJson(dir / "test.json", R"({ "m": { "k": 42, "v": "hello" } })");
  ayanami::Config cfg((dir / "test.json").string());
  EXPECT_EQ(cfg.param<int>("m", "k", 0), 42);
  EXPECT_EQ(cfg.param<std::string>("m", "v", ""), "hello");
  EXPECT_EQ(cfg.param<int>("m", "missing", 7), 7);  // default
}

TEST(ConfigTest, vectorStringParam) {
  // String arrays such as extension_modules are read via the Config API.
  auto dir = makeTempConfigDir();
  writeJson(dir / "test.json",
             R"({ "m": { "ext": ["libayanami_rviz_viewer.so", "libecho.so"] } })");
  ayanami::Config cfg((dir / "test.json").string());
  const auto v = cfg.param<std::vector<std::string>>("m", "ext", {});
  ASSERT_EQ(v.size(), 2u);
  EXPECT_EQ(v[0], "libayanami_rviz_viewer.so");
  EXPECT_EQ(v[1], "libecho.so");
  EXPECT_TRUE(cfg.param<std::vector<std::string>>("m", "missing", {}).empty());
}

TEST(ConfigTest, nestedVectorStringParam) {
  // tf_sync_edges expects [[target, source], ...] (std::vector<std::vector<std::string>>).
  auto dir = makeTempConfigDir();
  writeJson(dir / "test.json",
             R"({ "m": { "edges": [["map", "odom"], ["odom", "base_link"]] } })");
  ayanami::Config cfg((dir / "test.json").string());
  const auto v =
      cfg.param<std::vector<std::vector<std::string>>>("m", "edges", {});
  ASSERT_EQ(v.size(), 2u);
  ASSERT_EQ(v[0].size(), 2u);
  EXPECT_EQ(v[0][0], "map");
  EXPECT_EQ(v[0][1], "odom");
  ASSERT_EQ(v[1].size(), 2u);
  EXPECT_EQ(v[1][0], "odom");
  EXPECT_EQ(v[1][1], "base_link");
}

TEST(ConfigTest, vector2dFootprintParam) {
  auto dir = makeTempConfigDir();
  writeJson(dir / "test.json",
             R"({ "costmap": { "footprint": [[-0.2,-0.1],[0.2,0.1]] } })");
  ayanami::Config cfg((dir / "test.json").string());
  const auto footprint = cfg.param<std::vector<Eigen::Vector2d>>(
      "costmap", "footprint", {});
  ASSERT_EQ(footprint.size(), 2u);
  EXPECT_DOUBLE_EQ(footprint[0].x(), -0.2);
  EXPECT_DOUBLE_EQ(footprint[1].y(), 0.1);
}

TEST(ConfigTest, eigenTrait) {
  // TUM 7-element [x,y,z,qx,qy,qz,qw]; JSON with comments verifies ignore_comments=true
  auto dir = makeTempConfigDir();
  writeJson(dir / "test.json",
             R"({ "p": { "t": [1,2,3,0,0,0,1] } }  // TUM 7-tuple
)");
  ayanami::Config cfg((dir / "test.json").string());
  auto t = cfg.param<Eigen::Isometry3d>("p", "t",
                                        Eigen::Isometry3d::Identity());
  EXPECT_NEAR(t.translation().x(), 1.0, 1e-6);
  EXPECT_NEAR(t.translation().y(), 2.0, 1e-6);
  EXPECT_NEAR(t.translation().z(), 3.0, 1e-6);
}

TEST(GlobalConfigTest, configsSubdirFallback) {
  // §2.2 branch (4): no config.json in the directory but configs/config.json exists -> switch to configs/
  auto dir = makeTempConfigDir();
  std::filesystem::create_directories(dir / "configs");
  writeJson(dir / "configs" / "config.json", R"({ "global": {} })");
  ayanami::GlobalConfig* g = ayanami::GlobalConfig::instance(dir.string(), true);
  ASSERT_NE(g, nullptr);
  EXPECT_EQ(g->param<std::string>("global", "config_path", ""),
            (dir / "configs").string());
}

TEST(GlobalConfigTest, instanceOnce) {
  auto dir = makeTempConfigDir();
  writeJson(dir / "config.json", R"({ "global": { "config_path": "" } })");
  ayanami::GlobalConfig* a = ayanami::GlobalConfig::instance(dir.string());
  ayanami::GlobalConfig* b = ayanami::GlobalConfig::instance();
  EXPECT_EQ(a, b);
  EXPECT_FALSE(ayanami::GlobalConfig::getConfigPath("config_xyz").empty());
}

TEST(GlobalConfigTest, pathResolution) {
  // config_path is a file -> take the parent directory; the override rebuilds + writes config_path back
  auto dir = makeTempConfigDir();
  writeJson(dir / "config.json", R"({ "global": { "config_path": "" } })");
  ayanami::GlobalConfig* g =
      ayanami::GlobalConfig::instance((dir / "config.json").string(), true);
  ASSERT_NE(g, nullptr);
  EXPECT_EQ(g->param<std::string>("global", "config_path", ""),
            dir.string());
}
