#include <gtest/gtest.h>
#include <ayanami/utility/config.hpp>
#include <Eigen/Dense>

#include <filesystem>
#include <fstream>
#include <vector>

namespace {

std::filesystem::path make_temp_config_dir() {
  std::filesystem::path dir =
      std::filesystem::temp_directory_path() / "ayanami_config_test";
  std::filesystem::remove_all(dir);
  std::filesystem::create_directories(dir);
  return dir;
}

void write_json(const std::filesystem::path& path,
                const std::string& content) {
  std::ofstream ofs(path);
  ofs << content;
}

} // namespace

TEST(ConfigTest, EmptyFileGracefulReturn) {
  // 文件打不开 -> spdlog::error + 空 json，param 回退 default
  auto dir = make_temp_config_dir();
  ayanami::Config cfg((dir / "nonexistent.json").string());
  EXPECT_EQ(cfg.param<int>("any", "any", 42), 42);
}

TEST(ConfigTest, EmptyOrInvalidJsonGracefulReturn) {
  // 文件存在但内容非法 JSON -> nlohmann parse_error 被捕获 -> spdlog::error +
  // 空 json，param 回退 default（不 abort）
  auto dir = make_temp_config_dir();
  write_json(dir / "invalid_config.json", R"({ "unclosed": )");
  ayanami::Config cfg((dir / "invalid_config.json").string());
  EXPECT_EQ(cfg.param<int>("any", "any", 42), 42);
  EXPECT_FALSE(cfg.has_param("any", "any"));
}

TEST(ConfigTest, EigenIsometry3dVector) {
  // TUM 7 元 x2: [x,y,z,qx,qy,qz,qw] * 2
  auto dir = make_temp_config_dir();
  write_json(dir / "test.json",
             R"({ "p": { "vec": [1,2,3,0,0,0,1, 4,5,6,0,0,0,1] } })");
  ayanami::Config cfg((dir / "test.json").string());
  auto vec = cfg.param<std::vector<Eigen::Isometry3d>>(
      "p", "vec", std::vector<Eigen::Isometry3d>{});
  ASSERT_EQ(vec.size(), 2u);
  EXPECT_NEAR(vec[0].translation().x(), 1.0, 1e-6);
  EXPECT_NEAR(vec[1].translation().x(), 4.0, 1e-6);
  EXPECT_NEAR(vec[1].translation().y(), 5.0, 1e-6);
}

TEST(ConfigTest, LoadAndParamScalar) {
  auto dir = make_temp_config_dir();
  write_json(dir / "test.json", R"({ "m": { "k": 42, "v": "hello" } })");
  ayanami::Config cfg((dir / "test.json").string());
  EXPECT_EQ(cfg.param<int>("m", "k", 0), 42);
  EXPECT_EQ(cfg.param<std::string>("m", "v", ""), "hello");
  EXPECT_EQ(cfg.param<int>("m", "missing", 7), 7);  // default
}

TEST(ConfigTest, VectorStringParam) {
  // extension_modules 等字符串数组经 Config API 读取。
  auto dir = make_temp_config_dir();
  write_json(dir / "test.json",
             R"({ "m": { "ext": ["librviz_viewer.so", "libecho.so"] } })");
  ayanami::Config cfg((dir / "test.json").string());
  const auto v = cfg.param<std::vector<std::string>>("m", "ext", {});
  ASSERT_EQ(v.size(), 2u);
  EXPECT_EQ(v[0], "librviz_viewer.so");
  EXPECT_EQ(v[1], "libecho.so");
  EXPECT_TRUE(cfg.param<std::vector<std::string>>("m", "missing", {}).empty());
}

TEST(ConfigTest, NestedVectorStringParam) {
  // tf_sync_edges 所需 [[target, source], ...] (std::vector<std::vector<std::string>>)。
  auto dir = make_temp_config_dir();
  write_json(dir / "test.json",
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

TEST(ConfigTest, Vector2dFootprintParam) {
  auto dir = make_temp_config_dir();
  write_json(dir / "test.json",
             R"({ "costmap": { "footprint": [[-0.2,-0.1],[0.2,0.1]] } })");
  ayanami::Config cfg((dir / "test.json").string());
  const auto footprint = cfg.param<std::vector<Eigen::Vector2d>>(
      "costmap", "footprint", {});
  ASSERT_EQ(footprint.size(), 2u);
  EXPECT_DOUBLE_EQ(footprint[0].x(), -0.2);
  EXPECT_DOUBLE_EQ(footprint[1].y(), 0.1);
}

TEST(ConfigTest, EigenTrait) {
  // TUM 7 元 [x,y,z,qx,qy,qz,qw]; JSON 带注释验证 ignore_comments=true
  auto dir = make_temp_config_dir();
  write_json(dir / "test.json",
             R"({ "p": { "t": [1,2,3,0,0,0,1] } }  // TUM 7 元
)");
  ayanami::Config cfg((dir / "test.json").string());
  auto t = cfg.param<Eigen::Isometry3d>("p", "t",
                                        Eigen::Isometry3d::Identity());
  EXPECT_NEAR(t.translation().x(), 1.0, 1e-6);
  EXPECT_NEAR(t.translation().y(), 2.0, 1e-6);
  EXPECT_NEAR(t.translation().z(), 3.0, 1e-6);
}

TEST(GlobalConfigTest, ConfigsSubdirFallback) {
  // §2.2 branch ④: 目录下无 config.json 但 configs/config.json 存在 -> 切到 configs/
  auto dir = make_temp_config_dir();
  std::filesystem::create_directories(dir / "configs");
  write_json(dir / "configs" / "config.json", R"({ "global": {} })");
  ayanami::GlobalConfig* g = ayanami::GlobalConfig::instance(dir.string(), true);
  ASSERT_NE(g, nullptr);
  EXPECT_EQ(g->param<std::string>("global", "config_path", ""),
            (dir / "configs").string());
}

TEST(GlobalConfigTest, InstanceOnce) {
  auto dir = make_temp_config_dir();
  write_json(dir / "config.json", R"({ "global": { "config_path": "" } })");
  ayanami::GlobalConfig* a = ayanami::GlobalConfig::instance(dir.string());
  ayanami::GlobalConfig* b = ayanami::GlobalConfig::instance();
  EXPECT_EQ(a, b);
  EXPECT_FALSE(ayanami::GlobalConfig::get_config_path("config_xyz").empty());
}

TEST(GlobalConfigTest, PathResolution) {
  // config_path 是文件 -> 取父目录; override 重建 + config_path 写回
  auto dir = make_temp_config_dir();
  write_json(dir / "config.json", R"({ "global": { "config_path": "" } })");
  ayanami::GlobalConfig* g =
      ayanami::GlobalConfig::instance((dir / "config.json").string(), true);
  ASSERT_NE(g, nullptr);
  EXPECT_EQ(g->param<std::string>("global", "config_path", ""),
            dir.string());
}
