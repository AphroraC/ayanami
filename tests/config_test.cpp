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
