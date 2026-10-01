#include <gtest/gtest.h>

#include <costmap_2d/cost_values.hpp>
#include <costmap_2d/costmap_2d.hpp>
#include <costmap_2d/pgm_dump.hpp>
#include <costmap_2d/layer.hpp>
#include <costmap_2d/layered_costmap.hpp>

#include <memory>

namespace {

/// Custom test layer: records updateBounds/updateCosts calls and writes one lethal cell onto the master grid.
class TestLayer : public costmap_2d::Layer {
 public:
  int bounds_calls{0};
  int costs_calls{0};

  void updateBounds(double, double, double, double* min_x, double* min_y,
                    double* max_x, double* max_y) override {
    ++bounds_calls;
    *min_x = 0.0;
    *min_y = 0.0;
    *max_x = 1.0;
    *max_y = 1.0;
  }

  void updateCosts(costmap_2d::Costmap2D& master_grid, int, int, int,
                   int) override {
    ++costs_calls;
    master_grid.setCost(2, 3, costmap_2d::LETHAL_OBSTACLE);
  }
};

class ToggleLayer : public costmap_2d::Layer {
 public:
  bool mark{true};
  void updateBounds(double, double, double, double* min_x, double* min_y,
                    double* max_x, double* max_y) override {
    *min_x = 1.0;
    *min_y = 1.0;
    *max_x = 2.0;
    *max_y = 2.0;
  }
  void updateCosts(costmap_2d::Costmap2D& master, int, int, int, int) override {
    if (mark) master.setCost(1, 1, costmap_2d::LETHAL_OBSTACLE);
  }
};

}  // namespace

TEST(Costmap2DTest, resizeMapPopulatesMembers) {
  costmap_2d::Costmap2D map;
  map.resizeMap(10, 20, 0.05, 0, 0);
  EXPECT_EQ(map.getSizeInCellsX(), 10u);
  EXPECT_EQ(map.getSizeInCellsY(), 20u);
  EXPECT_DOUBLE_EQ(map.getResolution(), 0.05);
  EXPECT_NE(map.getCharMap(), nullptr);
}

TEST(Costmap2DTest, resizeMapOverwritesPreviousMembers) {
  // resizeMap parameters must not shadow the members (the size_x = size_x self-assignment bug)
  costmap_2d::Costmap2D map;
  map.resizeMap(10, 20, 0.05, 0, 0);
  map.resizeMap(30, 40, 0.1, 1.0, 2.0);
  EXPECT_EQ(map.getSizeInCellsX(), 30u);
  EXPECT_EQ(map.getSizeInCellsY(), 40u);
  EXPECT_DOUBLE_EQ(map.getResolution(), 0.1);
  EXPECT_DOUBLE_EQ(map.getOriginX(), 1.0);
  EXPECT_DOUBLE_EQ(map.getOriginY(), 2.0);
}

TEST(Costmap2DTest, setAndGetCost) {
  costmap_2d::Costmap2D map;
  map.resizeMap(10, 20, 0.05, 0, 0);
  map.setCost(5, 10, 200);
  EXPECT_EQ(map.getCost(5, 10), 200);
  EXPECT_EQ(map.getCost(0, 0), costmap_2d::FREE_SPACE);
}

TEST(Costmap2DTest, worldToMapMapToWorldRoundTrip) {
  costmap_2d::Costmap2D map;
  map.resizeMap(10, 20, 0.05, 1.0, 2.0);
  unsigned int mx = 0;
  unsigned int my = 0;
  double wx = 0.0;
  double wy = 0.0;
  map.mapToWorld(3, 7, wx, wy);
  EXPECT_TRUE(map.worldToMap(wx, wy, mx, my));
  EXPECT_EQ(mx, 3u);
  EXPECT_EQ(my, 7u);
}

TEST(Costmap2DTest, layeredCostmapInvokesLayer) {
  costmap_2d::LayeredCostmap lcm("map", false, false);
  lcm.resizeMap(10, 20, 0.05, 0, 0);
  auto layer = std::make_shared<TestLayer>();
  lcm.addLayer(layer);
  lcm.updateMap(0.0, 0.0, 0.0);
  EXPECT_GT(layer->bounds_calls, 0);
  EXPECT_GT(layer->costs_calls, 0);
  EXPECT_EQ(lcm.getCostmap()->getCost(2, 3), costmap_2d::LETHAL_OBSTACLE);
}

TEST(Costmap2DTest, layeredCostmapRebuildsDirtyMasterWindow) {
  costmap_2d::LayeredCostmap lcm("map", false, false);
  lcm.resizeMap(10, 10, 1.0, 0.0, 0.0);
  auto layer = std::make_shared<ToggleLayer>();
  lcm.addLayer(layer);
  lcm.updateMap(0.0, 0.0, 0.0);
  ASSERT_EQ(lcm.getCostmap()->getCost(1, 1),
            costmap_2d::LETHAL_OBSTACLE);
  layer->mark = false;
  lcm.updateMap(0.0, 0.0, 0.0);
  EXPECT_EQ(lcm.getCostmap()->getCost(1, 1), costmap_2d::FREE_SPACE);
}

TEST(Costmap2DTest, rollingOriginStaysCellAligned) {
  costmap_2d::Costmap2D map(10, 10, 0.5, 0.0, 0.0);
  map.updateOrigin(0.24, 0.24);
  EXPECT_DOUBLE_EQ(map.getOriginX(), 0.0);
  EXPECT_DOUBLE_EQ(map.getOriginY(), 0.0);
  map.updateOrigin(0.76, -0.26);
  EXPECT_DOUBLE_EQ(map.getOriginX(), 0.5);
  EXPECT_DOUBLE_EQ(map.getOriginY(), -0.5);
}

TEST(Costmap2DTest, footprintComputesNav2Radii) {
  costmap_2d::LayeredCostmap lcm("map", false, false);
  lcm.setFootprint({{-0.2, -0.1, 0.0}, {-0.2, 0.1, 0.0},
                    {0.2, 0.1, 0.0}, {0.2, -0.1, 0.0}});
  EXPECT_NEAR(lcm.getInscribedRadius(), 0.1, 1e-9);
  EXPECT_NEAR(lcm.getCircumscribedRadius(), std::hypot(0.2, 0.1), 1e-9);
}

TEST(Costmap2DTest, raytraceLineUsesBresenhamCells) {
  costmap_2d::Costmap2D map(10, 10, 1.0, 0.0, 0.0);
  std::vector<unsigned int> cells;
  map.raytraceLine([&](unsigned int index) { cells.push_back(index); },
                   1, 1, 5, 3);
  ASSERT_FALSE(cells.empty());
  EXPECT_EQ(cells.front(), map.getIndex(1, 1));
  EXPECT_EQ(cells.back(), map.getIndex(5, 3));
  EXPECT_EQ(cells.size(), 5u);
}

TEST(Costmap2DTest, costValuesMatchSpec) {
  EXPECT_EQ(costmap_2d::FREE_SPACE, 0);
  EXPECT_EQ(costmap_2d::INSCRIBED_INFLATED_OBSTACLE, 253);
  EXPECT_EQ(costmap_2d::LETHAL_OBSTACLE, 254);
  EXPECT_EQ(costmap_2d::NO_INFORMATION, 255);
}

TEST(Costmap2DTest, isLethalIncludes253) {
  // isLethal must classify 253 (inscribed inflated) as lethal,
  // otherwise the inflation band is misclassified as non-lethal.
  EXPECT_TRUE(costmap_2d::isLethal(253));
  EXPECT_TRUE(costmap_2d::isLethal(254));
  EXPECT_FALSE(costmap_2d::isLethal(100));
  EXPECT_FALSE(costmap_2d::isLethal(0));
}

TEST(Costmap2DTest, updateOriginShiftsContentInPlace) {
  // P6 regression: the in-place memmove shift must move content exactly
  // like the old allocate-and-copy implementation: new(x,y) = old(x+dx, y+dy)
  // where in bounds, default elsewhere, origin re-anchored to the aligned
  // new origin.
  costmap_2d::Costmap2D map(6, 4, 1.0, 0.0, 0.0);
  for (unsigned int y = 0; y < 4; ++y) {
    for (unsigned int x = 0; x < 6; ++x) {
      map.setCost(x, y, static_cast<unsigned char>((x + 1) + 10 * (y + 1)));
    }
  }

  map.updateOrigin(2.0, 1.0);  // +2 cells x, +1 cell y
  EXPECT_EQ(map.getOriginX(), 2.0);
  EXPECT_EQ(map.getOriginY(), 1.0);
  // new(0,0) = old(2,1)
  EXPECT_EQ(map.getCost(0, 0), 23);
  // new(3,0) = old(5,1)
  EXPECT_EQ(map.getCost(3, 0), 26);
  // new(4,0): old x = 6 out of bounds -> default
  EXPECT_EQ(map.getCost(4, 0), 0);
  // new(0,2) = old(2,3)
  EXPECT_EQ(map.getCost(0, 2), 43);
  // new(0,3): old y = 4 out of bounds -> default
  EXPECT_EQ(map.getCost(0, 3), 0);
  // new(5,3) = old(7,4): both out -> default
  EXPECT_EQ(map.getCost(5, 3), 0);
}

TEST(Costmap2DTest, updateOriginLargerThanMapClearsAll) {
  // A shift at least the map dimension vacates every cell.
  costmap_2d::Costmap2D map(4, 4, 1.0, 0.0, 0.0);
  map.setCost(1, 1, costmap_2d::LETHAL_OBSTACLE);
  map.updateOrigin(10.0, 0.0);
  EXPECT_EQ(map.getOriginX(), 10.0);
  for (unsigned int y = 0; y < 4; ++y) {
    for (unsigned int x = 0; x < 4; ++x) {
      EXPECT_EQ(map.getCost(x, y), costmap_2d::FREE_SPACE);
    }
  }
}

TEST(Costmap2DTest, pgmDumpWritesFlippedPixelsAndMetadata) {
  // Dump helper contract: P5 binary, image row i = map row (h-1-i) (the
  // static loader's flip, mirrored), pixel = 255 - cost, and a metadata
  // sidecar carrying resolution/origin in the static layer's map-json
  // schema.
  costmap_2d::Costmap2D grid(4, 3, 0.5, -1.0, -2.0);
  grid.setCost(0, 0, costmap_2d::LETHAL_OBSTACLE);  // map row 0, image row 2
  grid.setCost(3, 2, costmap_2d::FREE_SPACE);       // map row 2, image row 0
  grid.setCost(1, 1, 100);

  const auto base = std::filesystem::path("/tmp/opencode/pgm_dump_test/costmap");
  std::filesystem::create_directories(base.parent_path());
  std::string error;
  ASSERT_TRUE(costmap_2d::writePgmDump(grid.getCharMap(), grid.getSizeInCellsX(),
                                       grid.getSizeInCellsY(),
                                       grid.getResolution(), grid.getOriginX(),
                                       grid.getOriginY(), base, &error))
      << error;

  // Parse the PGM back (skip '#' comment lines, mirroring the static
  // layer's readToken so the dump is provably re-loadable by the framework).
  std::ifstream in(base.string() + ".pgm", std::ios::binary);
  ASSERT_TRUE(in);
  const auto readToken = [&in]() {
    std::string token;
    while (in >> token) {
      if (token.empty() || token[0] != '#') {
        return token;
      }
      in.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    }
    return token;
  };
  const std::string magic = readToken();
  const std::string width_tok = readToken();
  const std::string height_tok = readToken();
  const std::string maxval_tok = readToken();
  EXPECT_EQ(magic, "P5");
  EXPECT_EQ(std::stoul(width_tok), 4u);
  EXPECT_EQ(std::stoul(height_tok), 3u);
  EXPECT_EQ(std::stoul(maxval_tok), 255u);
  in.get();  // single whitespace after maxval
  unsigned char pixels[12];
  in.read(reinterpret_cast<char*>(pixels), 12);
  ASSERT_EQ(in.gcount(), 12);
  // Image pixel (x, i) = map cell (x, h-1-i) (flip); pixel = 255 - cost.
  EXPECT_EQ(pixels[0], static_cast<unsigned char>(255 - 0));    // image (0,0) = map (0,2) free
  EXPECT_EQ(pixels[8], static_cast<unsigned char>(255 - 254));  // image (0,2) = map (0,0) lethal
  EXPECT_EQ(pixels[5], static_cast<unsigned char>(255 - 100));  // image (1,1) = map (1,1)
  EXPECT_EQ(pixels[11], static_cast<unsigned char>(255 - 0));   // image (3,0) = map (3,2) free

  // Metadata sidecar in the static layer's map-json schema.
  std::ifstream meta_in(base.string() + ".json");
  ASSERT_TRUE(meta_in);
  std::string meta((std::istreambuf_iterator<char>(meta_in)),
                   std::istreambuf_iterator<char>());
  EXPECT_NE(meta.find("\"resolution\": 0.5"), std::string::npos);
  EXPECT_NE(meta.find("-1"), std::string::npos);
  EXPECT_NE(meta.find("-2"), std::string::npos);

  // Atomic rename: no .tmp leftovers.
  EXPECT_FALSE(std::filesystem::exists(base.string() + ".pgm.tmp"));
  std::filesystem::remove_all(base.parent_path());
}
