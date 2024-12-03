#include <OpenSimCreator/Documents/Model/BasicModelStatePair.h>

#include <TestOpenSimCreator/TestOpenSimCreatorConfig.h>

#include <gtest/gtest.h>
#include <oscar/Graphics/Scene/SceneCache.h>
#include <OpenSimCreator/Graphics/OpenSimDecorationGenerator.h>
#include <OpenSimCreator/Platform/OpenSimCreatorApp.h>
#include <Simbody.h>

#include <filesystem>

using namespace osc;

TEST(BasicModelStatePair, WhenConstructedFromFilepathLoadsTheOsimFileAndInitializesIt)
{
    const std::filesystem::path modelPath = std::filesystem::path{OSC_RESOURCES_DIR} / "models" / "Arm26" / "arm26.osim";

    BasicModelStatePair p{modelPath};
    ASSERT_GE(p.getState().getSystemStage(), SimTK::Stage::Dynamics);
}

TEST(BasicModelStatePair, HasAFullyRealizedStateWhenCopied)
{
    BasicModelStatePair p;
    ASSERT_EQ(p.getState().getSystemStage(), SimTK::Stage::Dynamics);
    BasicModelStatePair copy{p};
    ASSERT_EQ(p.getState(). getSystemStage(), SimTK::Stage::Dynamics);
}

TEST(BasicModelStatePair, CanGenerateDecorationsFromCopy)
{
    GloballyInitOpenSim();
    GloballyAddDirectoryToOpenSimGeometrySearchPath(std::filesystem::path{OSC_RESOURCES_DIR} / "geometry");

    const std::filesystem::path modelPath = std::filesystem::path{OSC_RESOURCES_DIR} / "models" / "Arm26" / "arm26.osim";

    BasicModelStatePair p{modelPath};
    SceneCache cache;
    ASSERT_NO_THROW({ GenerateModelDecorations(cache, p); });
    BasicModelStatePair copy{p};
    ASSERT_NO_THROW({ GenerateModelDecorations(cache, copy); });
}
