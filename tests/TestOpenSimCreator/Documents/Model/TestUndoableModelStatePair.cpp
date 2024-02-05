#include <OpenSimCreator/Documents/Model/UndoableModelStatePair.hpp>

#include <TestOpenSimCreator/TestOpenSimCreatorConfig.hpp>

#include <OpenSim/Common/Component.h>
#include <OpenSimCreator/Graphics/OpenSimDecorationGenerator.hpp>
#include <OpenSimCreator/Graphics/OpenSimDecorationOptions.hpp>
#include <OpenSimCreator/Platform/OpenSimCreatorApp.hpp>
#include <OpenSimCreator/Utils/OpenSimHelpers.hpp>
#include <gtest/gtest.h>
#include <oscar/Formats/DAE.hpp>
#include <oscar/Scene/SceneCache.hpp>
#include <oscar/Scene/SceneDecoration.hpp>
#include <oscar/Utils/NullOStream.hpp>

#include <filesystem>
#include <functional>
#include <sstream>

using osc::DAEMetadata;
using osc::GetAbsolutePathString;
using osc::GlobalInitOpenSim;
using osc::NullOStream;
using osc::OpenSimDecorationOptions;
using osc::SceneCache;
using osc::SceneDecoration;
using osc::UndoableModelStatePair;

TEST(UndoableModelStatePair, CanLoadAndRenderAllUserFacingExampleFiles)
{
    GlobalInitOpenSim();

    SceneCache meshCache;

    // turn as many decoration options on as possible, so that the code gets tested
    // against them (#661)
    OpenSimDecorationOptions decorationOpts;
    decorationOpts.setShouldShowAnatomicalMuscleLineOfActionForInsertion(true);
    decorationOpts.setShouldShowAnatomicalMuscleLineOfActionForOrigin(true);
    decorationOpts.setShouldShowEffectiveMuscleLineOfActionForInsertion(true);
    decorationOpts.setShouldShowEffectiveMuscleLineOfActionForOrigin(true);
    decorationOpts.setShouldShowCentersOfMass(true);
    decorationOpts.setShouldShowScapulo(true);
    decorationOpts.setShouldShowPointToPointSprings(true);

    std::filesystem::path const examplesDir = std::filesystem::path{OSC_TESTING_SOURCE_DIR} / "resources" / "models";
    ASSERT_TRUE(std::filesystem::exists(examplesDir) && std::filesystem::is_directory(examplesDir));

    size_t nExamplesTested = 0;
    for (std::filesystem::directory_entry const& e : std::filesystem::recursive_directory_iterator{examplesDir})
    {
        if (e.is_regular_file() && e.path().extension() == ".osim")
        {
            // all files are loadable
            UndoableModelStatePair p{e.path()};

            // and all can be used to generate 3D scenes
            std::vector<SceneDecoration> decorations;
            GenerateModelDecorations(
                meshCache,
                p.getModel(),
                p.getState(),
                decorationOpts,
                1.0f,  // 1:1 scaling
                [&decorations](OpenSim::Component const& component, SceneDecoration&& dec)
                {
                    dec.id = GetAbsolutePathString(component);
                    decorations.push_back(std::move(dec));
                }
            );

            // and decorations are generated
            ASSERT_FALSE(decorations.empty());

            // and all decorations can be exported to a DAE format
            NullOStream stream;
            DAEMetadata const metadata{TESTOPENSIMCREATOR_APPNAME_STRING, TESTOPENSIMCREATOR_APPNAME_STRING};
            WriteDecorationsAsDAE(stream, decorations, metadata);

            // and content is actually written to the DAE stream
            ASSERT_TRUE(stream.wasWrittenTo());

            ++nExamplesTested;
        }
    }
    ASSERT_GT(nExamplesTested, 0);  // sanity check: remove this if you want <10 examples
}
