#include "MainMenu.h"

#include <OpenSimCreator/Documents/Model/UndoableModelActions.h>
#include <OpenSimCreator/Documents/Model/UndoableModelStatePair.h>
#include <OpenSimCreator/Documents/Simulation/Simulation.h>
#include <OpenSimCreator/Documents/Simulation/StoFileSimulation.h>
#include <OpenSimCreator/Platform/RecentFiles.h>
#include <OpenSimCreator/UI/IMainUIStateAPI.h>
#include <OpenSimCreator/UI/MeshImporter/MeshImporterTab.h>
#include <OpenSimCreator/UI/Simulation/SimulatorTab.h>
#include <OpenSimCreator/Utils/OpenSimHelpers.h>

#include <IconsFontAwesome5.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <oscar/Graphics/Scene/SceneCache.h>
#include <oscar/Platform/App.h>
#include <oscar/Platform/AppConfig.h>
#include <oscar/Platform/AppMetadata.h>
#include <oscar/Platform/Log.h>
#include <oscar/Platform/os.h>
#include <oscar/UI/ImGuiHelpers.h>
#include <oscar/UI/oscimgui.h>
#include <oscar/Utils/CStringView.h>
#include <oscar/Utils/FilesystemHelpers.h>
#include <oscar/Utils/ParentPtr.h>

#include <algorithm>
#include <array>
#include <exception>
#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <typeinfo>
#include <utility>


// public API

osc::MainMenuFileTab::MainMenuFileTab() :
    exampleOsimFiles
    {
        FindFilesWithExtensionsRecursive(
            App::resourceFilepath("models"),
            std::to_array({std::string_view{".osim"}})
        )
    }
{
    std::sort(exampleOsimFiles.begin(), exampleOsimFiles.end(), IsFilenameLexographicallyGreaterThan);
}

void osc::MainMenuFileTab::onDraw(
    ParentPtr<IMainUIStateAPI> const& api,
    UndoableModelStatePair* maybeModel)
{
    // handle hotkeys enabled by just drawing the menu
    {
        auto const& io = ImGui::GetIO();

        bool mod = IsCtrlOrSuperDown();

        if (mod && ImGui::IsKeyPressed(ImGuiKey_N))
        {
            ActionNewModel(api);
        }
        else if (mod && ImGui::IsKeyPressed(ImGuiKey_O))
        {
            ActionOpenModel(api);
        }
        else if (maybeModel && mod && io.KeyShift && ImGui::IsKeyPressed(ImGuiKey_S))
        {
            ActionSaveCurrentModelAs(*maybeModel);
        }
        else if (maybeModel && mod && ImGui::IsKeyPressed(ImGuiKey_S))
        {
            ActionSaveModel(*api, *maybeModel);
        }
        else if (maybeModel && ImGui::IsKeyPressed(ImGuiKey_F5))
        {
            ActionReloadOsimFromDisk(*maybeModel, *App::singleton<SceneCache>());
        }
    }

    // draw "save as", if necessary
    if (maybeSaveChangesPopup)
    {
        maybeSaveChangesPopup->onDraw();
    }

    if (!ui::BeginMenu("File"))
    {
        return;
    }

    if (ui::MenuItem(ICON_FA_FILE " New", "Ctrl+N"))
    {
        ActionNewModel(api);
    }

    if (ui::MenuItem(ICON_FA_FOLDER_OPEN " Open", "Ctrl+O"))
    {
        ActionOpenModel(api);
    }

    int imgui_id = 0;

    auto recentFiles = App::singleton<RecentFiles>();
    if (ui::BeginMenu(ICON_FA_FOLDER_OPEN " Open Recent", !recentFiles->empty()))
    {
        // iterate in reverse: recent files are stored oldest --> newest
        for (RecentFile const& rf : *recentFiles)
        {
            ImGui::PushID(++imgui_id);
            if (ui::MenuItem(rf.path.filename().string().c_str()))
            {
                ActionOpenModel(api, rf.path);
            }
            ImGui::PopID();
        }

        ui::EndMenu();
    }

    if (ui::BeginMenu(ICON_FA_FOLDER_OPEN " Open Example"))
    {
        for (std::filesystem::path const& ex : exampleOsimFiles)
        {
            ImGui::PushID(++imgui_id);
            if (ui::MenuItem(ex.filename().string().c_str()))
            {
                ActionOpenModel(api, ex);
            }
            ImGui::PopID();
        }

        ui::EndMenu();
    }

    ImGui::Separator();

    if (ui::MenuItem(ICON_FA_FOLDER_OPEN " Load Motion", nullptr, false, maybeModel != nullptr))
    {
        std::optional<std::filesystem::path> maybePath = PromptUserForFile("sto,mot");
        if (maybePath && maybeModel)
        {
            try
            {
                std::unique_ptr<OpenSim::Model> cpy = std::make_unique<OpenSim::Model>(maybeModel->getModel());
                InitializeModel(*cpy);
                InitializeState(*cpy);

                api->addAndSelectTab<SimulatorTab>(api, std::make_shared<Simulation>(StoFileSimulation{std::move(cpy), *maybePath, maybeModel->getFixupScaleFactor()}));
            }
            catch (std::exception const& ex)
            {
                log_error("encountered error while trying to load an STO file against the model: %s", ex.what());
            }
        }
    }

    ImGui::Separator();

    if (ui::MenuItem(ICON_FA_SAVE " Save", "Ctrl+S", false, maybeModel != nullptr))
    {
        if (maybeModel)
        {
            ActionSaveModel(*api, *maybeModel);
        }
    }

    if (ui::MenuItem(ICON_FA_SAVE " Save As", "Shift+Ctrl+S", false, maybeModel != nullptr))
    {
        if (maybeModel)
        {
            ActionSaveCurrentModelAs(*maybeModel);
        }
    }

    ImGui::Separator();

    {
        bool const modelHasBackingFile = maybeModel != nullptr && HasInputFileName(maybeModel->getModel());

        if (ui::MenuItem(ICON_FA_RECYCLE " Reload", "F5", false, modelHasBackingFile) && maybeModel)
        {
            ActionReloadOsimFromDisk(*maybeModel, *App::singleton<SceneCache>());
        }
        DrawTooltipIfItemHovered("Reload", "Attempts to reload the osim file from scratch. This can be useful if (e.g.) editing third-party files that OpenSim Creator doesn't automatically track.");

        if (ui::MenuItem(ICON_FA_CLIPBOARD " Copy .osim path to clipboard", nullptr, false, modelHasBackingFile) && maybeModel)
        {
            ActionCopyModelPathToClipboard(*maybeModel);
        }
        DrawTooltipIfItemHovered("Copy .osim path to clipboard", "Copies the absolute path to the model's .osim file into your clipboard.\n\nThis is handy if you want to (e.g.) load the osim via a script, open it from the command line in another app, etc.");

        if (ui::MenuItem(ICON_FA_FOLDER " Open .osim's parent directory", nullptr, false, modelHasBackingFile) && maybeModel)
        {
            ActionOpenOsimParentDirectory(*maybeModel);
        }

        if (ui::MenuItem(ICON_FA_LINK " Open .osim in external editor", nullptr, false, modelHasBackingFile) && maybeModel)
        {
            ActionOpenOsimInExternalEditor(*maybeModel);
        }
        DrawTooltipIfItemHovered("Open .osim in external editor", "Open the .osim file currently being edited in an external text editor. The editor that's used depends on your operating system's default for opening .osim files.");
    }

    // reload
    // copy path to clipboard
    // parent dir
    // external editor

    ImGui::Separator();

    if (ui::MenuItem(ICON_FA_MAGIC " Import Meshes"))
    {
        api->addAndSelectTab<mi::MeshImporterTab>(api);
    }
    App::upd().addFrameAnnotation("MainMenu/ImportMeshesMenuItem", GetItemRect());



    if (ui::MenuItem(ICON_FA_TIMES_CIRCLE " Quit", "Ctrl+Q"))
    {
        App::upd().requestQuit();
    }

    ui::EndMenu();
}


void osc::MainMenuAboutTab::onDraw()
{
    if (!ui::BeginMenu("About"))
    {
        return;
    }

    constexpr float menuWidth = 400;
    ui::Dummy({menuWidth, 0});

    ui::TextUnformatted("graphics");
    ui::SameLine();
    DrawHelpMarker("OSMV's global graphical settings");
    ImGui::Separator();
    ui::Dummy({0.0f, 0.5f});
    {
        ui::Columns(2);

        ui::TextUnformatted("FPS");
        ui::NextColumn();
        ui::Text("%.0f", static_cast<double>(ImGui::GetIO().Framerate));
        ui::NextColumn();

        ui::TextUnformatted("MSXAA");
        ui::SameLine();
        DrawHelpMarker("the level of MultiSample Anti-Aliasing to use. This only affects 3D renders *within* the UI, not the whole UI (panels etc. will not be affected)");
        ui::NextColumn();
        {
            AntiAliasingLevel const current = App::get().getCurrentAntiAliasingLevel();
            AntiAliasingLevel const max = App::get().getMaxAntiAliasingLevel();

            if (ImGui::BeginCombo("##msxaa", to_string(current).c_str()))
            {
                for (AntiAliasingLevel l = AntiAliasingLevel::min(); l <= max; ++l)
                {
                    bool selected = l == current;
                    if (ImGui::Selectable(to_string(l).c_str(), &selected))
                    {
                        App::upd().setCurrentAntiAliasingLevel(l);
                    }
                }
                ImGui::EndCombo();
            }
        }
        ui::NextColumn();

        ui::TextUnformatted("window");
        ui::NextColumn();

        if (ui::Button(ICON_FA_EXPAND " fullscreen"))
        {
            App::upd().makeFullscreen();
        }
        if (ui::Button(ICON_FA_EXPAND " windowed fullscreen"))
        {
            App::upd().makeWindowedFullscreen();
        }
        if (ui::Button(ICON_FA_WINDOW_RESTORE " windowed"))
        {
            App::upd().makeWindowed();
        }
        ui::NextColumn();

        ui::TextUnformatted("VSYNC");
        ui::SameLine();
        DrawHelpMarker("whether the backend uses vertical sync (VSYNC), which will cap the rendering FPS to your monitor's refresh rate");
        ui::NextColumn();

        bool enabled = App::get().isVsyncEnabled();
        if (ui::Checkbox("##vsynccheckbox", &enabled)) {
            if (enabled) {
                App::upd().enableVsync();
            } else {
                App::upd().disableVsync();
            }
        }
        ui::NextColumn();

        ui::Columns();
    }

    ui::Dummy({0.0f, 2.0f});
    ui::TextUnformatted("properties");
    ui::SameLine();
    DrawHelpMarker("general software properties: useful information for bug reporting etc.");
    ImGui::Separator();
    ui::Dummy({0.0f, 0.5f});
    {
        AppMetadata const& metadata = App::get().getMetadata();

        ui::Columns(2);

        ui::TextUnformatted("VERSION");
        ui::NextColumn();
        ui::TextUnformatted(metadata.tryGetVersionString().value_or("(not known)"));
        ui::NextColumn();

        ui::TextUnformatted("BUILD_ID");
        ui::NextColumn();
        ui::TextUnformatted(metadata.tryGetBuildID().value_or("(not known)"));
        ui::NextColumn();

        ui::TextUnformatted("GRAPHICS_VENDOR");
        ui::NextColumn();
        ui::Text(App::get().getGraphicsBackendVendorString());
        ui::NextColumn();

        ui::TextUnformatted("GRAPHICS_RENDERER");
        ui::NextColumn();
        ui::Text(App::get().getGraphicsBackendRendererString());
        ui::NextColumn();

        ui::TextUnformatted("GRAPHICS_RENDERER_VERSION");
        ui::NextColumn();
        ui::Text(App::get().getGraphicsBackendVersionString());
        ui::NextColumn();

        ui::TextUnformatted("GRAPHICS_SHADER_VERSION");
        ui::NextColumn();
        ui::Text(App::get().getGraphicsBackendShadingLanguageVersionString());
        ui::NextColumn();

        ui::Columns(1);
    }

    ui::Dummy({0.0f, 2.5f});
    ui::TextUnformatted("debugging utilities:");
    ui::SameLine();
    DrawHelpMarker("standard utilities that can help with development, debugging, etc.");
    ImGui::Separator();
    ui::Dummy({0.0f, 0.5f});
    int id = 0;
    {
        ui::Columns(2);

        ui::TextUnformatted("OSC Install Location");
        ui::SameLine();
        DrawHelpMarker("opens OSC's installation location in your OS's default file browser");
        ui::NextColumn();
        ImGui::PushID(id++);
        if (ui::Button(ICON_FA_FOLDER " open"))
        {
            OpenPathInOSDefaultApplication(App::get().getExecutableDirPath());
        }
        ImGui::PopID();
        ui::NextColumn();

        ui::TextUnformatted("User Data Dir");
        ui::SameLine();
        DrawHelpMarker("opens your OSC user data directory in your OS's default file browser");
        ui::NextColumn();
        ImGui::PushID(id++);
        if (ui::Button(ICON_FA_FOLDER " open")) {
            OpenPathInOSDefaultApplication(App::get().getUserDataDirPath());
        }
        ImGui::PopID();
        ui::NextColumn();

        ui::TextUnformatted("Debug mode");
        ui::SameLine();
        DrawHelpMarker("Toggles whether the application is in debug mode or not: enabling this can reveal more inforamtion about bugs");
        ui::NextColumn();
        {
            bool appIsInDebugMode = App::get().isInDebugMode();
            if (ui::Checkbox("##debugmodecheckbox", &appIsInDebugMode))
            {
                if (appIsInDebugMode)
                {
                    App::upd().enableDebugMode();
                }
                else
                {
                    App::upd().disableDebugMode();
                }
            }
        }

        ui::Columns();
    }

    ui::Dummy({0.0f, 2.5f});
    ui::TextUnformatted("useful links:");
    ui::SameLine();
    DrawHelpMarker("links to external sites that might be useful");
    ImGui::Separator();
    ui::Dummy({0.0f, 0.5f});
    {
        ui::Columns(2);

        ui::TextUnformatted("OpenSim Creator Documentation");
        ui::NextColumn();
        ImGui::PushID(id++);
        if (ui::Button(ICON_FA_LINK " open"))
        {
            OpenPathInOSDefaultApplication(App::get().getConfig().getHTMLDocsDir() / "index.html");
        }
        DrawTooltipBodyOnlyIfItemHovered("this will open the (locally installed) documentation in a separate browser window");
        ImGui::PopID();
        ui::NextColumn();

        if (auto repoURL = App::get().getMetadata().tryGetRepositoryURL())
        {
            ui::TextUnformatted("OpenSim Creator Repository");
            ui::NextColumn();
            ImGui::PushID(id++);
            if (ui::Button(ICON_FA_LINK " open"))
            {
                OpenPathInOSDefaultApplication(std::filesystem::path{std::string_view{*repoURL}});
            }
            DrawTooltipBodyOnlyIfItemHovered("this will open the repository homepage in a separate browser window");
            ImGui::PopID();
            ui::NextColumn();
        }

        ui::TextUnformatted("OpenSim Documentation");
        ui::NextColumn();
        ImGui::PushID(id++);
        if (ui::Button(ICON_FA_LINK " open"))
        {
            OpenPathInOSDefaultApplication("https://simtk-confluence.stanford.edu/display/OpenSim/Documentation");
        }
        DrawTooltipBodyOnlyIfItemHovered("this will open the documentation in a separate browser window");
        ImGui::PopID();
        ui::NextColumn();

        ui::Columns();
    }

    ui::EndMenu();
}
