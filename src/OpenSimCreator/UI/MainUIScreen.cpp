#include "MainUIScreen.h"

#include <OpenSimCreator/UI/LoadingTab.h>
#include <OpenSimCreator/UI/SplashTab.h>
#include <OpenSimCreator/UI/MeshImporter/MeshImporterTab.h>
#include <OpenSimCreator/UI/ModelEditor/ModelEditorTab.h>

#include <oscar/Platform/App.h>
#include <oscar/Platform/AppSettings.h>
#include <oscar/Platform/Event.h>
#include <oscar/Platform/IconCodepoints.h>
#include <oscar/Platform/Log.h>
#include <oscar/Platform/os.h>
#include <oscar/Platform/Screenshot.h>
#include <oscar/Platform/ScreenPrivate.h>
#include <oscar/Shims/Cpp23/ranges.h>
#include <oscar/UI/Events/CloseTabEvent.h>
#include <oscar/UI/Events/OpenTabEvent.h>
#include <oscar/UI/Events/ResetUIContextEvent.h>
#include <oscar/UI/oscimgui.h>
#include <oscar/UI/Tabs/ErrorTab.h>
#include <oscar/UI/Tabs/Tab.h>
#include <oscar/UI/Tabs/ScreenshotTab.h>
#include <oscar/UI/Tabs/TabRegistry.h>
#include <oscar/UI/ui_context.h>
#include <oscar/UI/Widgets/SaveChangesPopup.h>
#include <oscar/UI/Widgets/SaveChangesPopupConfig.h>
#include <oscar/Utils/Algorithms.h>
#include <oscar/Utils/Assertions.h>
#include <oscar/Utils/Conversion.h>
#include <oscar/Utils/CStringView.h>
#include <oscar/Utils/LifetimedPtr.h>
#include <oscar/Utils/Perf.h>
#include <oscar/Utils/UID.h>

#include <algorithm>
#include <functional>
#include <iterator>
#include <limits>
#include <memory>
#include <optional>
#include <ranges>
#include <sstream>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

using namespace osc;
namespace rgs = std::ranges;

namespace
{
    std::unique_ptr<Tab> LoadConfigurationDefinedTabIfNecessary(
        const AppSettings& settings,
        const TabRegistry& tabRegistry,
        Widget& parent)
    {
        if (auto maybeRequestedTab = settings.find_value("initial_tab")) {
            if (std::optional<TabRegistryEntry> maybeEntry = tabRegistry.find_by_name(to<std::string>(*maybeRequestedTab))) {
                return maybeEntry->construct_tab(parent);
            }

            log_warn("%s: cannot find a tab with this name in the tab registry: ignoring", to<std::string>(*maybeRequestedTab).c_str());
            log_warn("available tabs are:");
            for (auto&& tabRegistryEntry : tabRegistry) {
                log_warn("    %s", tabRegistryEntry.name().c_str());
            }
        }

        return nullptr;
    }
}

class osc::MainUIScreen::Impl final : public ScreenPrivate {
public:

    explicit Impl(MainUIScreen& owner) :
        ScreenPrivate{owner, nullptr}
    {}

    bool onUnhandledKeyUp(const KeyEvent& e)
    {
        if (e.matches(KeyModifier::CtrlORGui, Key::P)) {
            // `Ctrl+P` or `Super+P`: "take a screenshot"
            m_MaybeScreenshotRequest = App::upd().request_screenshot();
            return true;
        }
        if (e.matches(KeyModifier::CtrlORGui, Key::PageUp) or e.matches(KeyModifier::Gui, KeyModifier::Alt, Key::LeftArrow)) {
            // `Ctrl+PageUp` or `Super+PageUp` or `Command+Option+Left`: focus the tab to the left of the currently-active tab
            auto it = findTabByID(m_ActiveTabID);
            if (it != m_Tabs.begin() and it != m_Tabs.end()) {
                --it;  // previous
                impl_select_tab((*it)->id());
            }
            return true;
        }
        if (e.matches(KeyModifier::CtrlORGui, Key::PageDown) or e.matches(KeyModifier::Gui, KeyModifier::Alt, Key::RightArrow)) {
            // `Ctrl+PageDown` or `Super+PageDown` or `Command+Option+Right`: focus the tab to the right of the currently-active tab
            auto it = findTabByID(m_ActiveTabID);
            if (it != m_Tabs.end()-1) {
                ++it;  // next
                impl_select_tab((*it)->id());
            }
            return true;
        }
        if (e.matches(KeyModifier::CtrlORGui, Key::W) and m_Tabs.size() > 1 and m_ActiveTabID != m_Tabs.front()->id()) {
            // `Ctrl+W` or `Command+W`: close the current tab - unless it's the splash tab
            impl_close_tab(m_ActiveTabID);
            return true;
        }
        return false;
    }

    // called when an event is pumped into this screen but isn't handled by
    // either the global 2D UI context or the active tab
    bool onUnhandledEvent(Event& e)
    {
        if (e.type() == EventType::KeyUp) {
            return onUnhandledKeyUp(dynamic_cast<const KeyEvent&>(e));
        }
        return false;
    }

    UID addTab(std::unique_ptr<Tab> tab)
    {
        return impl_add_tab(std::move(tab));
    }

    void open(const std::filesystem::path& p)
    {
        addTab(std::make_unique<LoadingTab>(owner(), p));
    }

    void on_mount()
    {
        if (!std::exchange(m_HasBeenMountedBefore, true)) {
            // on first mount, place the splash tab at the front of the tabs collection
            m_Tabs.insert(m_Tabs.begin(), std::make_unique<SplashTab>(owner()));

            // if the application configuration has requested that a specific tab should be opened,
            // then try looking it up and open it
            if (auto tab = LoadConfigurationDefinedTabIfNecessary(App::settings(), *App::singleton<TabRegistry>(), owner())) {
                addTab(std::move(tab));
            }

            // focus on the rightmost tab
            if (not m_Tabs.empty()) {
                m_RequestedTab = m_Tabs.back()->id();
            }
        }

        ui::context::init(App::upd());
    }

    void on_unmount()
    {
        // unmount the active tab before unmounting this (host) screen
        if (Tab* active = getActiveTab()) {
            try {
                active->on_unmount();
            }
            catch (const std::exception& ex) {
                // - the tab is faulty in some way
                // - soak up the exception to prevent the whole application from terminating
                // - and emit the error to the log, because we have to assume that this
                //   screen is about to die (it's being unmounted)
                log_error("MainUIScreen::on_unmount: unmounting active tab threw an exception: %s", ex.what());
            }

            m_ActiveTabID = UID::empty();
        }

        ui::context::shutdown(App::upd());
    }

    bool on_event(Event& e)
    {
        if (e.type() == EventType::KeyDown or
            e.type() == EventType::KeyUp or
            e.type() == EventType::MouseButtonUp or
            e.type() == EventType::MouseMove or
            e.type() == EventType::MouseWheel) {

            // if the user just potentially changed something via a mouse/keyboard
            // interaction then the screen should be aggressively redrawn to reduce
            // and input delays

            m_ShouldRequestRedraw = true;
        }

        bool handled = false;

        if (ui::context::on_event(e)) {
            // if the 2D UI captured the event, then assume that the event will be "handled"
            // during `Tab::onDraw` (immediate-mode UI)

            m_ShouldRequestRedraw = true;
            handled = true;
        }
        else if (e.type() == EventType::Quit) {
            // if it's an application-level QUIT request, then it should be pumped into each
            // tab, while checking whether a tab wants to "block" the request (e.g. because it
            // wants to show a "do you want to save changes?" popup to the user

            bool atLeastOneTabHandledQuit = false;
            for (int i = 0; i < static_cast<int>(m_Tabs.size()); ++i) {
                try {
                    atLeastOneTabHandledQuit = m_Tabs[i]->on_event(e) || atLeastOneTabHandledQuit;
                }
                catch (const std::exception& ex) {
                    log_error("MainUIScreen::on_event: exception thrown by tab: %s", ex.what());

                    // - the tab is faulty in some way
                    // - soak up the exception to prevent the whole application from terminating
                    // - then create a new tab containing the error message, so the user can see the error
                    const UID id = addTab(std::make_unique<ErrorTab>(owner(), ex));
                    impl_select_tab(id);
                    impl_close_tab(m_Tabs[i]->id());
                }
            }

            if (not atLeastOneTabHandledQuit) {
                // if no tab handled the quit event, treat it as-if the user
                // has tried to close all tabs

                for (const auto& tab : m_Tabs) {
                    impl_close_tab(tab->id());
                }
                m_QuitRequested = true;
            }

            // handle any deletion-related side-effects (e.g. showing save prompt)
            handleDeletedTabs();

            if (not atLeastOneTabHandledQuit and (not m_MaybeSaveChangesPopup or !m_MaybeSaveChangesPopup->is_open())) {
                // - if no tab handled a quit event
                // - and the UI isn't currently showing a save prompt
                // - then it's safe to outright quit the application from this screen

                App::upd().request_quit();
            }

            handled = true;
        }
        else if (auto* addTabEv = dynamic_cast<OpenTabEvent*>(&e)) {
            if (addTabEv->has_tab()) {
                impl_select_tab(impl_add_tab(addTabEv->take_tab()));
                handled = true;
            }
        }
        else if (auto* closeTabEv = dynamic_cast<CloseTabEvent*>(&e)) {
            impl_close_tab(closeTabEv->tabid_to_close());
            handled = true;
        }
        else if (dynamic_cast<ResetUIContextEvent*>(&e)) {
            impl_reset_imgui();
        }
        else if (Tab* active = getActiveTab()) {
            // if there's an active tab, pump the event into the active tab and check
            // whether the tab handled the event

            bool activeTabHandledEvent = false;
            try {
                activeTabHandledEvent = active->on_event(e);
            }
            catch (const std::exception& ex) {
                log_error("MainUIScreen::on_event: exception thrown by tab: %s", ex.what());

                // - the tab is faulty in some way
                // - soak up the exception to prevent the whole application from terminating
                // - then create a new tab containing the error message, so the user can see the error
                UID id = addTab(std::make_unique<ErrorTab>(owner(), ex));
                impl_select_tab(id);
                impl_close_tab(m_ActiveTabID);
            }

            // the event may have triggered tab deletions
            handleDeletedTabs();

            if (activeTabHandledEvent) {
                m_ShouldRequestRedraw = true;
                handled = true;
            }
            else {
                handled = onUnhandledEvent(e);
            }
        }

        return handled;
    }

    void on_tick()
    {
        // tick all the tabs, because they may internally be polling something (e.g.
        // updating something as a simulation runs)
        for (size_t i = 0; i < m_Tabs.size(); ++i) {
            try {
                m_Tabs[i]->on_tick();
            }
            catch (const std::exception& ex) {

                log_error("MainUIScreen::on_tick: tab thrown an exception: %s", ex.what());

                // - the tab is faulty in some way
                // - soak up the exception to prevent the whole application from terminating
                // - then create a new tab containing the error message, so the user can see the error
                const UID id = addTab(std::make_unique<ErrorTab>(owner(), ex));
                impl_select_tab(id);
                impl_close_tab(m_Tabs[i]->id());
            }
        }

        // clear the flagged-to-be-deleted tabs
        handleDeletedTabs();

        // handle any currently-active user screenshot requests
        tryHandleScreenshotRequest();
    }

    void onDraw()
    {
        OSC_PERF("MainUIScreen/draw");

        {
            OSC_PERF("MainUIScreen/clear_screen");
            App::upd().clear_screen();
        }

        ui::context::on_start_new_frame(App::upd());

        {
            OSC_PERF("MainUIScreen/drawUIContent");
            drawUIContent();
        }

        if (m_ImguiWasAggressivelyReset) {
            if (m_RequestedTab == UID::empty()) {
                m_RequestedTab = m_ActiveTabID;
            }
            m_ActiveTabID = UID::empty();

            ui::context::shutdown(App::upd());
            ui::context::init(App::upd());
            App::upd().request_redraw();
            m_ImguiWasAggressivelyReset = false;

            return;
        }

        {
            OSC_PERF("MainUIScreen/ui::context::render()");
            ui::context::render();
        }

        if (std::exchange(m_ShouldRequestRedraw, false)) {
            App::upd().request_redraw();
        }
    }

    void drawTabSpecificMenu()
    {
        OSC_PERF("MainUIScreen/drawTabSpecificMenu");

        if (ui::begin_main_viewport_top_bar("##TabSpecificMenuBar")) {
            if (ui::begin_menu_bar()) {
                if (Tab* active = getActiveTab()) {
                    try {
                        active->on_draw_main_menu();
                    }
                    catch (const std::exception& ex) {
                        log_error("MainUIScreen::drawTabSpecificMenu: tab thrown an exception: %s", ex.what());

                        // - the tab is faulty in some way
                        // - soak up the exception to prevent the whole application from terminating
                        // - then create a new tab containing the error message, so the user can see the error
                        const UID id = addTab(std::make_unique<ErrorTab>(owner(), ex));
                        impl_select_tab(id);
                        impl_close_tab(m_ActiveTabID);
                    }

                    if (m_ImguiWasAggressivelyReset) {
                        return;  // must return here to prevent the ImGui end_panel calls from erroring
                    }
                }
                ui::end_menu_bar();
            }
            ui::end_panel();
            handleDeletedTabs();
        }
    }

    void drawTabBar()
    {
        OSC_PERF("MainUIScreen/drawTabBar");

        ui::push_style_var(ui::StyleVar::FramePadding, ui::get_style_frame_padding() + 2.0f);
        ui::push_style_var(ui::StyleVar::ItemInnerSpacing, Vec2{5.0f, 0.0f});
        ui::push_style_var(ui::StyleVar::TabRounding, 10.0f);
        ui::push_style_var(ui::StyleVar::FrameRounding, 10.0f);
        if (ui::begin_main_viewport_top_bar("##TabBarViewport")) {
            if (ui::begin_menu_bar()) {
                if (ui::begin_tab_bar("##TabBar")) {
                    for (size_t i = 0; i < m_Tabs.size(); ++i) {
                        ui::TabItemFlags flags = ui::TabItemFlag::NoReorder;

                        if (i == 0) {
                            flags |= ui::TabItemFlag::NoCloseButton;  // splash screen
                        }

                        if (m_Tabs[i]->is_unsaved()) {
                            flags |= ui::TabItemFlag::UnsavedDocument;
                        }

                        if (m_Tabs[i]->id() == m_RequestedTab) {
                            flags |= ui::TabItemFlag::SetSelected;;
                        }

                        if (m_Tabs[i]->id() == m_ActiveTabID and m_Tabs[i]->name() != m_ActiveTabNameLastFrame) {
                            flags |= ui::TabItemFlag::SetSelected;
                            m_ActiveTabNameLastFrame = m_Tabs[i]->name();
                        }

                        ui::push_id(m_Tabs[i].get());
                        bool active = true;

                        if (ui::begin_tab_item(m_Tabs[i]->name(), &active, flags)) {
                            if (m_Tabs[i]->id() != m_ActiveTabID) {
                                if (Tab* activeTab = getActiveTab()) {
                                    activeTab->on_unmount();
                                }
                                m_Tabs[i]->on_mount();
                            }

                            m_ActiveTabID = m_Tabs[i]->id();
                            m_ActiveTabNameLastFrame = m_Tabs[i]->name();

                            if (m_RequestedTab == m_ActiveTabID) {
                                m_RequestedTab = UID::empty();
                            }

                            if (m_ImguiWasAggressivelyReset) {
                                return;
                            }

                            ui::end_tab_item();
                        }

                        ui::pop_id();
                        if (not active and i != 0)  {  // can't close the splash tab
                            impl_close_tab(m_Tabs[i]->id());
                        }
                    }

                    // adding buttons to tab bars: https://github.com/ocornut/imgui/issues/3291
                    ui::draw_tab_item_button(OSC_ICON_PLUS);

                    if (ui::begin_popup_context_menu("popup", ui::PopupFlag::MouseButtonLeft)) {
                        drawAddNewTabMenu();
                        ui::end_popup();
                    }

                    ui::end_tab_bar();
                }
                ui::end_menu_bar();
            }

            ui::end_panel();
            handleDeletedTabs();
        }
        ui::pop_style_var(4);
    }

    void drawUIContent()
    {
        drawTabSpecificMenu();

        if (m_ImguiWasAggressivelyReset) {
            return;
        }

        drawTabBar();

        if (m_ImguiWasAggressivelyReset) {
            return;
        }

        // draw the active tab (if any)
        if (Tab* active = getActiveTab()) {
            try {
                OSC_PERF("MainUIScreen/drawActiveTab");
                active->on_draw();
            }
            catch (const std::exception& ex) {
                log_error("MainUIScreen::drawUIConent: tab thrown an exception: %s", ex.what());

                // - the tab is faulty in some way
                // - soak up the exception to prevent the whole application from terminating
                // - then create a new tab containing the error message, so the user can see the error
                // - and indicate that the UI was aggressively reset, because the drawcall may have thrown midway
                //   through rendering the 2D UI
                const UID id = addTab(std::make_unique<ErrorTab>(owner(), ex));
                impl_select_tab(id);
                impl_close_tab(m_ActiveTabID);
                impl_reset_imgui();
            }

            handleDeletedTabs();
        }

        if (m_ImguiWasAggressivelyReset) {
            return;
        }

        if (m_MaybeSaveChangesPopup) {
            m_MaybeSaveChangesPopup->on_draw();
        }
    }

    void drawAddNewTabMenu()
    {
        if (ui::draw_menu_item(OSC_ICON_EDIT " Editor")) {
            impl_select_tab(addTab(std::make_unique<ModelEditorTab>(owner())));
        }

        if (ui::draw_menu_item(OSC_ICON_CUBE " Mesh Importer")) {
            impl_select_tab(addTab(std::make_unique<mi::MeshImporterTab>(owner())));
        }

        const std::shared_ptr<const TabRegistry> tabRegistry = App::singleton<TabRegistry>();
        if (not tabRegistry->empty()) {
            if (ui::begin_menu("Experimental Tabs")) {
                for (auto&& tabRegistryEntry : *tabRegistry) {
                    if (ui::draw_menu_item(tabRegistryEntry.name())) {
                        impl_select_tab(addTab(tabRegistryEntry.construct_tab(owner())));
                    }
                }
                ui::end_menu();
            }
        }
    }

    std::vector<std::unique_ptr<Tab>>::iterator findTabByID(UID id)
    {
        return rgs::find(m_Tabs, id, [](const auto& ptr) { return ptr->id(); });
    }

    Tab* getTabByID(UID id)
    {
        auto it = findTabByID(id);
        return it != m_Tabs.end() ? it->get() : nullptr;
    }

    Tab* getActiveTab()
    {
        return getTabByID(m_ActiveTabID);
    }

    Tab* getRequestedTab()
    {
        return getTabByID(m_RequestedTab);
    }

    UID impl_add_tab(std::unique_ptr<Tab> tab)
    {
        return m_Tabs.emplace_back(std::move(tab))->id();
    }

    void impl_select_tab(UID id)
    {
        m_RequestedTab = id;
    }

    void impl_close_tab(UID id)
    {
        m_DeletedTabs.insert(id);
    }

    // called by the "save changes?" popup when user opts to save changes
    bool onUserSelectedSaveChangesInSavePrompt()
    {
        bool savingFailedSomewhere = false;
        for (UID id : m_DeletedTabs) {
            if (Tab* tab = getTabByID(id); tab && tab->is_unsaved()) {
                savingFailedSomewhere = not tab->try_save() or savingFailedSomewhere;
            }
        }

        if (not savingFailedSomewhere) {
            nukeDeletedTabs();
            if (m_QuitRequested) {
                App::upd().request_quit();
            }
            return true;
        }
        else {
            return false;
        }
    }

    // called by the "save changes?" popup when user opts to not save changes
    bool onUserSelectedDoNotSaveChangesInSavePrompt()
    {
        nukeDeletedTabs();
        if (m_QuitRequested) {
            App::upd().request_quit();
        }
        return true;
    }

    // called by the "save changes?" popup when user clicks "cancel"
    bool onUserCancelledOutOfSavePrompt()
    {
        m_DeletedTabs.clear();
        m_QuitRequested = false;
        return true;
    }

    void nukeDeletedTabs()
    {
        int lowestDeletedTab = std::numeric_limits<int>::max();
        for (UID id : m_DeletedTabs) {
            auto it = rgs::find(m_Tabs, id, [](const auto& ptr) { return ptr->id(); });
            if (it != m_Tabs.end()) {
                if (id == m_ActiveTabID) {
                    (*it)->on_unmount();
                    m_ActiveTabID = UID::empty();
                    lowestDeletedTab = min(lowestDeletedTab, static_cast<int>(std::distance(m_Tabs.begin(), it)));
                }
                m_Tabs.erase(it);
            }
        }
        m_DeletedTabs.clear();

        // coerce active tab, if it has become stale due to a deletion
        if (not getRequestedTab() and not getActiveTab() and not m_Tabs.empty()) {
            // focus the tab just to the left of the closed one
            if (1 <= lowestDeletedTab and lowestDeletedTab <= static_cast<int>(m_Tabs.size())) {
                m_RequestedTab = m_Tabs[lowestDeletedTab - 1]->id();
            }
            else {
                m_RequestedTab = m_Tabs.front()->id();
            }
        }
    }

    void handleDeletedTabs()
    {
        // tabs aren't immediately deleted, because they may hold onto unsaved changes
        //
        // this top-level screen has to handle the unsaved changes. This is because it would be
        // annoying, from a UX PoV, to have each tab individually prompt the user. It is preferable
        // to have all the "do you want to save changes?" things in one prompt


        // if any of the to-be-deleted tabs have unsaved changes, then open a save changes dialog
        // that prompts the user to decide on how to handle it
        //
        // don't delete the tabs yet, because the user can always cancel out of the operation

        std::vector<Tab*> tabsWithUnsavedChanges;

        for (UID id : m_DeletedTabs) {
            if (Tab* t = getTabByID(id)) {
                if (t->is_unsaved()) {
                    tabsWithUnsavedChanges.push_back(t);
                }
            }
        }

        if (!tabsWithUnsavedChanges.empty()) {
            std::stringstream ss;
            if (tabsWithUnsavedChanges.size() > 1) {
                ss << tabsWithUnsavedChanges.size() << " tabs have unsaved changes:\n";
            }
            else {
                ss << "A tab has unsaved changes:\n";
            }

            for (Tab* t : tabsWithUnsavedChanges) {
                ss << "\n  - " << t->name();
            }
            ss << "\n\n";

            // open the popup
            SaveChangesPopupConfig cfg{
                "Save Changes?",
                [this]() { return onUserSelectedSaveChangesInSavePrompt(); },
                [this]() { return onUserSelectedDoNotSaveChangesInSavePrompt(); },
                [this]() { return onUserCancelledOutOfSavePrompt(); },
                ss.str(),
            };
            m_MaybeSaveChangesPopup.emplace(cfg);
            m_MaybeSaveChangesPopup->open();
        }
        else {
            // just nuke all the tabs
            nukeDeletedTabs();
        }
    }

    void impl_reset_imgui()
    {
        m_ImguiWasAggressivelyReset = true;
    }

    void tryHandleScreenshotRequest()
    {
        if (!m_MaybeScreenshotRequest.valid()) {
            return;  // probably empty/errored
        }

        if (m_MaybeScreenshotRequest.valid() and m_MaybeScreenshotRequest.wait_for(std::chrono::seconds{0}) == std::future_status::ready) {
            const UID tabID = addTab(std::make_unique<ScreenshotTab>(owner(), m_MaybeScreenshotRequest.get()));
            impl_select_tab(tabID);
        }
    }

private:
    OSC_OWNER_GETTERS(MainUIScreen);

    // lifetime of this object
    SharedLifetimeBlock m_Lifetime;

    // set the first time `onMount` is called
    bool m_HasBeenMountedBefore = false;

    // user-visible UI tabs
    std::vector<std::unique_ptr<Tab>> m_Tabs;

    // set of tabs that should be deleted once control returns to this screen
    std::unordered_set<UID> m_DeletedTabs;

    // currently-active UI tab
    UID m_ActiveTabID = UID::empty();

    // cached version of active tab name - used to ensure the UI can re-focus a renamed tab
    std::string m_ActiveTabNameLastFrame;

    // a tab that should become active next frame
    UID m_RequestedTab = UID::empty();

    // a popup that is shown when a tab, or the whole screen, is requested to close
    //
    // effectively, shows the "do you want to save changes?" popup
    std::optional<SaveChangesPopup> m_MaybeSaveChangesPopup;

    // true if the screen is midway through trying to quit
    bool m_QuitRequested = false;

    // true if the screen should request a redraw from the application
    bool m_ShouldRequestRedraw = false;

    // true if the UI context was aggressively reset by a tab (and, therefore, this screen should reset the UI)
    bool m_ImguiWasAggressivelyReset = false;

    // `valid` if the user has requested a screenshot (that hasn't yet been handled)
    std::future<Screenshot> m_MaybeScreenshotRequest;
};


osc::MainUIScreen::MainUIScreen() :
    Screen{std::make_unique<Impl>(*this)}
{}
osc::MainUIScreen::~MainUIScreen() noexcept = default;
void osc::MainUIScreen::open(const std::filesystem::path& path) { private_data().open(path); }
void osc::MainUIScreen::impl_on_mount() { private_data().on_mount(); }
void osc::MainUIScreen::impl_on_unmount() { private_data().on_unmount(); }
bool osc::MainUIScreen::impl_on_event(Event& e) { return private_data().on_event(e); }
void osc::MainUIScreen::impl_on_tick() { private_data().on_tick(); }
void osc::MainUIScreen::impl_on_draw() { private_data().onDraw(); }
