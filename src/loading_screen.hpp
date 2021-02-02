#pragma once

#include "screen.hpp"

#include <SDL_events.h>
#include <filesystem>
#include <memory>

namespace osmv {
    struct Loading_screen_impl;
}

namespace osmv {
    // loading screen: screen shown when UI has just booted and is loading (e.g.) an osim file
    class Loading_screen final : public Screen {
        std::unique_ptr<Loading_screen_impl> impl;

    public:
        Loading_screen(std::filesystem::path);
        Loading_screen(Loading_screen const&) = delete;
        Loading_screen& operator=(Loading_screen const&) = delete;
        Loading_screen& operator=(Loading_screen&&) = delete;
        ~Loading_screen() noexcept override;

        Event_response on_event(SDL_Event const&) override;
        void tick() override;
        void draw() override;
    };
}
