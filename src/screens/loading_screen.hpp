#pragma once

#include "src/screens/screen.hpp"

#include <SDL_events.h>
#include <filesystem>

namespace osc {
    // shows a basic loading message while an .osim file loads
    class Loading_screen final : public Screen {
        struct Impl;
        Impl* impl;

    public:
        Loading_screen(std::filesystem::path);
        Loading_screen(Loading_screen const&) = delete;
        Loading_screen& operator=(Loading_screen const&) = delete;
        Loading_screen& operator=(Loading_screen&&) = delete;
        ~Loading_screen() noexcept override;

        bool on_event(SDL_Event const&) override;
        void tick() override;
        void draw() override;
    };
}
