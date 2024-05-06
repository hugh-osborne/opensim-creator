#include "StandardPopup.h"

#include <oscar/Maths/MathHelpers.h>
#include <oscar/Maths/Rect.h>
#include <oscar/Maths/Vec2.h>
#include <oscar/UI/oscimgui.h>

#include <optional>
#include <string_view>

osc::StandardPopup::StandardPopup(std::string_view popup_name) :
    StandardPopup{popup_name, {512.0f, 0.0f}, ImGuiWindowFlags_AlwaysAutoResize}
{}

osc::StandardPopup::StandardPopup(
    std::string_view popup_name,
    Vec2 dimensions,
    ImGuiWindowFlags popup_flags) :

    popup_name_{popup_name},
    dimensions_{dimensions},
    maybe_position_{std::nullopt},
    popup_flags_{popup_flags},
    should_open_{false},
    should_close_{false},
    just_opened_{false},
    is_open_{false},
    is_modal_{true}
{}

bool osc::StandardPopup::impl_is_open() const
{
    return should_open_ or is_open_;
}

void osc::StandardPopup::impl_open()
{
    should_open_ = true;
    should_close_ = false;
}

void osc::StandardPopup::impl_close()
{
    should_close_ = true;
    should_open_ = false;
}

bool osc::StandardPopup::impl_begin_popup()
{
    if (should_open_) {
        ui::OpenPopup(popup_name_);
        should_open_ = false;
        should_close_ = false;
        just_opened_ = true;
    }

    if (is_modal_) {
        // if specified, set the position of the modal upon appearing
        //
        // else, position the modal in the center of the viewport
        if (maybe_position_) {
            ui::SetNextWindowPos(
                static_cast<Vec2>(*maybe_position_),
                ImGuiCond_Appearing
            );
        }
        else {
            ui::SetNextWindowPos(
                ui::GetMainViewport()->GetCenter(),
                ImGuiCond_Appearing,
                Vec2{0.5f, 0.5f}
            );
        }

        // if the modal doesn't auto-resize each frame, then set the size of
        // modal only upon appearing
        //
        // else, set the position every frame, because the __nonzero__ dimensions
        // will stretch out the modal accordingly
        if (not (popup_flags_ & ImGuiWindowFlags_AlwaysAutoResize)) {
            ui::SetNextWindowSize(
                Vec2{dimensions_},
                ImGuiCond_Appearing
            );
        }
        else {
            ui::SetNextWindowSize(
                Vec2{dimensions_}
            );
        }

        // try to begin the modal window
        impl_before_imgui_begin_popup();
        const bool opened = ui::BeginPopupModal(popup_name_, nullptr, popup_flags_);
        impl_after_imgui_begin_popup();

        if (not opened) {
            // modal not showing
            is_open_ = false;
            return false;
        }
    }
    else
    {
        // if specified, set the position of the modal upon appearing
        //
        // else, do nothing - the popup's position will be determined
        // by other means (unlike a modal, which usually takes control
        // of the screen and, therefore, should proabably be centered
        // in it)
        if (maybe_position_) {
            ui::SetNextWindowPos(
                static_cast<Vec2>(*maybe_position_),
                ImGuiCond_Appearing
            );
        }

        // try to begin the popup window
        impl_before_imgui_begin_popup();
        const bool opened = ui::BeginPopup(popup_name_, popup_flags_);
        impl_after_imgui_begin_popup();

        // try to show popup
        if (not opened) {
            // popup not showing
            is_open_ = false;
            return false;
        }
    }

    is_open_ = true;
    return true;
}

void osc::StandardPopup::impl_on_draw()
{
    if (should_close_) {
        impl_on_close();
        ui::CloseCurrentPopup();
        should_close_ = false;
        should_open_ = false;
        just_opened_ = false;
        return;
    }

    impl_draw_content();
}

void osc::StandardPopup::impl_end_popup()
{
    ui::EndPopup();
    just_opened_ = false;
}

bool osc::StandardPopup::is_popup_opened_this_frame() const
{
    return just_opened_;
}

void osc::StandardPopup::request_close()
{
    should_close_ = true;
    should_open_ = false;
}

bool osc::StandardPopup::is_modal() const
{
    return is_modal_;
}

void osc::StandardPopup::set_modal(bool v)
{
    is_modal_ = v;
}

void osc::StandardPopup::set_rect(const Rect& rect)
{
    maybe_position_ = rect.p1;
    dimensions_ = dimensions_of(rect);
}

void osc::StandardPopup::set_dimensions(Vec2 d)
{
    dimensions_ = d;
}

void osc::StandardPopup::set_position(std::optional<Vec2> p)
{
    maybe_position_ = p;
}
