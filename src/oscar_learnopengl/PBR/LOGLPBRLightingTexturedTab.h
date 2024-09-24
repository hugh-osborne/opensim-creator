#pragma once

#include <oscar/UI/Tabs/ITab.h>
#include <oscar/Utils/CStringView.h>
#include <oscar/Utils/UID.h>

#include <memory>

namespace osc { template<typename T> class ParentPtr; }
namespace osc { class ITabHost; }

namespace osc
{
    class LOGLPBRLightingTexturedTab final : public ITab {
    public:
        static CStringView id();

        explicit LOGLPBRLightingTexturedTab(const ParentPtr<ITabHost>&);
        LOGLPBRLightingTexturedTab(const LOGLPBRLightingTexturedTab&) = delete;
        LOGLPBRLightingTexturedTab(LOGLPBRLightingTexturedTab&&) noexcept;
        LOGLPBRLightingTexturedTab& operator=(const LOGLPBRLightingTexturedTab&) = delete;
        LOGLPBRLightingTexturedTab& operator=(LOGLPBRLightingTexturedTab&&) noexcept;
        ~LOGLPBRLightingTexturedTab() noexcept override;

    private:
        UID impl_get_id() const final;
        CStringView impl_get_name() const final;
        void impl_on_mount() final;
        void impl_on_unmount() final;
        bool impl_on_event(Event&) final;
        void impl_on_draw() final;

        class Impl;
        std::unique_ptr<Impl> impl_;
    };
}
