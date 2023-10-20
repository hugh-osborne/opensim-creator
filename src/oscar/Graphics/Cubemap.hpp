#pragma once

#include <oscar/Graphics/CubemapFace.hpp>
#include <oscar/Graphics/TextureFormat.hpp>
#include <oscar/Utils/CopyOnUpdPtr.hpp>

#include <nonstd/span.hpp>

namespace osc
{
    class Cubemap final {
    public:
        Cubemap(int32_t width, TextureFormat format);
        Cubemap(Cubemap const&);
        Cubemap(Cubemap&&) noexcept;
        Cubemap& operator=(Cubemap const&);
        Cubemap& operator=(Cubemap&&) noexcept;
        ~Cubemap() noexcept;

        int32_t getWidth() const;
        TextureFormat getTextureFormat() const;

        // `data` must match the channel layout, bytes per channel, and
        // width*height of the cubemap, or an exception will be thrown
        void setPixelData(CubemapFace, nonstd::span<uint8_t const>);

        friend bool operator==(Cubemap const& lhs, Cubemap const& rhs) noexcept
        {
            return lhs.m_Impl == rhs.m_Impl;
        }

        friend bool operator!=(Cubemap const& lhs, Cubemap const& rhs) noexcept
        {
            return lhs.m_Impl != rhs.m_Impl;
        }
    private:
        friend class GraphicsBackend;

        class Impl;
        CopyOnUpdPtr<Impl> m_Impl;
    };
}
