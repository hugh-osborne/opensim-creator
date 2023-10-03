#include <oscar/Graphics/TextureChannelFormat.hpp>

#include <gtest/gtest.h>
#include <oscar/Utils/EnumHelpers.hpp>

TEST(TextureChannelFormat, NumBytesPerChannelReturnsExpectedValues)
{
    static_assert(osc::NumOptions<osc::TextureChannelFormat>() == 2);

    ASSERT_EQ(osc::NumBytesPerChannel(osc::TextureChannelFormat::Uint8), 1);
    ASSERT_EQ(osc::NumBytesPerChannel(osc::TextureChannelFormat::Float32), 4);
}
