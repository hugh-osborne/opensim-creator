#pragma once

#include <oscar/Utils/EnumHelpers.h>

namespace osc::detail
{
    // used by the texture implementation to keep track of what kind of
    // data it is storing
    enum class CPUImageFormat {
        R8,
        RG,
        RGB,
        RGBA,
        DepthStencil,
        NUM_OPTIONS,
    };

    using CPUImageFormatList = OptionList<CPUImageFormat,
        CPUImageFormat::R8,
        CPUImageFormat::RG,
        CPUImageFormat::RGB,
        CPUImageFormat::RGBA,
        CPUImageFormat::DepthStencil
    >;
}
