#pragma once

#include <OpenSimCreator/Documents/Landmarks/Landmark.hpp>

#include <oscar/Maths/Vec3.hpp>

#include <string>

namespace osc::lm
{
    struct NamedLandmark final {
        std::string name;
        Vec3 position;

        friend bool operator==(NamedLandmark const&, NamedLandmark const&) = default;
        friend bool operator==(Landmark const& lhs, NamedLandmark const& rhs)
        {
            return lhs.maybeName == rhs.name && lhs.position == rhs.position;
        }
    };
}
