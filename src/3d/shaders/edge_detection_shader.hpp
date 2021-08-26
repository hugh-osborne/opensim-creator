#pragma once

#include "src/3d/gl.hpp"

namespace osc {
    // A specialized edge-detection shader for rim highlighting
    struct Edge_detection_shader final {
        gl::Program p;

        static constexpr gl::Attribute_vec3 aPos = 0;
        static constexpr gl::Attribute_vec2 aTexCoord = 1;

        gl::Uniform_mat4 uModelMat;
        gl::Uniform_mat4 uViewMat;
        gl::Uniform_mat4 uProjMat;
        gl::Uniform_sampler2d uSampler0;
        gl::Uniform_vec4 uRimRgba;
        gl::Uniform_float uRimThickness;

        Edge_detection_shader();
    };
}
