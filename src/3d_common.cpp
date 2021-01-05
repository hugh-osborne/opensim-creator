#include "3d_common.hpp"

#include <cmath>
#include <stdexcept>

constexpr float pi_f = static_cast<float>(M_PI);

static glm::vec3 normals(glm::vec3 const& p1, glm::vec3 const& p2, glm::vec3 const& p3) {
    //https://stackoverflow.com/questions/19350792/calculate-normal-of-a-single-triangle-in-3d-space/23709352
    glm::vec3 a{p2.x - p1.x, p2.y - p1.y, p2.z - p1.z};
    glm::vec3 b{p3.x - p1.x, p3.y - p1.y, p3.z - p1.z};

    float x = a.y * b.z - a.z * b.y;
    float y = a.z * b.x - a.x * b.z;
    float z = a.x * b.y - a.y * b.x;

    return glm::vec3{x,y,z};
}


// Returns triangles of a "unit" (radius = 1.0f, origin = 0,0,0) sphere
void osmv::unit_sphere_triangles(std::vector<osmv::Untextured_vert>& out) {
    out.clear();

    // this is a shitty alg that produces a shitty UV sphere. I don't have
    // enough time to implement something better, like an isosphere, or
    // something like a patched sphere:
    //
    // https://www.iquilezles.org/www/articles/patchedsphere/patchedsphere.htm
    //
    // This one is adapted from:
    //    http://www.songho.ca/opengl/gl_sphere.html#example_cubesphere

    size_t sectors = 12;
    size_t stacks = 12;

    // polar coords, with [0, 0, -1] pointing towards the screen with polar
    // coords theta = 0, phi = 0. The coordinate [0, 1, 0] is theta = (any)
    // phi = PI/2. The coordinate [1, 0, 0] is theta = PI/2, phi = 0
    std::vector<osmv::Untextured_vert> points;

    float theta_step = 2.0f*pi_f / sectors;
    float phi_step = pi_f / stacks;

    for (size_t stack = 0; stack <= stacks; ++stack) {
        float phi = pi_f/2.0f - static_cast<float>(stack)*phi_step;
        float y = sin(phi);

        for (unsigned sector = 0; sector <= sectors; ++sector) {
            float theta = sector * theta_step;
            float x = sin(theta) * cos(phi);
            float z = -cos(theta) * cos(phi);
            glm::vec3 pos{ x, y, z };
            glm::vec3 normal{ pos };
            points.push_back({ pos, normal });
        }
    }

    // the points are not triangles. They are *points of a triangle*, so the
    // points must be triangulated

    for (size_t stack = 0; stack < stacks; ++stack) {
        size_t k1 = stack * (sectors + 1);
        size_t k2 = k1 + sectors + 1;

        for (size_t sector = 0; sector < sectors; ++sector, ++k1, ++k2) {
            // 2 triangles per sector - excluding the first and last stacks
            // (which contain one triangle, at the poles)
            osmv::Untextured_vert p1 = points.at(k1);
            osmv::Untextured_vert p2 = points.at(k2);
            osmv::Untextured_vert p1_plus1 = points.at(k1+1u);
            osmv::Untextured_vert p2_plus1 = points.at(k2+1u);

            if (stack != 0) {
                out.push_back(p1);
                out.push_back(p1_plus1);
                out.push_back(p2);
            }

            if (stack != (stacks-1)) {
                out.push_back(p1_plus1);
                out.push_back(p2_plus1);
                out.push_back(p2);
            }
        }
    }
}

// Returns triangles for a "unit" cylinder with `num_sides` sides.
//
// Here, "unit" means:
//
// - radius == 1.0f
// - top == [0.0f, 0.0f, -1.0f]
// - bottom == [0.0f, 0.0f, +1.0f]
// - (so the height is 2.0f, not 1.0f)
void osmv::unit_cylinder_triangles(size_t num_sides, std::vector<osmv::Untextured_vert>& out) {
    // TODO: this is dumb because a cylinder can be EBO-ed quite easily, which
    //       would reduce the amount of vertices needed
    if (num_sides < 3) {
        throw std::runtime_error{"cannot create a cylinder with fewer than 3 sides"};
    }

    out.clear();
    out.reserve(4*num_sides);  // side quad, top triangle, bottom triangle

    float step_angle = (2.0f*pi_f)/num_sides;
    float top_z = -1.0f;
    float bottom_z = +1.0f;

    // top
    {
        glm::vec3 p1{0.0f, 0.0f, top_z};  // middle
        for (auto i = 0U; i < num_sides; ++i) {
            float theta_start = i*step_angle;
            float theta_end = (i+1)*step_angle;
            glm::vec3 p2(sin(theta_start), cos(theta_start), top_z);
            glm::vec3 p3(sin(theta_end), cos(theta_end), top_z);
            glm::vec3 normal = normals(p1, p2, p3);

            out.push_back({p1, normal});
            out.push_back({p2, normal});
            out.push_back({p3, normal});
        }
    }

    // bottom
    {
        glm::vec3 p1{0.0f, 0.0f, -1.0f};  // middle
        for (auto i = 0U; i < num_sides; ++i) {
            float theta_start = i*step_angle;
            float theta_end = (i+1)*step_angle;

            glm::vec3 p2(sin(theta_start), cos(theta_start), bottom_z);
            glm::vec3 p3(sin(theta_end), cos(theta_end), bottom_z);
            glm::vec3 normal = normals(p1, p2, p3);

            out.push_back({p1, normal});
            out.push_back({p2, normal});
            out.push_back({p3, normal});
        }
    }

    // sides
    {
        float norm_start = step_angle/2.0f;
        for (auto i = 0U; i < num_sides; ++i) {
            float theta_start = i * step_angle;
            float theta_end = theta_start + step_angle;
            float norm_theta = theta_start + norm_start;

            glm::vec3 p1(sin(theta_start), cos(theta_start), top_z);
            glm::vec3 p2(sin(theta_end), cos(theta_end), top_z);
            glm::vec3 p3(sin(theta_start), cos(theta_start), bottom_z);
            glm::vec3 p4(sin(theta_start), cos(theta_start), bottom_z);

            // triangle 1
            glm::vec3 n1 = normals(p1, p2, p3);
            out.push_back({p1, n1});
            out.push_back({p2, n1});
            out.push_back({p3, n1});

            // triangle 2
            glm::vec3 n2 = normals(p3, p4, p2);
            out.push_back({p3, n2});
            out.push_back({p4, n2});
            out.push_back({p2, n2});
        }
    }
}

// Returns triangles for a "simbody" cylinder with `num_sides` sides.
//
// This matches simbody-visualizer.cpp's definition of a cylinder, which
// is:
//
// radius
//     1.0f
// top
//     [0.0f, 1.0f, 0.0f]
// bottom
//     [0.0f, -1.0f, 0.0f]
//
// see simbody-visualizer.cpp::makeCylinder for my source material
void osmv::simbody_cylinder_triangles(size_t num_sides, std::vector<osmv::Untextured_vert>& out) {
    // TODO: this is dumb because a cylinder can be EBO-ed quite easily, which
    //       would reduce the amount of vertices needed
    if (num_sides < 3) {
        throw std::runtime_error{"cannot create a cylinder with fewer than 3 sides"};
    }

    out.clear();
    out.reserve(2*num_sides + 2*num_sides);

    float step_angle = (2.0f*pi_f)/num_sides;
    float top_y = +1.0f;
    float bottom_y = -1.0f;

    // top
    {
        glm::vec3 normal = {0.0f, 1.0f, 0.0f};
        osmv::Untextured_vert top_middle{ {0.0f, top_y, 0.0f}, normal };
        for (auto i = 0U; i < num_sides; ++i) {
            float theta_start = i*step_angle;
            float theta_end = (i+1)*step_angle;

            // note: these are wound CCW for backface culling
            out.push_back(top_middle);
            out.push_back({
                glm::vec3(cos(theta_end), top_y, sin(theta_end)),
                normal
            });
            out.push_back({
                glm::vec3(cos(theta_start), top_y, sin(theta_start)),
                normal
            });
        }
    }

    // bottom
    {
        glm::vec3 bottom_normal{0.0f, -1.0f, 0.0f};
        osmv::Untextured_vert top_middle{ {0.0f, bottom_y, 0.0f}, bottom_normal };
        for (auto i = 0U; i < num_sides; ++i) {
            float theta_start = i*step_angle;
            float theta_end = (i+1)*step_angle;

            // note: these are wound CCW for backface culling
            out.push_back(top_middle);
            out.push_back({
                glm::vec3(cos(theta_start), bottom_y, sin(theta_start)),
                bottom_normal,
            });
            out.push_back({
                glm::vec3(cos(theta_end), bottom_y, sin(theta_end)),
                bottom_normal,
            });
        }
    }

    // sides
    {
        float norm_start = step_angle/2.0f;
        for (auto i = 0U; i < num_sides; ++i) {
            float theta_start = i * step_angle;
            float theta_end = theta_start + step_angle;
            float norm_theta = theta_start + norm_start;

            glm::vec3 normal(cos(norm_theta), 0.0f, sin(norm_theta));
            glm::vec3 top1(cos(theta_start), top_y, sin(theta_start));
            glm::vec3 top2(cos(theta_end), top_y, sin(theta_end));

            glm::vec3 bottom1 = top1;
            bottom1.y = bottom_y;
            glm::vec3 bottom2 = top2;
            bottom2.y = bottom_y;

            // draw 2 triangles per quad cylinder side
            //
            // note: these are wound CCW for backface culling
            out.push_back({top1, normal});
            out.push_back({top2, normal});
            out.push_back({bottom1, normal});

            out.push_back({bottom2, normal});
            out.push_back({bottom1, normal});
            out.push_back({top2, normal});
        }
    }
}

gl::Texture_2d osmv::generate_chequered_floor_texture() {
    struct Rgb { unsigned char r, g, b; };
    constexpr size_t w = 512;
    constexpr size_t h = 512;
    constexpr Rgb on_color = {0xfd, 0xfd, 0xfd};
    constexpr Rgb off_color = {0xeb, 0xeb, 0xeb};

    std::array<Rgb, w*h> pixels;
    for (size_t row = 0; row < h; ++row) {
        size_t row_start = row * w;
        bool y_on = (row/32) % 2 == 0;
        for (size_t col = 0; col < w; ++col) {
            bool x_on = (col/32) % 2 == 0;
            pixels[row_start + col] = y_on xor x_on ? on_color : off_color;
        }
    }

    gl::Texture_2d rv = gl::GenTexture2d();
    gl::BindTexture(rv.type, rv);
    glTexImage2D(rv.type, 0, GL_RGB, w, h, 0, GL_RGB, GL_UNSIGNED_BYTE, pixels.data());
    glGenerateMipmap(rv.type);
    return rv;
}
