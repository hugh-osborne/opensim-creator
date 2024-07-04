#include <oscar/oscar.h>

#ifdef EMSCRIPTEN
    #include <emscripten.h>
    #include <emscripten/html5.h>
#endif

using namespace osc;

namespace
{
    class HelloTriangleScreen final : public IScreen {
    public:
        HelloTriangleScreen()
        {
            const Vec3 viewer_position = {3.0f, 0.0f, 0.0f};
            camera_.set_position(viewer_position);
            camera_.set_direction({-1.0f, 0.0f, 0.0f});
            const Color color = Color::red();
            material_.set_ambient_color(0.2f * color);
            material_.set_diffuse_color(0.5f * color);
            material_.set_specular_color(0.5f * color);
            material_.set_viewer_position(viewer_position);
        }
    private:
         void impl_on_draw() override
         {
             const auto seconds_since_startup = App::get().frame_delta_since_startup().count();
             const auto transform = identity<Transform>().with_rotation(angle_axis(Radians{seconds_since_startup}, Vec3{0.0f, 1.0f, 0.0f}));
             graphics::draw(mesh_, transform, material_, camera_);
             camera_.render_to_screen();
         }

        TorusKnotGeometry mesh_;
        MeshPhongMaterial material_;
        Camera camera_;
    };
}


int main(int, char**)
{
#ifdef EMSCRIPTEN
    osc::App* app = new osc::App{};
    app->setup_main_loop<HelloTriangleScreen>();
    emscripten_request_animation_frame_loop([](double, void* ptr) -> int
    {
        const bool keep_running = static_cast<App*>(ptr)->do_main_loop_step();
        return keep_running ? EM_TRUE : EM_FALSE;
    }, app);
    return 0;
#else
    osc::App app;
    app.setup_main_loop<HelloTriangleScreen>();
    ScopeGuard guard{[&app](){ app.teardown_main_loop(); }};
    while (app.do_main_loop_step()) {
    }
    return 0;
#endif
}
