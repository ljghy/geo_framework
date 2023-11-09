#ifndef GEO_VIEWER_H_
#define GEO_VIEWER_H_

#include <glad/glad.h>

#include <string>
#include <functional>

#include <GLFW/glfw3.h>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>

#include <geo/viewer/Camera.h>

class Viewer
{
public:
    struct Context
    {
        Camera &camera;
        int &windowWidth;
        int &windowHeight;

        ImGuiIO *imGuiIO;

        ImVec4 &clearColor;
    };

public:
    Viewer(const char *title, int width, int height, int samples = 1);
    ~Viewer();

    Viewer(const Viewer &) = delete;
    Viewer &operator=(const Viewer &) = delete;

    void init(std::function<void(const Context &)>);
    void run(std::function<void(const Context &)>);

private:
    std::string m_title;
    int m_windowWidth;
    int m_windowHeight;
    int m_samples;
    ImVec4 m_clearColor{0.15f, 0.15f, 0.15f, 1.f};

    GLFWwindow *m_window;

    Camera m_camera;

    Context m_context;
};

#endif
