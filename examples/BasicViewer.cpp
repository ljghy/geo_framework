#include <iostream>

#include <imgui.h>
#include <opengl_framework/Common.h>
#include <geo/mesh/Mesh.h>
#include <geo/io/IO.h>
#include <geo/viewer/Viewer.h>
#include <opengl_framework/IndexBufferObject.h>

class MeshRenderer
{
public:
    MeshRenderer(Mesh &mesh)
    {
        const char *vertSrc = R"(
#version 330 core

layout(location = 0) in vec3 aPos;
layout(location = 1) in vec3 aNorm;

uniform mat4 u_model;
uniform mat4 u_MVP;

out vec3 norm;

void main() {
    mat3 normMat = mat3(transpose(inverse(u_model)));
    norm = normalize(normMat * aNorm);
    gl_Position = u_MVP * vec4(aPos, 1.0);
}
        )";

        const char *fragSrc = R"(
#version 330 core

in vec3 norm;

out vec4 fragColor;

void main() {
    vec3 n = normalize(norm) * 0.5 + 0.5;
    fragColor = vec4(n, 1.0);
}
        )";

        m_shader.create(vertSrc, fragSrc);

        mesh.require(Mesh::VertexNormals);

        std::vector<float> vertices(mesh.nV() * 3);
        for (size_t i = 0; i < mesh.nV(); ++i)
            for (size_t j = 0; j < 3; ++j)
                vertices[i * 3 + j] = mesh.vertices[i].position(j);

        std::vector<float> normals(mesh.nV() * 3);
        for (size_t i = 0; i < mesh.nV(); ++i)
            for (size_t j = 0; j < 3; ++j)
                normals[i * 3 + j] = mesh.vertices[i].normal(j);

        m_VBO.create(nullptr, (3 + 3) * sizeof(float) * mesh.nV(),
                     GL_STATIC_DRAW);
        m_VBO.uploadSubData(vertices.data(), 0, 3 * sizeof(float) * mesh.nV());
        m_VBO.uploadSubData(normals.data(), 3 * sizeof(float) * mesh.nV(),
                            3 * sizeof(float) * mesh.nV());

        ogl::VertexBufferLayout layout(true);
        layout.push(GL_FLOAT, 3, vertices.size());
        layout.push(GL_FLOAT, 3, normals.size());

        m_VAO.create();
        m_VAO.bind();
        m_VAO.addBuffer(m_VBO, layout);

        std::vector<unsigned int> indices(mesh.nF() * 3);
        for (size_t i = 0; i < mesh.nF(); ++i)
            for (size_t j = 0; j < 3; ++j)
                indices[i * 3 + j] = mesh.faces[i].indices(j);
        m_IBO.create(indices.data(), indices.size() * sizeof(unsigned int));
    }

    void render(const Camera &camera)
    {
        m_VBO.bind();
        m_VAO.bind();
        m_IBO.bind();

        glEnable(GL_DEPTH_TEST);
        glDepthFunc(GL_LESS);
        glEnable(GL_CULL_FACE);
        glCullFace(GL_BACK);

        m_shader.use();
        auto model = glm::scale(glm::mat4(1.f), glm::vec3(1.f));
        auto view = camera.view();
        auto proj = camera.proj();
        auto mvp = proj * view * model;

        m_shader.setUniformMatrix4fv("u_model", &model[0][0]);
        m_shader.setUniformMatrix4fv("u_MVP", &mvp[0][0]);

        glDrawElements(GL_TRIANGLES, m_IBO.getCount(), GL_UNSIGNED_INT,
                       nullptr);
    }

private:
    ogl::VertexBufferObject m_VBO;
    ogl::VertexArrayObject m_VAO;
    ogl::IndexBufferObject m_IBO;
    ogl::Shader m_shader;
};

class PointRenderer
{
public:
    PointRenderer(Mesh &mesh)
    {
        const char *vertSrc = R"(
#version 330 core
layout(location = 0) in vec3 aPos;
uniform mat4 u_MVP;
void main() {
    gl_Position = u_MVP * vec4(aPos, 1.0);
    gl_Position.z -= 0.001;
}
        )";

        const char *fragSrc = R"(
#version 330 core
out vec4 fragColor;
void main() {
    fragColor = vec4(1.0, 0.0, 0.0, 1.0);
}
        )";

        m_shader.create(vertSrc, fragSrc);

        std::vector<float> vertices(mesh.nV() * 3);
        for (size_t i = 0; i < mesh.nV(); ++i)
            for (size_t j = 0; j < 3; ++j)
                vertices[i * 3 + j] = mesh.vertices[i].position(j);

        m_VBO.create(nullptr, 3 * sizeof(float) * mesh.nV(), GL_STATIC_DRAW);
        m_VBO.uploadSubData(vertices.data(), 0, 3 * sizeof(float) * mesh.nV());

        m_nV = mesh.nV();
    }

    void render(const Camera &camera)
    {
        m_VBO.bind();

        glEnable(GL_DEPTH_TEST);
        glDepthFunc(GL_LESS);

        m_shader.use();
        auto model = glm::scale(glm::mat4(1.f), glm::vec3(1.f));
        auto view = camera.view();
        auto proj = camera.proj();
        auto mvp = proj * view * model;

        m_shader.setUniformMatrix4fv("u_MVP", &mvp[0][0]);

        glPointSize(5.f);
        glDrawArrays(GL_POINTS, 0, m_nV);
    }

private:
    ogl::VertexBufferObject m_VBO;
    ogl::Shader m_shader;
    size_t m_nV;
};

void updateCamera(const Viewer::Context &ctx)
{
    if (ctx.windowWidth == 0 || ctx.windowHeight == 0)
        return;
    ctx.camera.resize(ctx.windowWidth, ctx.windowHeight);

    auto &io = *ctx.imGuiIO;

    if (!io.WantCaptureMouse)
    {
        constexpr float sensitivity = glm::radians(0.1f);
        ImVec2 mouseDelta = io.MouseDelta;

        if (ImGui::IsMouseDragging(1))
        {
            ctx.camera.rotateAround(glm::vec3(0.f), -mouseDelta.x * sensitivity,
                                    -mouseDelta.y * sensitivity);
        }

        if (ImGui::IsMouseDragging(2))
        {
            float vel = 0.01f * glm::tan(ctx.camera.fovy * 0.5f);
            ctx.camera.position -= vel * mouseDelta.x * ctx.camera.right;
            ctx.camera.position += vel * mouseDelta.y * ctx.camera.up;
        }

        ctx.camera.fovy -= glm::radians(5.f) * io.MouseWheel;
        ctx.camera.fovy =
            glm::clamp(ctx.camera.fovy, glm::radians(1.f), glm::radians(120.f));
    }

    if (!io.WantCaptureKeyboard)
    {
        float vel = 5.f;
        if (ImGui::IsKeyDown(ImGuiKey_W))
            ctx.camera.position += vel * ctx.camera.front * io.DeltaTime;
        if (ImGui::IsKeyDown(ImGuiKey_S))
            ctx.camera.position -= vel * ctx.camera.front * io.DeltaTime;
        if (ImGui::IsKeyDown(ImGuiKey_A))
            ctx.camera.position -= vel * ctx.camera.right * io.DeltaTime;
        if (ImGui::IsKeyDown(ImGuiKey_D))
            ctx.camera.position += vel * ctx.camera.right * io.DeltaTime;
    }
}

int main(int argc, char **argv)
{
    if (argc != 2)
    {
        std::cerr << "Usage: " << argv[0] << " <mesh.vtk>" << std::endl;
        return 1;
    }

    Viewer viewer("Viewer", 1600, 900);

    Mesh mesh;
    loadMeshFromVtk(argv[1], mesh);
    MeshRenderer renderer(mesh);
    PointRenderer pointRenderer(mesh);

    viewer.init(
        [](const Viewer::Context &ctx)
        {
            ctx.camera.position = glm::vec3(0.f, 0.f, 5.f);
            ctx.camera.rotate(glm::radians(180.f), 0.f);
        });

    viewer.run(
        [&](const Viewer::Context &ctx)
        {
            updateCamera(ctx);
            renderer.render(ctx.camera);
            pointRenderer.render(ctx.camera);
            ImGui::ColorEdit4("clear color", &ctx.clearColor.x);
        });
}
