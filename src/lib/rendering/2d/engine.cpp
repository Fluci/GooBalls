#include "engine.hpp"

namespace GooBalls {
namespace d2 {
namespace Render {

Engine::Engine() {
    m_particleShader.initFromFiles(
        "particle_shader",
        "shaders/particle.vert",
        "shaders/particle.frag"
    );

    nanogui::MatrixXu indices(3, 12); /* Draw a cube */
    indices.col( 0) << 0, 1, 3;
    indices.col( 1) << 3, 2, 1;
    indices.col( 2) << 3, 2, 6;
    indices.col( 3) << 6, 7, 3;
    indices.col( 4) << 7, 6, 5;
    indices.col( 5) << 5, 4, 7;
    indices.col( 6) << 4, 5, 1;
    indices.col( 7) << 1, 0, 4;
    indices.col( 8) << 4, 0, 3;
    indices.col( 9) << 3, 7, 4;
    indices.col(10) << 5, 6, 2;
    indices.col(11) << 2, 1, 5;

    Eigen::MatrixXf positions(3, 8);
    positions.col(0) << -1,  1,  1;
    positions.col(1) << -1,  1, -1;
    positions.col(2) <<  1,  1, -1;
    positions.col(3) <<  1,  1,  1;
    positions.col(4) << -1, -1,  1;
    positions.col(5) << -1, -1, -1;
    positions.col(6) <<  1, -1, -1;
    positions.col(7) <<  1, -1,  1;

    Eigen::MatrixXf colors(3, 12);
    colors.col( 0) << 1, 0, 0;
    colors.col( 1) << 0, 1, 0;
    colors.col( 2) << 1, 1, 0;
    colors.col( 3) << 0, 0, 1;
    colors.col( 4) << 1, 0, 1;
    colors.col( 5) << 0, 1, 1;
    colors.col( 6) << 1, 1, 1;
    colors.col( 7) << 0.5, 0.5, 0.5;
    colors.col( 8) << 1, 0, 0.5;
    colors.col( 9) << 1, 0.5, 0;
    colors.col(10) << 0.5, 1, 0;
    colors.col(11) << 0.5, 1, 0.5;

    m_particleShader.bind();
    m_particleShader.uploadIndices(indices);

    m_particleShader.uploadAttrib("position", positions);
    m_particleShader.uploadAttrib("color", colors);
}

Engine::~Engine() {
    m_particleShader.free();
}

void Engine::render(const Scene& scene) {
    m_particleShader.bind();

    Eigen::Matrix4f mvp;
    mvp.setIdentity();
    float fTime = (float)glfwGetTime();
    Eigen::Vector3f mRotation(0.25f, 0.5f, 0.33f);

    mvp.topLeftCorner<3,3>() = Eigen::Matrix3f(Eigen::AngleAxisf(mRotation[0]*fTime, Eigen::Vector3f::UnitX()) *
                                               Eigen::AngleAxisf(mRotation[1]*fTime,  Eigen::Vector3f::UnitY()) *
                                               Eigen::AngleAxisf(mRotation[2]*fTime, Eigen::Vector3f::UnitZ())) * 0.25f;

    m_particleShader.setUniform("modelViewProj", mvp);

    glEnable(GL_DEPTH_TEST);
    /* Draw 12 triangles starting at index 0 */
    m_particleShader.drawIndexed(GL_TRIANGLES, 0, 12);
    glDisable(GL_DEPTH_TEST);
    
    // Mock:
    for(const auto& fluid : scene.fluids){
        fluid->render();
    }   
    for(const auto& mesh : scene.meshes){
        mesh->render();
    }   
}
} // Render
} // d2
} // GooBalls
