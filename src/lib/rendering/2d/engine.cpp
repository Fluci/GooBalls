#include "engine.hpp"

namespace GooBalls {
namespace d2 {
namespace Render {

Engine::Engine() {
}

void Engine::init() {
    m_particleShader.initFromFiles(
        "particle_shader",
        "shaders/particle.vert",
        "shaders/particle.frag",
        "shaders/particle.geom"
    );
}

Engine::~Engine() {
    m_particleShader.free();
}

void Engine::render(const Scene& scene) {
    glEnable(GL_DEPTH_TEST);
    m_particleShader.bind();

    for(const auto& fluid : scene.fluids){
        m_particleShader.uploadAttrib("center", fluid->particles_position().transpose());
        m_particleShader.uploadAttrib("radius", fluid->particles_radius().transpose());
        m_particleShader.uploadAttrib("color", fluid->particles_color().transpose());
        m_particleShader.drawArray(GL_POINTS, 0, fluid->particles_position().rows());
    }
    bool showBoundaryParticles = true;
    if(showBoundaryParticles){
        for(const auto& fluid : scene.fluids){
            auto& bound = fluid->boundary_position();
            //bound.setRandom();
            fluid->boundary_radius().resize(bound.rows(), Eigen::NoChange);
            fluid->boundary_radius().array() = 0.03;
            fluid->boundary_color().resize(bound.rows(), Eigen::NoChange);
            fluid->boundary_color().array() = 0.5;
        }
        for(const auto& fluid : scene.fluids){
            m_particleShader.uploadAttrib("center", fluid->boundary_position().transpose());
            m_particleShader.uploadAttrib("radius", fluid->boundary_radius().transpose());
            m_particleShader.uploadAttrib("color", fluid->boundary_color().transpose());
            m_particleShader.drawArray(GL_POINTS, 0, fluid->boundary_position().rows());
        }
    }

    for(const auto& mesh : scene.meshes){
        // mesh->render();
    }
    glDisable(GL_DEPTH_TEST);
}
} // Render
} // d2
} // GooBalls
