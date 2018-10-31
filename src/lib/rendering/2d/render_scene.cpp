#include "render_scene.hpp"

namespace GooBalls {
namespace d2 {

RenderScene::RenderScene(std::shared_ptr<Coordinates> ptr) : m_particles_position(ptr) {
	// empty
}

// Fluid

const RenderScene::Coordinates& RenderScene::particles_position() const {
	assert(m_particles_position.get() != nullptr);
	return *m_particles_position;
}

RenderScene::Coordinates& RenderScene::particles_position() {
	assert(m_particles_position.get() != nullptr);
	return *m_particles_position;
}

const RenderScene::Colors& RenderScene::particles_color() const {
	return m_particles_color;
}

RenderScene::Colors& RenderScene::particles_color() {
	return m_particles_color;
}

const RenderScene::Radii& RenderScene::particles_radius() const {
	return m_particles_radius;
}

RenderScene::Radii& RenderScene::particles_radius() {
	return m_particles_radius;
}

// Mesh: Vertex

// Mesh: Triangles

// Background
const Background& RenderScene::background() const {
	return m_background;
}
Background& RenderScene::background() {
	return m_background;
}

// Camera
const Camera& RenderScene::camera() const {
	return m_camera;
}

Camera& RenderScene::camera() {
	return m_camera;
}



} // d2
} // GooBalls
