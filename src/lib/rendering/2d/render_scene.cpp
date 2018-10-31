#include "render_scene.hpp"

namespace GooBalls {
namespace d2 {

RenderScene::RenderScene(std::shared_ptr<Coordinates> ptr) : m_particles_position(ptr) {
	// empty
}

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

RenderScene::Radii& RenderScene::particles_radius() {
	return m_particles_radius;
}

const RenderScene::Radii& RenderScene::particles_radius() const {
	return m_particles_radius;
}

} // d2
} // GooBalls
