#include "mesh.hpp"
#include "pick_rows.hpp"
#include "kernel_poly6.hpp"

#include "spatial/2d/neighborhood_spatial_hashing.hpp"

#include <Eigen/Geometry>
#include <iostream>

namespace GooBalls {
namespace d2 {
namespace Physics {

using namespace Spatial;

Mesh::Mesh() : 
    m_vertices_position_global(new Coordinates()), 
    m_triangles(new TriangleList()){
    // empty
}

Mesh::Mesh(std::shared_ptr<Mesh::Coordinates> verts, std::shared_ptr<TriangleList> tris) : 
    m_vertices_position_global(verts),
    m_triangles(tris)
{
    m_vertices_position_local = *verts;
}

const Mesh::Coordinates& Mesh::vertices_position_global() const {
    return *m_vertices_position_global;
}

Mesh::Coordinates& Mesh::vertices_position_global() {
    return *m_vertices_position_global;
}

const Mesh::Coordinates& Mesh::vertices_position_local() const {
    return m_vertices_position_local;
}
Mesh::Coordinates& Mesh::vertices_position_local() {
    return m_vertices_position_local;
}

const TriangleList& Mesh::triangles() const {
    return *m_triangles;
}
TriangleList& Mesh::triangles() {
    return *m_triangles;
}

const Coordinates2d& Mesh::particles_position_local() const {
    return m_particles_position_local;
}

const Coordinates2d& Mesh::particles_velocity() const {
    return m_particles_velocity;
}

const Coordinates1d& Mesh::particles_volume() const {
    return m_particles_volume;
}

RotationMatrix Mesh::rotation() const {
    FloatPrecision radianAngle = body->GetAngle();
    // eigen convention: counter-clockwise rotation in radians
    // box2d convention: not clear, only hint is in chapter 8.6 Revolute Joint of the manual -> test
    // TODO: test rotation direction of Box2d
    Eigen::Rotation2D<FloatPrecision> rot(radianAngle);
    RotationMatrix rotM = rot.toRotationMatrix();
    return rotM;
}

TranslationVector Mesh::translation() const {
    TranslationVector dx;
    dx[0] = body->GetPosition().x;
    dx[1] = body->GetPosition().y;
    return dx;
}

TranslationVector Mesh::linear_velocity() const {
    TranslationVector v;
    v[0] = body->GetLinearVelocity().x;
    v[1] = body->GetLinearVelocity().y;
    return v;
}

RotationMatrix Mesh::angular_velocity() const {
    // box2d: radians/second
    RotationMatrix r;
    FloatPrecision v = body->GetAngularVelocity();
    Eigen::Rotation2D<FloatPrecision> rot(v);
    RotationMatrix rotM = rot.toRotationMatrix();
    // TODO: it is not clear, if rotM needs to be transposed here -> test
    return rotM;
}

/// takes the rigid body transformation from the box2D object and applies it locally
void Mesh::update_from_rigid_body() {
    const auto& loc = this->vertices_position_local();
    auto& glob = this->vertices_position_global();
    RotationMatrix rotM = rotation();
    TranslationVector dx = translation();
    rotM.transpose();
    glob = (loc * rotM).rowwise() + dx;
    //m_particles_global = (m_particles_local * rotM).rowwise() + dy;
    // speed of particle
    if(m_particles_position_local.rows() > 0){
        m_particles_velocity.resize(m_particles_position_local.rows(), Eigen::NoChange);
        m_particles_velocity.col(0) = -m_particles_position_local.col(1);
        m_particles_velocity.col(1) = m_particles_position_local.col(0);
        m_particles_velocity *= body->GetAngularVelocity();
        m_particles_velocity = m_particles_velocity.rowwise() + linear_velocity();
    }
}

void Mesh::prepare(FloatPrecision h){
    update_from_rigid_body();
    create_particles(h);
    compute_particles_volume(h);

}
void Mesh::create_particles(FloatPrecision h) {
    // most basic particle generation algorithm:
    // for each triangle
    // - add three particles for each corner
    // - look at all edges and sample each edge n additional times
    const auto& tris = triangles();
    const auto& verts = vertices_position_local();
    /*
    std::cout << "pts: \n";
    for(int i = 0; i < verts.rows(); ++i){
        std::cout << verts(i,0) << " " << verts(i, 1) << std::endl;
    }
    std::cout << "body: " << body->GetPosition().x << ", " << body->GetPosition().y << std::endl;
    */
    int totalPoints = 0;
    int inserted = 0;
    auto& pts = m_particles_position_local;
    int sampling = 4;

    for(int t = 0; t < tris.rows(); ++t){
        for(int v = 0; v < 3; v++){
            int vn = (v+1)%3; // next vertex
            // process edge
            auto diffV = verts.row(tris(t, vn)) - verts.row(tris(t, v));
            int n = std::ceil(diffV.norm()/h*sampling)+1;
            totalPoints += 1;
            totalPoints += n;
            pts.conservativeResize(totalPoints, Eigen::NoChange);
            // insert vertex
            pts.row(inserted++) = verts.row(tris(t, v));
            TranslationVector diff = (diffV)/double(n+1);
            for(int ni = 0; ni < n; ni++){
                pts.row(inserted++) = (ni+1) * diff + verts.row(tris(t, v));
            }
        }
    }
}

void Mesh::compute_particles_volume(FloatPrecision h) {
    // occupied volume according to Versatile Rigid-Fluid Coupling for Incompressible SPH, Akinci et. al
    // V_bi = 1/sum_k W_ik, W: is a kernel
    m_particles_volume.resize(m_particles_position_local.rows(), Eigen::NoChange);
    NeighborhoodSpatialHashing neighbor;
    Poly6 kernel;
    kernel.setH(h);
    neighbor.inRange(m_particles_position_local, h);
    const auto& indexes = neighbor.indexes();
    for(size_t i = 0; i < indexes.size(); ++i){
        Coordinates2d jpos;
        Coordinates1d W;
        pickRows(m_particles_position_local, indexes[i], jpos);
        auto xij = -(jpos.rowwise() - m_particles_position_local.row(i));
        kernel.compute(xij, &W, nullptr, nullptr);
        m_particles_volume[i] = 1.0/W.sum();
    }
}


} // Physics
} // d2
} // GooBalls
