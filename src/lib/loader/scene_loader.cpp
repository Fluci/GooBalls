#include "scene_loader.hpp"
#include "spatial/2d/convex_hull.hpp"
#include <boost/log/trivial.hpp>
#include <boost/filesystem.hpp>
#include "rendering/2d/disk_fluid.hpp"
#include <Eigen/Geometry>

/**
 * Loads a scene from a file <scenename>.json located in the scenes-folder.
 * Inserts the objects from the scene directly into the Box2D physics scene and the RenderScene.
 * 
 * The scene coinsists of objects made of triangles and faces. Each object, as well as the scene,
 * can have names. These are currently only here for debug reasons.
 * 
 * The scene object has a scale and a translation. The scaling is applied around (0, 0) before the
 * whole scene is translated by the translation vector.
 *
 * Currently all scene objects are just enclosed in an axis-aligned bounding box going from (min_x, min_y)
 * to (max_x, max_y) where these are the minimal, resp. maximal x and y coordinates of all vertices in the
 * object's list of vertices.
 * Later on, we shall implement convex hulls instead.
 *
 * The triangles of each object are currently colored grey (RGB = (0.5, 0.5, 0.5)).
 * Further color specifications may be implemented if deemed necessary.
 *
 * Authored: vramp, 19.11.18 (vramp@student.ethz.ch)
 */


namespace GooBalls {

using namespace d2;

bool showDebugOutput = true;

bool SceneLoader::loadScene(Physics::Scene& physScene, Render::Scene& renderScene, std::string pathStr) {

    std::ifstream ifs(pathStr);

    if (!ifs.is_open()) {
        BOOST_LOG_TRIVIAL(error) << "Error on opening scene file: " + pathStr;
        return false;
    }
    auto path = boost::filesystem::canonical(pathStr);
    Json::Reader reader;
    Json::Value scene;
    reader.parse(ifs, scene);
    BOOST_LOG_TRIVIAL(debug) << "Loading Scene: " << scene["name"].asString() << std::endl;
    if(scene.isMember("includes")){
        const auto& includes = scene["includes"];
        for(unsigned int i = 0; i < includes.size(); ++i){
            auto includePath = boost::filesystem::path(includes[i].asString());
            if(includePath.is_absolute()){
                loadScene(physScene, renderScene, includePath.string());
            } else {
                auto thisDir = path.parent_path();
                loadScene(physScene, renderScene, (thisDir / includePath).string());
            }
        }
    }
    scale = 1.0;
    transl_x = 0.0;
    transl_y = 0.0;
    if(scene.isMember("scale")){
        scale = scene["scale"].asDouble();
    }
    if(scene.isMember("translation")){
        transl_x = scene["translation"][0].asDouble();
        transl_y = scene["translation"][1].asDouble();
    }

    // iterate through the scene's objects
    //int objectCount = obj["objects"]["count"].asInt();
    for (unsigned int objIndex = 0; objIndex < scene["objects"].size(); objIndex++) {
        const auto& obj = scene["objects"][objIndex];
        readObject(physScene, renderScene, obj);
    }

    if(scene.isMember("fluid")){
        readFluid(physScene, renderScene, scene["fluid"]);
    } else {
        setEmptyFluid(physScene, renderScene);
    }
    if(scene.isMember("room")){
        readRoom(physScene, renderScene, scene["room"]);
    }
    if(scene.isMember("gravity")){
        physScene.gravity[0] = scene["gravity"][0].asDouble();
        physScene.gravity[1] = scene["gravity"][1].asDouble();
    }
    return true;
}

void SceneLoader::readRoom(Physics::Scene& physScene, Render::Scene&, const Json::Value& room) const {
    if(room.isMember("floor")){
        physScene.room.floor = room["floor"].asDouble()*scale + transl_y;
    }
    if(room.isMember("ceiling")){
        physScene.room.ceiling = room["ceiling"].asDouble()*scale + transl_y;
    }
    if(room.isMember("leftWall")){
        physScene.room.left_wall = room["leftWall"].asDouble()*scale + transl_x;
    }
    if(room.isMember("rightWall")){
        physScene.room.right_wall = room["rightWall"].asDouble()*scale + transl_x;
    }
}

Coordinates2d SceneLoader::createGrid(int px, int py, Float gridSpacing) const {
    Coordinates2d coords;
    coords.setZero(px*py, 2);
    for(int i = 0; i < px; ++i){
        for(int j = 0; j < py; ++j){
            coords(i*py + j, 0) = i;
            coords(i*py + j, 1) = j;
        }
    }
    coords = gridSpacing * coords;
    return coords;
}

Coordinates2d SceneLoader::readGrid(const Json::Value& grid) const {
    assert(grid["type"].asString() == "grid");
    int px = grid["gridWidth"].asInt();
    int py = grid["gridHeight"].asInt();
    Float distance = grid["gridSpacing"].asDouble();
    return createGrid(px, py, distance);
}

void SceneLoader::setEmptyFluid(Physics::Scene& physScene, Render::Scene& renderScene) const {
    auto particleCoordinates = std::make_shared<Coordinates2d>();
    auto boundaryCoords = std::make_shared<Coordinates2d>();
    auto fluidPhys = std::make_unique<Physics::Fluid>(particleCoordinates, boundaryCoords);
    auto fluidRender = std::make_unique<Render::DiskFluid>(particleCoordinates, boundaryCoords);

    physScene.fluid = std::move(fluidPhys);
    renderScene.fluids.resize(1);
    renderScene.fluids[0] = std::move(fluidRender);
}

void SceneLoader::readSubFluid(Physics::Scene& physScene, Render::Scene& renderScene, const Json::Value& fluid) const {
    auto& fluidPhys = *physScene.fluid;
    auto& fluidRender = *renderScene.fluids[0];
    auto& particles = fluidPhys.particles_position();
    const auto& parts = fluid["particles"];
    Coordinates2d newCoords;
    if(parts["type"].asString() == "grid"){
        newCoords = readGrid(parts);

    } else if(parts["type"].asString() == "coordinates") {
        newCoords = readCoordinates(parts["positions"]);
    }
    
    Float pos_x = 0;
    Float pos_y = 0;
    if(fluid.isMember("position")){
        pos_x = fluid["position"][0].asDouble();
        pos_y = fluid["position"][1].asDouble();
    }
    newCoords.array() *= scale;
    newCoords.col(0).array() += transl_x + pos_x*scale;
    newCoords.col(1).array() += transl_y + pos_y*scale;
    const int ps = particles.rows();
    const int pn = newCoords.rows();
    const int pa = ps+pn;
    particles.conservativeResize(pa, Eigen::NoChange);
    particles.block(ps, 0, pn, 2) = newCoords;
    // physics data
    fluidPhys.particles_velocity().conservativeResize(pa, Eigen::NoChange);
    auto vs = fluidPhys.particles_velocity().block(ps, 0, pn, 2);
    vs.setZero();
    if(fluid.isMember("initialVelocity")){
        const auto& vinit = fluid["initialVelocity"];
        if(vinit[0].isDouble()){
            vs.col(0).array() = scale * vinit[0].asDouble();
            vs.col(1).array() = scale * vinit[1].asDouble();
        } else {
            Coordinates2d v0 = readCoordinates(fluid["initialVelocity"]);
            vs.array() = scale*v0;
        }
    }
    // TODO: compute the mass from rho*V = m
    // TODO: check all known constraint equations (CLF)
    // TOOD: make sanity checks
    fluidPhys.particles_mass().conservativeResize(pa, Eigen::NoChange);
    auto ms = fluidPhys.particles_mass().block(ps, 0, pn, 1);
    ms.setOnes();
    if(fluid.isMember("mass"))
        ms.array() = fluid["mass"].asDouble();
    assert(ms.minCoeff() > 0.0);
    fluidPhys.particles_velocity_correction().conservativeResize(pa, Eigen::NoChange);
    auto cs = fluidPhys.particles_velocity_correction().block(ps, 0, pn, 1);
    if(fluid.isMember("velocityCorrectionCoefficient"))
        cs.array() = fluid["velocityCorrectionCoefficient"].asDouble();

    // Render properties
    fluidRender.particles_color().conservativeResize(pa, 3);
    auto col = fluidRender.particles_color().block(ps, 0, pn, 3);
    if(fluid.isMember("color")){
        col.col(0).array() = fluid["color"][0].asDouble();
        col.col(1).array() = fluid["color"][1].asDouble();
        col.col(2).array() = fluid["color"][2].asDouble();
    }
    fluidRender.particles_radius().conservativeResize(pa, 1);
    auto rad = fluidRender.particles_radius().block(ps, 0, pn, 1);
    rad.setOnes();
    if(fluid.isMember("radius")){
        rad.array() = scale * fluid["radius"].asDouble();
    }
}

void SceneLoader::readFluid(Physics::Scene& physScene, Render::Scene& renderScene, const Json::Value& fluid) const {
    if(physScene.fluid.get() == nullptr){
        setEmptyFluid(physScene, renderScene);
    }
    if(fluid.isMember("fluids")){
        for(auto& subFluid : fluid["fluids"]){
            readSubFluid(physScene, renderScene, subFluid);
        }
    } else {
        readSubFluid(physScene, renderScene, fluid);
    }
    auto& fluidPhys = *physScene.fluid;
    Float h = fluid["kernelRadius"].asDouble() * scale;
    fluidPhys.h(h);
    BOOST_LOG_TRIVIAL(debug) << "Loader: kernelRadius: " << h;

    // create physics data

    if(fluid.isMember("stiffnessConstant"))
        fluidPhys.stiffnessConstant(fluid["stiffnessConstant"].asDouble());
    if(fluid.isMember("restDensity"))
        fluidPhys.rest_density(fluid["restDensity"].asDouble());
    if(fluid.isMember("surfaceTension"))
        fluidPhys.surface_tension(fluid["surfaceTension"].asDouble());
    if(fluid.isMember("fluidViscosity"))
        fluidPhys.fluid_viscosity(fluid["fluidViscosity"].asDouble());
    if(fluid.isMember("boundaryViscosity"))
        fluidPhys.boundary_viscosity(fluid["boundaryViscosity"].asDouble());
    if(fluid.isMember("pressureGamma"))
        fluidPhys.pressure_gamma(fluid["pressureGamma"].asDouble());

    if(fluid.isMember("velocityCorrectionCoefficientMin"))
        fluidPhys.particles_lower_velocity_correction_limit() = fluid["velocityCorrectionCoefficientMin"].asDouble();
    if(fluid.isMember("weakeningSpeed"))
        fluidPhys.particles_weakening_speed() = fluid["weakeningSpeed"].asDouble();
    if(fluid.isMember("yieldCriterion"))
        fluidPhys.particles_yield_criterion() = fluid["yieldCriterion"].asDouble();
    if(fluid.isMember("mergeThreshold"))
        fluidPhys.merge_threshold() = fluid["mergeThreshold"].asDouble();
    if(fluid.isMember("splitThreshold"))
        fluidPhys.split_threshold() = fluid["splitThreshold"].asDouble();
    if(fluid.isMember("boundaryMergeThreshold"))
        fluidPhys.boundary_merge_threshold() = fluid["boundaryMergeThreshold"].asDouble();
    if(fluid.isMember("continuousMerge"))
        fluidPhys.continuous_merge(fluid["continuousMerge"].asBool());
    if(fluid.isMember("continuousSplit"))
        fluidPhys.continuous_split(fluid["continuousSplit"].asBool());
}

Coordinates2d SceneLoader::readCoordinates(const Json::Value& coords) const {
    Coordinates2d vertices(coords.size(), 2);
    for (unsigned int i = 0; i < coords.size(); i++) {
        vertices(i, 0) = coords[i][0].asDouble();
        vertices(i, 1) = coords[i][1].asDouble();
    }
    return vertices;
}

void SceneLoader::readObject(Physics::Scene& physScene, Render::Scene& renderScene, const Json::Value& obj) const {
    const auto& vertex = obj["vertices"];
    int faceCount = obj["faces"].size();

    BOOST_LOG_TRIVIAL(debug) << "OBJECT: " << obj["name"] << std::endl << "-----------";

    // iterate through the object's vertices
    Coordinates2d vertices = readCoordinates(vertex);
    if(obj.isMember("rotation")){
        Float degreeAngle = obj["rotation"].asDouble();
        Eigen::Rotation2D<Float> rot(degreeAngle/180*M_PI);
        RotationMatrix rotM = rot.toRotationMatrix();
        vertices = vertices * rotM.transpose();
    }
    vertices = vertices*scale;
    vertices.col(0) = vertices.col(0).array() + transl_x;
    vertices.col(1) = vertices.col(1).array() + transl_y;
    // iterate through the object's faces
    TriangleList faces(faceCount, 3);
    for (int i = 0; i < faceCount; i++) {
        faces(i, 0) = obj["faces"][i][0].asInt();
        faces(i, 1) = obj["faces"][i][1].asInt();
        faces(i, 2) = obj["faces"][i][2].asInt();
    }

    BOOST_LOG_TRIVIAL(debug) << "Vertices:" << std::endl << vertices;
    BOOST_LOG_TRIVIAL(debug) << "Faces:" << std::endl << faces;


    if(m_convertMeshToBoundingBox){
        convertMeshToBoundingBox(vertices, faces);
    }

    // create a PhysicsMesh object for Box2D
    auto vertex_ptr = std::make_shared<Coordinates2d>(vertices);
    auto face_ptr = std::make_shared<TriangleList>(faces);
    Physics::Mesh physMesh(vertex_ptr, face_ptr);

    // create a body for box2d
    b2BodyDef bodyDefinition;
    //bodyDefinition.position.Set(min_x + half_width, min_y + half_width); // the center of the body
    if(obj.isMember("position")){
        bodyDefinition.position.x = scale*obj["position"][0].asDouble();
        bodyDefinition.position.y = scale*obj["position"][1].asDouble();
        BOOST_LOG_TRIVIAL(debug) << "position: " << bodyDefinition.position.x << ", " << bodyDefinition.position.y;
    }
    if(obj.isMember("initialVelocity")){
        bodyDefinition.linearVelocity.x = obj["initialVelocity"][0].asDouble();
        bodyDefinition.linearVelocity.y = obj["initialVelocity"][1].asDouble();
        BOOST_LOG_TRIVIAL(debug) << "velocity: " << bodyDefinition.linearVelocity.x << ", " << bodyDefinition.linearVelocity.y;
    }
    if(obj.isMember("acceleration")){
        // only for non-dynamic objects
        physMesh.linearAcceleration.x = obj["acceleration"][0].asDouble();
        physMesh.linearAcceleration.y = obj["acceleration"][1].asDouble();
        BOOST_LOG_TRIVIAL(debug) << "acceleration: " << physMesh.linearAcceleration.x << ", " << physMesh.linearAcceleration.y;
    }
    if(obj.isMember("rotation")){
        //bodyDefinition.angle = obj["rotation"].asDouble()/180*M_PI;
    }
    if(obj.isMember("angularVelocity")){
        //bodyDefinition.angularVelocity = obj["angularVelocity"].asDouble();
    }
    if(obj.isMember("angularAcceleration")){
        //physMesh.angularAcceleration = obj["angularAcceleration"].asDouble();
    }
    if(obj.isMember("dynamic")){
        BOOST_LOG_TRIVIAL(warning) << "Found deprecated property 'dynamic', please use 'type' = 'dynamic'. Ignoring property.";
    }
    if(obj.isMember("type") && obj["type"].asString() == "dynamic"){
        bodyDefinition.type = b2_dynamicBody;
    } else if (obj.isMember("type") && obj["type"].asString() == "kinematic") {
        bodyDefinition.type = b2_kinematicBody;
    } else {
        bodyDefinition.type = b2_staticBody;
    }
    physMesh.body = physScene.world.CreateBody(&bodyDefinition); // attach the body to the mesh

    // create bounding volume
    b2PolygonShape boundingBox;
    auto b2Verts = computeConvexHull(vertices);
    boundingBox.Set(b2Verts.data(), b2Verts.size());
    Float objDensity = 1000.0f;
    if(obj.isMember("density")){
        objDensity = obj["density"].asDouble();
    }
    physMesh.body->CreateFixture(&boundingBox, objDensity); // attach the bounding box to the body

    if(obj.isMember("velocityCorrectionCoefficient")){
        physMesh.particles_velocity_correction_coefficient() = obj["velocityCorrectionCoefficient"].asDouble();
    }

    // insert the box2d mesh into the scene (including body and bounding box)
    physScene.meshes.push_back(std::move(physMesh));


    // insert the mesh into the scene's rendering
    auto mesh = std::make_unique<Render::Mesh>(vertex_ptr, face_ptr);
    mesh->vertices_color() = Eigen::MatrixXd::Zero(vertex.size(), 3);
    mesh->vertices_color().array() += 0.5; // grey
    renderScene.meshes.push_back(std::move(mesh));
}

std::vector<b2Vec2> SceneLoader::computeConvexHull(const Coordinates2d& vertices) const{
    auto convexHull = Spatial::convex_hull(vertices);
    std::vector<b2Vec2> b2Verts(convexHull.rows());
    for(int i = 0; i < convexHull.rows(); ++i) {
        b2Verts[i].x = convexHull(i, 0);
        b2Verts[i].y = convexHull(i, 1);
    }
    return b2Verts;
}

std::vector<b2Vec2> SceneLoader::computeAABB(const Coordinates2d& vertices) const {
    double min_x, min_y, max_x, max_y;
    Coordinates2d maxCoords = vertices.colwise().maxCoeff();
    Coordinates2d minCoords = vertices.colwise().minCoeff();
    min_x = minCoords(0, 0);
    min_y = minCoords(0, 1);
    max_x = maxCoords(0, 0);
    max_y = maxCoords(0, 1);
    BOOST_LOG_TRIVIAL(debug) << "Bounding Box at (" << min_x << ", " << min_y << "), (" << max_x << ", " << max_y << ")";
    std::vector<b2Vec2> b2Verts(4);
    b2Verts[0].x = min_x;
    b2Verts[0].y = min_y;
    b2Verts[1].x = max_x;
    b2Verts[1].y = min_y;
    b2Verts[2].x = max_x;
    b2Verts[2].y = max_y;
    b2Verts[3].x = min_x;
    b2Verts[3].y = max_y;
    return b2Verts;
}

void SceneLoader::convertMeshToBoundingBox(Coordinates2d vertices, TriangleList& faces) const {
    double min_x, min_y, max_x, max_y;
    Coordinates2d maxCoords = vertices.colwise().maxCoeff();
    Coordinates2d minCoords = vertices.colwise().minCoeff();
    min_x = minCoords(0, 0);
    min_y = minCoords(0, 1);
    max_x = maxCoords(0, 0);
    max_y = maxCoords(0, 1);

    vertices.resize(4,2);
    vertices(0,0) = min_x;
    vertices(0,1) = min_y;
    vertices(1,0) = max_x;
    vertices(1,1) = min_y;
    vertices(2,0) = max_x;
    vertices(2,1) = max_y;
    vertices(3,0) = min_x;
    vertices(3,1) = max_y;
    faces.resize(2,3);
    faces(0,0) = 0;
    faces(0,1) = 1;
    faces(0,2) = 2;
    faces(1,0) = 2;
    faces(1,1) = 3;
    faces(1,2) = 0;
}


} // GooBalls
