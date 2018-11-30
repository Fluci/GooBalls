#include "scene_loader.hpp"
#include "spatial/2d/convex_hull.hpp"
#include <boost/log/trivial.hpp>

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

void SceneLoader::loadScene(Physics::Scene& physScene, Render::Scene& renderScene, std::string path) {

	std::ifstream ifs(path);

	if (!ifs.is_open()) {
		std::cout << "Error on opening scene file!" << std::endl;
	} else {
		Json::Reader reader;
        Json::Value scene;
        reader.parse(ifs, scene);
        BOOST_LOG_TRIVIAL(info) << "Loading Scene: " << scene["name"].asString() << std::endl;
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
        for (int objIndex = 0; objIndex < scene["objects"].size(); objIndex++) {
            const auto& obj = scene["objects"][objIndex];
            readObject(physScene, renderScene, obj);
		}
	}
}

void SceneLoader::readFluid(Physics::Scene& physScene, Render::Scene& renderScene, const Json::Value& fluid) const {
    /*
    //physScene.gravity.array() *= 0.0;
    constexpr int PN_X = 10;
    int PN_Y = 20;
    int PN = PN_Y * PN_X;

    // some example data to allow first testing with rendering
    auto particleCoordinates = std::make_shared<Coordinates2d>(Coordinates2d::Random(PN, 2)/2.0);
    for(int i = 0; i < PN_X; ++i){
        for(int j = 0; j < PN_Y; ++j){
            (*particleCoordinates)(i*PN_Y+j, 0) = i;
            (*particleCoordinates)(i*PN_Y+j, 1) = j;
        }
    }
    auto boundaryCoords = std::make_shared<Coordinates2d>();
    (*particleCoordinates) = (*particleCoordinates) * 0.032;
    particleCoordinates->col(0).array() += 0.27;
    particleCoordinates->col(1).array() += 0.7;
    std::cout << "particles: \n";
    for(int i = 0; i < particleCoordinates->rows(); ++i){
        std::cout << (*particleCoordinates)(i,0) << " " << (*particleCoordinates)(i,1) << std::endl;
    }
    int VN = 3*4; // number of verts, multiple of three
    int TN = 2;
    auto verts = std::make_shared<Coordinates2d>(Coordinates2d::Random(VN, 2)*0.5);
    auto triangles = std::make_shared<TriangleList>(TriangleList::Random(TN, 3));
    triangles->array() = triangles->unaryExpr([VN](const VertexIndex x) { return std::abs(x)%VN; });
    double h = 0.05;

    // create physics data
    auto fluidPhys = std::make_unique<Physics::Fluid>(particleCoordinates, boundaryCoords);
    fluidPhys->particles_velocity().setRandom(PN, 2);
    fluidPhys->particles_velocity() *= 0.0;
    fluidPhys->particles_mass().resize(PN);
    // compute the mass from rho*V = m
    fluidPhys->particles_mass().array() = 100;
    fluidPhys->h(h);
    fluidPhys->stiffnessConstant(10000);
    fluidPhys->rest_density(1000.0);
    fluidPhys->surface_tension(0.0);
    fluidPhys->fluid_viscosity(.03);
    fluidPhys->boundary_viscosity(.03);
    fluidPhys->pressure_gamma(7);
    physScene.fluid = std::move(fluidPhys);*/
}

void SceneLoader::readObject(Physics::Scene& physScene, Render::Scene& renderScene, const Json::Value& obj) const {
    const auto& vertex = obj["vertices"];
    int faceCount = obj["faces"].size();

    Coordinates2d vertices(vertex.size(), 2);
    TriangleList faces(faceCount, 3);

    BOOST_LOG_TRIVIAL(info) << "OBJECT: " << obj["name"] << std::endl << "-----------";

    // iterate through the object's vertices
    for (int i = 0; i < vertex.size(); i++) {
        vertices(i, 0) = (scale * vertex[i][0].asDouble()) + transl_x;
        vertices(i, 1) = (scale * vertex[i][1].asDouble()) + transl_y;
    }

    // iterate through the object's faces
    for (int i = 0; i < faceCount; i++) {
        faces(i, 0) = obj["faces"][i][0].asInt();
        faces(i, 1) = obj["faces"][i][1].asInt();
        faces(i, 2) = obj["faces"][i][2].asInt();
    }

    BOOST_LOG_TRIVIAL(info) << "Vertices:" << std::endl << vertices;
    BOOST_LOG_TRIVIAL(info) << "Faces:" << std::endl << faces;


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
    double posx = 0, posy = 0;
    if(obj.isMember("position")){
        posx = scale*obj["position"][0].asDouble();
        posy = scale*obj["position"][1].asDouble();
    }
    bodyDefinition.position.Set(posx, posy);
    if(obj.isMember("dynamic") && obj["dynamic"].asBool()){
        bodyDefinition.type = b2_dynamicBody;
        /*
        // create collision object (a polygon shape) -> bounding box
        // create a fixture:
        // this defines physical properties of the collision shape
        b2FixtureDef fixtureDef;
        fixtureDef.shape = &dynamicBox;
        fixtureDef.density = 1.0f;
        fixtureDef.friction = 0.3f;
        physScene.meshes[1].body = physScene.world.CreateBody(&bodyDef);
        physScene.meshes[1].body->CreateFixture(&fixtureDef);*/
    }
    physMesh.body = physScene.world.CreateBody(&bodyDefinition); // attach the body to the mesh

    // create bounding volume
    b2PolygonShape boundingBox;
    auto b2Verts = computeConvexHull(vertices);
    boundingBox.Set(b2Verts.data(), b2Verts.size());
    physMesh.body->CreateFixture(&boundingBox, 100000.0f); // attach the bounding box to the body

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
    BOOST_LOG_TRIVIAL(info) << "Bounding Box at (" << min_x << ", " << min_y << "), (" << max_x << ", " << max_y << ")";
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
