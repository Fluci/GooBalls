#include<iostream>
#include<fstream>
#include<jsoncpp/json/json.h>

#include "placeholder.hpp"
#include "generic/hello.hpp"
#include "rendering/2d/engine.hpp"
#include "rendering/2d/disk_fluid.hpp"
#include "physics/2d/engine.hpp"

using namespace GooBalls;
using namespace d2;

// TODO: this should be refactored somewhere close to the asset loader 
/// Create a random scene as mock data
void createRandomScene(Physics::Scene& physScene, Render::Scene& aRenderScene) {
    // some example data to allow first testing with rendering
    auto particleCoordinates = std::make_shared<Coordinates2d>(Coordinates2d::Random(10, 2));
    auto verts = std::make_shared<Coordinates2d>(Coordinates2d::Random(30, 2));
    auto triangles = std::make_shared<TriangleList>(TriangleList::Random(10, 3));
    triangles->array() = triangles->unaryExpr([](const VertexIndex x) { return std::abs(x)%30; });

    // create physics data
    auto fluidPhys = std::make_unique<Physics::Fluid>(particleCoordinates);
    fluidPhys->particles_velocity().setRandom(10, 2);
    physScene.fluid = std::move(fluidPhys);

    Physics::Mesh physMesh(verts, triangles);
    physScene.meshes.push_back(std::move(physMesh));


    // create scene floor
    b2BodyDef groundBodyDef;
    groundBodyDef.position.Set(0, -10);
    b2PolygonShape groundBox;
    groundBox.SetAsBox(50.0, 10.0);
    b2Body* groundBody = physScene.world.CreateBody(&groundBodyDef);
    groundBody->CreateFixture(&groundBox, 0.0f);
    // create first dynamic body
    b2BodyDef bodyDef;
    bodyDef.type = b2_dynamicBody;
    bodyDef.position.Set(0.0f, 4.0f);
    // create collision object (a polygon shape) -> bounding box
    b2PolygonShape dynamicBox;
    dynamicBox.SetAsBox(1.0f, 1.0f);
    // create a fixture: 
    // this defines physical properties of the collision shape
    b2FixtureDef fixtureDef;
    fixtureDef.shape = &dynamicBox;
    fixtureDef.density = 1.0f;
    fixtureDef.friction = 0.3f;
    // attach to body
    physScene.world.SetGravity(b2Vec2(physScene.gravity[0], physScene.gravity[1]));
    // store pointer to Rigid Body in our Scene-Mesh
    physScene.meshes[0].body = physScene.world.CreateBody(&bodyDef);
    physScene.meshes[0].body->CreateFixture(&fixtureDef);

    // create data for rendering
    std::unique_ptr<Render::DiskFluid> fluid = std::make_unique<Render::DiskFluid>(particleCoordinates);
    fluid->particles_color().setRandom(10, 3);
    fluid->particles_color().array() += 1.0;
    fluid->particles_color() /= 2.0;
    fluid->particles_radius().setRandom(10, 1);
    fluid->particles_radius().array() += 2.0;
    auto mesh = std::make_unique<Render::Mesh>(verts, triangles);
    mesh->vertices_color().setRandom(30,3);
    mesh->vertices_color().array() += 1.0;
    mesh->vertices_color() /= 2.0;
    aRenderScene.fluids.push_back(std::move(fluid));
    aRenderScene.meshes.push_back(std::move(mesh));



	// test parsing a scene
	std::ifstream ifs("../../../src/GooBallsApp/scenes/scene0.json");
	if (!ifs.is_open()) {
		std::cout << "Error on opening scene file!" << std::endl;
	} else {
		Json::Reader reader;
		Json::Value obj;
		reader.parse(ifs, obj);
		std::cout << "Scene Name: " << obj["name"].asString() << std::endl;
		double scale = obj["scale"].asDouble();
		double transl_x = obj["translation"][0].asDouble();
		double transl_y = obj["translation"][1].asDouble();
	
		// iterate through the scene's objects
		int objectCount = obj["objects"]["count"].asInt();
		for (int objIndex = 0; objIndex < objectCount; objIndex++) {

			std::string obj_index_str = std::to_string(objIndex);
			int vertexCount = obj["objects"][obj_index_str]["vertices"]["count"].asInt();
			int faceCount = obj["objects"][obj_index_str]["faces"]["count"].asInt();

			Coordinates2d vertices(vertexCount, 2);
			TriangleList faces(faceCount, 3);

			std::cout << std::endl << "OBJECT " << objIndex << ": " << obj["objects"][obj_index_str]["name"] << std::endl << "-----------" << std::endl;

			// iterate through the object's vertices
			for (int i = 0; i < vertexCount; i++) {
				std::string i_str = std::to_string(i);
				vertices(i, 0) = (scale * obj["objects"][obj_index_str]["vertices"][i_str][0].asDouble()) + transl_x;
				vertices(i, 1) = (scale * obj["objects"][obj_index_str]["vertices"][i_str][1].asDouble()) + transl_y;
			}

			// iterate through the object's faces
			for (int i = 0; i < faceCount; i++) {
				std::string i_str = std::to_string(i);
				faces(i, 0) = obj["objects"][obj_index_str]["faces"][i_str][0].asInt();
				faces(i, 1) = obj["objects"][obj_index_str]["faces"][i_str][1].asInt();
				faces(i, 2) = obj["objects"][obj_index_str]["faces"][i_str][2].asInt();
			}

			std::cout << "Vertices:" << std::endl << vertices << std::endl;
			std::cout << "Faces:" << std::endl << faces << std::endl;

			double min_x, min_y, max_x, max_y;
			Coordinates2d maxCoords = vertices.colwise().maxCoeff();
			Coordinates2d minCoords = vertices.colwise().minCoeff();
			min_x = minCoords(0, 0);
			min_y = minCoords(0, 1);
			max_x = maxCoords(0, 0);
			max_y = maxCoords(0, 1);
			double half_width = (max_x - min_x) / 2.0;
			double half_height = (max_y - min_y) / 2.0;

			std::cout << "Bounding Box at (" << min_x << ", " << min_y << "), (" << max_x << ", " << max_y << ")" << std::endl;


			// create a PhysicsMesh object for Box2D
			Physics::Mesh physMesh(std::make_shared<Coordinates2d>(vertices),
								   std::make_shared<TriangleList>(faces));

			// create a body for box2d
			b2BodyDef bodyDefinition;
			bodyDefinition.position.Set(min_x + half_width, min_y + half_width); // the center of the body
			physMesh.body = physScene.world.CreateBody(&bodyDefinition); // attach the body to the mesh

			// create a bounding box for box2d
			// temporarily just using axis aligned bounding boxes
			// (later we can use convex polygonal hulls where needed)
			// -> define in scene.json where more than axis aligned bounding box is necessary?
			// And either define by hand (counter clockwise) ordering of vertices on the convex hull -> scene.json
			// or maybe use CGAL or smth for this.
			b2PolygonShape boundingBox;
			boundingBox.SetAsBox(half_width, half_height);
			physMesh.body->CreateFixture(&boundingBox, 0.0f); // attach the bounding box to the body

			// insert the box2d mesh into the scene (including body and bounding box)
			physScene.meshes.push_back(std::move(physMesh));

	
			// insert the mesh into the scene's rendering
			auto mesh = std::make_unique<Render::Mesh>(std::make_shared<Coordinates2d>(vertices),
													   std::make_shared<TriangleList>(faces));
			mesh->vertices_color() = Eigen::MatrixXd::Zero(vertexCount, 3);
			mesh->vertices_color().array() += 0.5; // grey
			aRenderScene.meshes.push_back(std::move(mesh));
		}
	}
}


int main(int argc, char *argv[]){
    std::cout << "Hi there!" << std::endl;
    anotherHello();
    hello();

    Physics::Engine physEngine;
    Physics::Scene physScene;
    Render::Scene aRenderScene;
    createRandomScene(physScene, aRenderScene);
    Render::Engine render;
    // DEMO
    // show what we can
    std::cout << "first frame: " << std::endl;
    render.render(aRenderScene);
    // do one step
    // as both scene objects share the underlying particle positions, 
    // we don't need to copy anything for the render engine
    physEngine.advance(physScene, 0.1);
    std::cout << "second frame: " << std::endl;
    render.render(aRenderScene);
    return 0;
}
