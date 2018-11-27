#include "scene_loader.hpp"

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

namespace SceneLoader {

using namespace d2;

bool showDebugOutput = true;

void loadScene(Physics::Scene& physScene, Render::Scene& aRenderScene, std::string path) {

	std::ifstream ifs(path);

	if (!ifs.is_open()) {
		std::cout << "Error on opening scene file!" << std::endl;
	} else {
		Json::Reader reader;
		Json::Value obj;
		reader.parse(ifs, obj);
		if (showDebugOutput) std::cout << "Loading Scene: " << obj["name"].asString() << std::endl;
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

			if (showDebugOutput) std::cout << std::endl << "OBJECT " << objIndex << ": " << obj["objects"][obj_index_str]["name"] << std::endl << "-----------" << std::endl;

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

			if (showDebugOutput) {
				std::cout << "Vertices:" << std::endl << vertices << std::endl;
				std::cout << "Faces:" << std::endl << faces << std::endl;
			}

			double min_x, min_y, max_x, max_y;
			Coordinates2d maxCoords = vertices.colwise().maxCoeff();
			Coordinates2d minCoords = vertices.colwise().minCoeff();
			min_x = minCoords(0, 0);
			min_y = minCoords(0, 1);
			max_x = maxCoords(0, 0);
			max_y = maxCoords(0, 1);
			double half_width = (max_x - min_x) / 2.0;
			double half_height = (max_y - min_y) / 2.0;

			if (showDebugOutput) std::cout << "Bounding Box at (" << min_x << ", " << min_y << "), (" << max_x << ", " << max_y << ")" << std::endl;


			// create a PhysicsMesh object for Box2D
			auto vertex_ptr = std::make_shared<Coordinates2d>(vertices);
			auto face_ptr = std::make_shared<TriangleList>(faces);
			Physics::Mesh physMesh(vertex_ptr, face_ptr);

			// create a body for box2d
			b2BodyDef bodyDefinition;
            //bodyDefinition.position.Set(min_x + half_width, min_y + half_width); // the center of the body
            bodyDefinition.position.Set(0, 0);
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
			auto mesh = std::make_unique<Render::Mesh>(vertex_ptr, face_ptr);
			mesh->vertices_color() = Eigen::MatrixXd::Zero(vertexCount, 3);
			mesh->vertices_color().array() += 0.5; // grey
			aRenderScene.meshes.push_back(std::move(mesh));
		}
	}
}

} // SceneLoader

} // GooBalls
