
#include <glew/glew.h>
#include <glm/ext.hpp>
#include <iostream>

#include "SDLCallbacks.h"
#include "ApplicationDriver.h"
#include "MeshLoader.h"

using namespace MeshQuery;

namespace RenderAbstractAPI
{
	
	float cameraZoom = 8.0f;
	float cameraX = 0.0f;
	glm::mat4 projection;
	GLuint renderProg;
	GLuint modelLocation;
	GLuint viewLocation;
	GLuint projectionLocation;
	GLuint VAO;
	std::unique_ptr<OctreeNode> root;
	size_t objectSize = 0;

	bool ReadFile(const char* pFileName, std::string& outFile)
	{
		std::ifstream f(pFileName);
		
		if (f.is_open()) 
		{
			std::string line;
			while (getline(f, line)) 
			{
				outFile.append(line);
				outFile.append("\n");
			}

			f.close();
			return true;
		}
		else 
		{
			std::cout << "Error Loading file";
			return false;
		}
	}

	inline bool loadObject(std::string& fileName)
	{
		std::vector<tinyobj::shape_t> shapes;
		std::vector<tinyobj::material_t> materials;
		std::string err;

		bool ret = tinyobj::LoadObj(shapes, materials, err, fileName.c_str());

		if (err.find("ERROR") != std::string::npos)
		{
			printf("%s\n", err.c_str());
			return false;
		}

		mesh.vertices_ = shapes[0].mesh.positions;
		mesh.faces_ = shapes[0].mesh.indices;
		mesh.normals_ = shapes[0].mesh.normals;
	
		std::vector<glm::vec3> vert;
		for (size_t i = 0; i < mesh.vertices_.size(); i += 3)
		{
			glm::vec3 v;
			v.x = mesh.vertices_[i + 0];
			v.y = mesh.vertices_[i + 1];
			v.z = mesh.vertices_[i + 2];

			vert.push_back(v);
		}

		mesh.aabb_.min_.x = mesh.aabb_.min_.y = mesh.aabb_.min_.z = std::numeric_limits<float>::max();
		mesh.aabb_.max_.x = mesh.aabb_.max_.y = mesh.aabb_.max_.z = -std::numeric_limits<float>::max();

		for (uint32_t i = 0; i < mesh.faces_.size(); i+=3)
		{
			Triangle t;

			t.aabb_.min_.x = t.aabb_.min_.y = t.aabb_.min_.z = std::numeric_limits<float>::max();
			t.aabb_.max_.x = t.aabb_.max_.y = t.aabb_.max_.z = -std::numeric_limits<float>::max();

			t.vertices_[0] = vert[mesh.faces_[i + 0]];
			t.vertices_[1] = vert[mesh.faces_[i + 1]];
			t.vertices_[2] = vert[mesh.faces_[i + 2]];

			mesh.aabb_.extendBy(t.vertices_[0]);
			mesh.aabb_.extendBy(t.vertices_[1]);
			mesh.aabb_.extendBy(t.vertices_[2]);

			auto t1 = t.vertices_[0];
			auto t2 = t.vertices_[1];
			auto t3 = t.vertices_[2];

			t.aabb_.extendBy(t.vertices_[0]);
			t.aabb_.extendBy(t.vertices_[1]);
			t.aabb_.extendBy(t.vertices_[2]);

			mesh.triangles_.push_back(t);
		}
		
		using milisec = std::chrono::milliseconds;
		using seconds = std::chrono::seconds;
		

	    NoAcclerationStructure na;
		glm::vec3 p(-8.0f);

		auto t1 = std::chrono::time_point_cast<milisec>(std::chrono::high_resolution_clock::now());
		float f = na.findMinDistance(p, mesh.triangles_);
		auto t2 = std::chrono::time_point_cast<milisec>(std::chrono::high_resolution_clock::now());
		const auto timeForNaiveMethod = std::chrono::duration<double>(t2 - t1).count();

		//Adjust AABB
		mesh.aabb_.min_ = glm::vec3(-2.0f);
		mesh.aabb_.max_ = glm::vec3(2.0f);
	
		Octree oc;

		root = std::make_unique<OctreeNode>();
		root->aabb_ = mesh.aabb_;
		root->depth_ = 0;
		root->objectList_ = mesh.triangles_;
		root->isLeaf_ = false;

		oc.buildTree(root.get());

		for (auto t : root->objectList_)
		{
			oc.insertTriangle(root.get(), t);
		}

		const auto t3 = std::chrono::time_point_cast<milisec>(std::chrono::high_resolution_clock::now());
		float x = na.findMinDistance(p, root.get());
		const auto t4 = std::chrono::time_point_cast<milisec>(std::chrono::high_resolution_clock::now());
		const auto timeForOctreeMethod = std::chrono::duration<double>(t4 - t3).count();
		
		std::ofstream myfile;
		myfile.open("timings.txt");
		myfile << "timeForNaiveMethod :: " << timeForNaiveMethod << std::endl;
		myfile << "timeForOctreeMethod :: " << timeForOctreeMethod << std::endl;
		myfile.close();

		return true;
    }

	inline void initBuffers()
	{
		glGenVertexArrays(1, &VAO);
		glBindVertexArray(VAO);

		glGenBuffers(1, &mesh.indexVbo_);
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, mesh.indexVbo_);
		glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(GLuint) * mesh.faces_.size(), mesh.faces_.data(), GL_STATIC_DRAW);

		glGenBuffers(1, &mesh.vertexVbo_);
		glBindBuffer(GL_ARRAY_BUFFER, mesh.vertexVbo_);
		glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * mesh.vertices_.size(), mesh.vertices_.data(), GL_STATIC_DRAW);


		glGenBuffers(1, &mesh.normalVbo_);
		glBindBuffer(GL_ARRAY_BUFFER, mesh.normalVbo_);
		glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * mesh.normals_.size(), mesh.normals_.data(), GL_STATIC_DRAW);

		glEnableVertexAttribArray(0);
		glBindBuffer(GL_ARRAY_BUFFER, mesh.vertexVbo_);
		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, (void*)0);

		glEnableVertexAttribArray(1);
		glBindBuffer(GL_ARRAY_BUFFER, mesh.normalVbo_);
		glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, (void*)0);
	}

	inline bool initShaders()
	{
		int success;
		std::string vertexSource;
		std::string fragmentSource;

		GLuint vsShaderObj;
		GLuint fsShaderObj;

		ReadFile("../Assets/shader.vs", vertexSource);
		ReadFile("../Assets/shader.fs", fragmentSource);

		const char* p[1];
		p[0] = vertexSource.c_str();

		vsShaderObj = glCreateShader(GL_VERTEX_SHADER);
		glShaderSource(vsShaderObj, 1, p, NULL);
		glCompileShader(vsShaderObj);
		glGetShaderiv(vsShaderObj, GL_COMPILE_STATUS, &success);

		if (!success) 
		{
			char InfoLog[1024];
			glGetShaderInfoLog(vsShaderObj, 1024, NULL, InfoLog);
			std::cout << "Error compiling : '%s'\n" << InfoLog;
			return false;
		}

		const char* p1[1];
		p1[0] = fragmentSource.c_str();

		fsShaderObj = glCreateShader(GL_FRAGMENT_SHADER);
		glShaderSource(fsShaderObj, 1, p1, NULL);
		glCompileShader(fsShaderObj);
		glGetShaderiv(fsShaderObj, GL_COMPILE_STATUS, &success);

		if (!success) 
		{
			char InfoLog[1024];
			glGetShaderInfoLog(fsShaderObj, 1024, NULL, InfoLog);
			std::cout << "Error compiling : '%s'\n" << InfoLog;
			return false;
		}
		
		renderProg = glCreateProgram();
		glAttachShader(renderProg, vsShaderObj);
		glAttachShader(renderProg, fsShaderObj);

		glLinkProgram(renderProg);
		glGetProgramiv(renderProg, GL_LINK_STATUS, &success);

		if (!success) 
		{
			char InfoLog[1024];
			glGetProgramInfoLog(renderProg, 1024, NULL, InfoLog);
			std::cout << "Error Linking: '%s'\n" << InfoLog;
			return false;
		}

		glValidateProgram(renderProg);
		glGetProgramiv(renderProg, GL_VALIDATE_STATUS, &success);
		if (!success) {
			char InfoLog[1024];
			glGetProgramInfoLog(renderProg, 1024, NULL, InfoLog);
			fprintf(stderr, "Invalid shader program: '%s'\n", InfoLog);
			return false;
		}

		glUseProgram(renderProg);

		modelLocation = glGetUniformLocation(renderProg, "u_Model");
		viewLocation = glGetUniformLocation(renderProg, "u_View");
		projectionLocation = glGetUniformLocation(renderProg, "u_Projection");

		return true;
	}

};

class Callbacks : public SDLCallbacks
{
public:
	virtual void onInit()override;
	virtual void onKey(int keyCode, bool pressed) override;
	virtual void onViewportSizeChanged(int w, int h) override;
	virtual void onRenderFrame(double deltaTime) override;
	virtual void onUpdateFrame(double elapseTime) override;
};

void Callbacks::onInit()
{
	std::string asset{ "../Assets/model.obj" };

	if (!RenderAbstractAPI::loadObject(asset))
	{
		throw std::runtime_error("Cannot Load Assets!!");
	}

	RenderAbstractAPI::initBuffers();

	if (!RenderAbstractAPI::initShaders())
	{
		throw std::runtime_error("Failed to load shaders!!");
	}

#if _DEBUG
	init_gl_db();
	
#endif
}

void Callbacks::onKey(int keyCode, bool pressed)
{
	switch (keyCode)
	{
	case SDLK_s:
		RenderAbstractAPI::cameraZoom -= 0.01f;
		break;
	case SDLK_w:
		RenderAbstractAPI::cameraZoom += 0.01f;
		break;
	case SDLK_d:
		RenderAbstractAPI::cameraX += 0.01f;
		break;

	case SDLK_a:
		RenderAbstractAPI::cameraX -= 0.01f;
		break;
	}
}

void Callbacks::onViewportSizeChanged(int w, int h)
{
    RenderAbstractAPI::projection = glm::perspective(45.0f, w / (float)h, 1.f, 1000.0f);
}

void print(OctreeNode* node)
{
	if (node == nullptr)
		return;

	float white[] = { 1.0f, 1.0f, 1.0f, 1.0f };
	for (int i = 0; i < 8; i++)
	{
		if (node->child_[i] != nullptr && node->child_[i]->objectList_.size())
		{
			add_gl_db_aabb(&node->child_[i]->aabb_.min_[0], &node->child_[i]->aabb_.max_[0], white);
			RenderAbstractAPI::objectSize += node->child_[i]->objectList_.size();
		}
		
		print(node->child_[i].get());
	}
}

void Callbacks::onRenderFrame(double deltaTime)
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glEnable(GL_DEPTH);
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_CULL_FACE);

	glUseProgram(RenderAbstractAPI::renderProg);

	glm::mat4 view = glm::lookAt(glm::vec3(0.0, 0.0, 0.0), glm::vec3(0.0, 0.0, -5.0), glm::vec3(0.0, 1.0, 0.0));
	glm::mat4 model = glm::translate(glm::mat4(1.0), glm::vec3(0.0, 0.0, -RenderAbstractAPI::cameraZoom));
	model = glm::rotate(model, RenderAbstractAPI::cameraX, glm::vec3(0.0, 1.0, 0.0));

	glUniformMatrix4fv(RenderAbstractAPI::modelLocation, 1, GL_FALSE, &model[0][0]);
	glUniformMatrix4fv(RenderAbstractAPI::viewLocation, 1, GL_FALSE, &view[0][0]);
	glUniformMatrix4fv(RenderAbstractAPI::projectionLocation, 1, GL_FALSE, &RenderAbstractAPI::projection[0][0]);

	glBindVertexArray(RenderAbstractAPI::VAO);
    glDrawElements(GL_TRIANGLES, static_cast<GLsizei>(mesh.faces_.size()), GL_UNSIGNED_INT, 0);

#if _DEBUG
	float white[] = { 1.0f, 1.0f, 1.0f, 1.0f };
	glm::mat4 mvp = RenderAbstractAPI::projection * view * model;

	add_gl_db_aabb(&mesh.aabb_.min_[0], &mesh.aabb_.max_[0], white);
	update_gl_db_cam_mat(&mvp[0][0]);

	RenderAbstractAPI::objectSize = 0;
	print(RenderAbstractAPI::root.get());

	const auto temp = RenderAbstractAPI::objectSize;

	draw_gl_db(false);


#endif
}

void Callbacks::onUpdateFrame(double elapseTime)
{
}

int main(int argc, char* argv[])
{
	try
	{
		Callbacks cb;

		std::unique_ptr<ApplicationDriver> ge = std::make_unique<ApplicationDriver>(&cb, "MeshQuery", 640, 640, SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, false);

		ge->renderMainLoop();
	}
	catch (std::exception e)
	{
		std::cerr << e.what();
	}
	return 0;
}