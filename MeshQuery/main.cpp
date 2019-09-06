
#include <glew/glew.h>
#include <glm/ext.hpp>
#include <iostream>

#include "SDLCallbacks.h"
#include "ApplicationDriver.h"
#include "RenderAbstractAPI.h"
#include "AcclerationStructures.h"

using namespace MeshQuery;

using milisec = std::chrono::milliseconds;
using seconds = std::chrono::seconds;
std::unique_ptr<OctreeNode> root;

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
	NoAcclerationStructure na;
	glm::vec3 p(-8.0f);

	auto t1 = std::chrono::time_point_cast<milisec>(std::chrono::high_resolution_clock::now());
	const auto naivePair = na.getClosestPoint(p, mesh.triangles_);
	auto t2 = std::chrono::time_point_cast<milisec>(std::chrono::high_resolution_clock::now());
	const auto timeForNaiveMethod = std::chrono::duration<double>(t2 - t1).count();
	
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
	const auto ocPair = na.getClosestPoint(p, root.get());
	const auto t4 = std::chrono::time_point_cast<milisec>(std::chrono::high_resolution_clock::now());
	const auto timeForOctreeMethod = std::chrono::duration<double>(t4 - t3).count();

	std::ofstream myfile;
	myfile.open("Results.txt");
	myfile << "Point p :: " << "(" << p.x << "," << p.y << "," << p.z << ")" << std::endl;
	myfile << "ClosestPoint q :: " << "(" << p.x << "," << p.y << "," << p.z << ")" << std::endl;
	myfile << "timeForNaiveMethod :: " << timeForNaiveMethod << std::endl;
	myfile << "timeForNaiveMethod :: " << timeForNaiveMethod << std::endl;
	myfile << "timeForOctreeMethod :: " << timeForOctreeMethod << std::endl;
	myfile.close();
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
	print(root.get());

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