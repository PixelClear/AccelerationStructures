#pragma once
#include <glew/glew.h>
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

		for (uint32_t i = 0; i < mesh.faces_.size(); i += 3)
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

		//Adjust AABB
		mesh.aabb_.min_ = glm::vec3(-2.0f);
		mesh.aabb_.max_ = glm::vec3(2.0f);

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

