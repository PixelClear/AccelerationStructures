#include "ApplicationDriver.h"
#include <glew/glew.h>

SDL_Event MeshQuery::ApplicationDriver::event_;

MeshQuery::ApplicationDriver::ApplicationDriver(SDLCallbacks * cb, std::string title, uint32_t width, uint32_t height, uint32_t xpos, uint32_t ypos, bool fullscreen) :
	cb_(cb), title_(title), width_(width), height_(height), fullscreen_(fullscreen)
{
	int flags;

	flags = (fullscreen) ? (SDL_WINDOW_FULLSCREEN) : 0;

	if (SDL_Init(SDL_INIT_VIDEO) < 0)
	{
		throw std::runtime_error("SDL_Init() failed to initialize!!");
	}

	window_ = SDL_CreateWindow(title.c_str(), xpos, ypos, width, height, flags | SDL_WINDOW_OPENGL);

	if (!window_)
	{
		throw std::runtime_error("SDL_CreateWindow() failed to create window!!");
	}

	context_ = SDL_GL_CreateContext(window_);

	if (!context_)
	{
		throw std::runtime_error("SDL_GL_CreateContext() context creation failed!!");
	}

	SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
	SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 4);
	SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 2);
	SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);

	GLenum err = glewInit();
	if (GLEW_OK != err)
	{
		throw std::runtime_error("glew init failed!!");
	}

	cb_->onInit(); // This will let application load assets 

	glClearColor(0.0, 0.0, 0.0, 1.0);
	glViewport(0, 0, width, height);

	cb_->onViewportSizeChanged(width, height);
}

void MeshQuery::ApplicationDriver::renderMainLoop() noexcept
{
	const double FPS = 60;
	const double frameDelay = 1000.0 / FPS;
	double elapsedTime = 0.0;

	static auto timeStart_ = std::chrono::time_point_cast<milisec>(std::chrono::high_resolution_clock::now());


	while (handleEvent())
	{
		const auto newTimeStamp = std::chrono::time_point_cast<milisec>(std::chrono::high_resolution_clock::now());
		//Time from start of game loop
		const double deltaTime = std::chrono::duration<double>(newTimeStamp - timeStart_).count(); //time_point.count is long long hence doube cast


		cb_->onUpdateFrame(deltaTime);
		cb_->onRenderFrame(deltaTime);

		SDL_GL_SwapWindow(window_);

		//Time to render this frame
		timeStamp_ = std::chrono::time_point_cast<milisec>(std::chrono::high_resolution_clock::now()); //This is time_point
		elapsedTime = std::chrono::duration<double>(timeStamp_ - newTimeStamp).count();

		//To control rendering to 60 FPS
		if (frameDelay > elapsedTime)
		{
			SDL_Delay(static_cast<int>(frameDelay - elapsedTime));
		}
	}
}

bool MeshQuery::ApplicationDriver::handleEvent() noexcept
{
	SDL_PollEvent(&event_);

	switch (event_.type)
	{
	case SDL_QUIT:
		return false;

	case SDL_KEYDOWN:
	case SDL_KEYUP:

		cb_->onKey(event_.key.keysym.sym, event_.type == SDL_KEYDOWN);
		break;
	case SDL_WINDOWEVENT:

		if (event_.window.type == SDL_WINDOWEVENT_SIZE_CHANGED)
		{
			width_ = event_.window.data1;
			height_ = event_.window.data2;
			cb_->onViewportSizeChanged(width_, height_);
		}
		break;
	default:
		break;
	}

	return true;
}