#pragma once
#include "SDLCallbacks.h"
#include "DebugOgl.h"
#include <chrono>
#include <string>

namespace MeshQuery
{
	template<typename T>
	class NonCopyable
	{
	public:
		NonCopyable(const NonCopyable& drv) = delete;
		NonCopyable(const NonCopyable&& drv) = delete;
		T& operator=(const T& rhs) = delete;
		T& operator=(const T&& rhs) noexcept = delete;

	protected:
		NonCopyable() = default;
		~NonCopyable() = default;

	};

	class ApplicationDriver : private NonCopyable<ApplicationDriver>
	{
	private:
		using milisec = std::chrono::milliseconds;
		using seconds = std::chrono::seconds;

		template<class Clock, class Duration>
		using TimeStamp = std::chrono::time_point<Clock, Duration>;

	public:
		
		ApplicationDriver() = delete;
		ApplicationDriver(SDLCallbacks* cb, std::string title, uint32_t width, uint32_t height, uint32_t xpos = SDL_WINDOWPOS_CENTERED, uint32_t ypos = SDL_WINDOWPOS_CENTERED, bool fullscreen = false);

		static SDL_Event getEvent() noexcept { return event_; }

		void renderMainLoop() noexcept;

		~ApplicationDriver()
		{
			SDL_DestroyWindow(window_);
			SDL_GL_DeleteContext(context_);
			SDL_Quit();
#if _DEBUG
			free_gl_db();
#endif
		}

	private:

		bool handleEvent() noexcept;

		SDL_GLContext context_;
		SDLCallbacks* cb_;
		SDL_Window* window_ = nullptr;
		std::string title_ = "Empty";
		uint32_t width_ = 600;
		uint32_t height_ = 400;
		bool fullscreen_ = false;
		TimeStamp<std::chrono::high_resolution_clock, milisec> timeStamp_;
		static SDL_Event event_;
	};
}
