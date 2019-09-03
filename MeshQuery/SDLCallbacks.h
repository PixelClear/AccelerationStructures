#pragma once

#include "SDL.h"

namespace MeshQuery
{
	class SDLCallbacks
	{
	public:
		virtual void onInit() = 0;
		virtual void onKey(int keyCode, bool pressed) = 0;
		virtual void onViewportSizeChanged(int w, int h) = 0;
		virtual void onRenderFrame(double deltaTime) = 0;
		virtual void onUpdateFrame(double elapseTime) = 0;
	};

}
