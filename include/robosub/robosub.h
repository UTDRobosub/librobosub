#pragma once

#include "common.h"

#ifdef _WIN32

BOOL APIENTRY DllMain(HMODULE hModule,
	DWORD  ul_reason_for_call,
	LPVOID lpReserved
)
{
	switch (ul_reason_for_call)
	{
	case DLL_PROCESS_ATTACH:
	case DLL_THREAD_ATTACH:
	case DLL_THREAD_DETACH:
	case DLL_PROCESS_DETACH:
		break;
	}
	return TRUE;
}

#endif

#include "timeutil.h"
#include "fps.h"
#include "util.h"
#include "videoio.h"
#include "image.h"
#include "networkudp.h"
#include "networkvideo.h"
#include "telemetry.h"
#include "serial.h"
#include "robosub/image-processing/shape_recognition.h"
