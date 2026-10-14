#pragma once

#ifdef __linux__
#include <unistd.h>
#endif

#ifdef __APPLE__
#include <unistd.h>
#endif

#ifdef _WIN32
#define usleep(x) ::Sleep(x/1000)
#endif
