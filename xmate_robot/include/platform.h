#pragma once

#undef LIBXmate_X64
#undef LIBXmate_X86
#undef LIBXmate_ARM64
#undef LIBXmate_ARM

#if defined(__amd64__) || defined(_M_AMD64)
#define LIBXmate_X64
#elif defined(__X86__) || defined(_M_IX86)
#define LIBXmate_X86
#elif defined(__aarch64__) || defined(_M_ARM64)
#define LIBXmate_ARM64
#elif defined(__arm__) || defined(_M_ARM)
#define LIBXmate_ARM
#endif

#undef LIBXmate_WINDOWS
#undef LIBXmate_LINUX

#if defined(_WIN32) || defined(_WIN64)
#define LIBXmate_WINDOWS
#elif defined(__unix) || defined(__unix__)
#define LIBXmate_LINUX
#endif
