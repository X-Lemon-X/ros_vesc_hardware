#pragma once

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define HARDWARE_PUBLIC_EXPORT __attribute__((dllexport))
#define HARDWARE_PUBLIC_IMPORT __attribute__((dllimport))
#else
#define HARDWARE_PUBLIC_EXPORT __declspec(dllexport)
#define HARDWARE_PUBLIC_IMPORT __declspec(dllimport)
#endif
#ifdef HARDWARE_PUBLIC_BUILDING_DLL
#define HARDWARE_PUBLIC HARDWARE_PUBLIC_EXPORT
#else
#define HARDWARE_PUBLIC HARDWARE_PUBLIC_IMPORT
#endif
#define HARDWARE_PUBLIC_TYPE HARDWARE_PUBLIC
#define HARDWARE_PUBLIC_LOCAL
#else
#define HARDWARE_PUBLIC_EXPORT __attribute__((visibility("default")))
#define HARDWARE_PUBLIC_IMPORT
#if __GNUC__ >= 4
#define HARDWARE_PUBLIC __attribute__((visibility("default")))
#define HARDWARE_PUBLIC_LOCAL __attribute__((visibility("hidden")))
#else
#define HARDWARE_PUBLIC
#define HARDWARE_PUBLIC_LOCAL
#endif
#define HARDWARE_PUBLIC_TYPE
#endif
