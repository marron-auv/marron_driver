// Copyright (c) 2024 MARRON Project
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef MARRON_DRIVER__VISIBILITY_CONTROL_H_
#define MARRON_DRIVER__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define MARRON_DRIVER_EXPORT __attribute__((dllexport))
#define MARRON_DRIVER_IMPORT __attribute__((dllimport))
#else
#define MARRON_DRIVER_EXPORT __declspec(dllexport)
#define MARRON_DRIVER_IMPORT __declspec(dllimport)
#endif
#ifdef MARRON_DRIVER_BUILDING_LIBRARY
#define MARRON_DRIVER_PUBLIC MARRON_DRIVER_EXPORT
#else
#define MARRON_DRIVER_PUBLIC MARRON_DRIVER_IMPORT
#endif
#define MARRON_DRIVER_PUBLIC_TYPE MARRON_DRIVER_PUBLIC
#define MARRON_DRIVER_LOCAL
#else
#define MARRON_DRIVER_EXPORT __attribute__((visibility("default")))
#define MARRON_DRIVER_IMPORT
#if __GNUC__ >= 4
#define MARRON_DRIVER_PUBLIC __attribute__((visibility("default")))
#define MARRON_DRIVER_LOCAL __attribute__((visibility("hidden")))
#else
#define MARRON_DRIVER_PUBLIC
#define MARRON_DRIVER_LOCAL
#endif
#define MARRON_DRIVER_PUBLIC_TYPE
#endif

#endif  // MARRON_DRIVER__VISIBILITY_CONTROL_H_
