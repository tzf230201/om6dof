// Copyright 2026 OM6DOF maintainers.
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

#ifndef OM6DOF_CONTROLLERS__VISIBILITY_CONTROL_H_
#define OM6DOF_CONTROLLERS__VISIBILITY_CONTROL_H_

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define OM6DOF_CONTROLLERS_EXPORT __attribute__((dllexport))
#define OM6DOF_CONTROLLERS_IMPORT __attribute__((dllimport))
#else
#define OM6DOF_CONTROLLERS_EXPORT __declspec(dllexport)
#define OM6DOF_CONTROLLERS_IMPORT __declspec(dllimport)
#endif
#ifdef OM6DOF_CONTROLLERS_BUILDING_DLL
#define OM6DOF_CONTROLLERS_PUBLIC OM6DOF_CONTROLLERS_EXPORT
#else
#define OM6DOF_CONTROLLERS_PUBLIC OM6DOF_CONTROLLERS_IMPORT
#endif
#define OM6DOF_CONTROLLERS_LOCAL
#else
#define OM6DOF_CONTROLLERS_EXPORT __attribute__((visibility("default")))
#define OM6DOF_CONTROLLERS_IMPORT
#define OM6DOF_CONTROLLERS_PUBLIC __attribute__((visibility("default")))
#define OM6DOF_CONTROLLERS_LOCAL __attribute__((visibility("hidden")))
#endif

#endif  // OM6DOF_CONTROLLERS__VISIBILITY_CONTROL_H_
