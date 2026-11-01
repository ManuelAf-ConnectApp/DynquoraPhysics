cmake_minimum_required(VERSION 3.20)

project(DynquoraPhysics VERSION 0.1.0 LANGUAGES C CXX)

option(DQ_BUILD_TESTS "Build Dynquora tests" ON)
option(DQ_BUILD_EXAMPLES "Build Dynquora examples" ON)
option(DQ_ENABLE_SANITIZERS "Enable address and undefined behavior sanitizers" OFF)
option(DQ_WARNINGS_AS_ERRORS "Treat warnings as errors" OFF)
option(DQ_ENABLE_NATIVE_ARCH "Enable host-specific CPU instructions (local builds only)" OFF)
option(DQ_BUILD_VIEWER "Build the optional raylib diagnostic viewer" OFF)

add_library(dynquora_physics
  src/c_api/physics.cpp
  src/contacts/particle_contacts.cpp
  src/solvers/xpbd.cpp
  src/spatial/uniform_grid.cpp)
add_library(Dynquora::Physics ALIAS dynquora_physics)
if(BUILD_SHARED_LIBS)
  target_compile_definitions(dynquora_physics PRIVATE DQ_BUILDING_LIBRARY PUBLIC DQ_BUILD_SHARED)
endif()
target_compile_features(dynquora_physics PRIVATE cxx_std_20)
target_include_directories(dynquora_physics PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_include_directories(dynquora_physics PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
set_target_properties(dynquora_physics PROPERTIES
  CXX_VISIBILITY_PRESET hidden VISIBILITY_INLINES_HIDDEN YES POSITION_INDEPENDENT_CODE YES)

if(MSVC)
  target_compile_options(dynquora_physics PRIVATE /W4 /permissive-)
  if(DQ_WARNINGS_AS_ERRORS)
    target_compile_options(dynquora_physics PRIVATE /WX)
  endif()
  if(DQ_ENABLE_NATIVE_ARCH)
    target_compile_options(dynquora_physics PRIVATE /arch:AVX2)
  endif()
else()
  target_compile_options(dynquora_physics PRIVATE -Wall -Wextra -Wpedantic)
  if(DQ_WARNINGS_AS_ERRORS)
    target_compile_options(dynquora_physics PRIVATE -Werror)
  endif()
  if(DQ_ENABLE_NATIVE_ARCH)
    target_compile_options(dynquora_physics PRIVATE -march=native)
  endif()
  if(DQ_ENABLE_SANITIZERS)
    target_compile_options(dynquora_physics PRIVATE -fsanitize=address,undefined -fno-omit-frame-pointer)
    # Consumers of a static library must link the sanitizer runtimes too.
    target_link_options(dynquora_physics PUBLIC -fsanitize=address,undefined)
  endif()
endif()

if(DQ_BUILD_EXAMPLES)
  add_executable(dq_c_minimal examples/c/minimal.c)
  target_compile_features(dq_c_minimal PRIVATE c_std_11)
  target_link_libraries(dq_c_minimal PRIVATE Dynquora::Physics)
  set_target_properties(dq_c_minimal PROPERTIES LINKER_LANGUAGE CXX)
endif()

if(DQ_BUILD_VIEWER)
  find_package(raylib 5.0 CONFIG QUIET)
  if(NOT TARGET raylib AND NOT TARGET raylib::raylib)
    find_package(raylib 5.0 QUIET)
  endif()
  if(NOT TARGET raylib AND NOT TARGET raylib::raylib)
    message(FATAL_ERROR "DQ_BUILD_VIEWER requires raylib 5.0+. Install it (for example: brew install raylib) or configure with -DDQ_BUILD_VIEWER=OFF.")
  endif()
  if(TARGET raylib::raylib)
    set(DQ_RAYLIB_TARGET raylib::raylib)
  else()
    set(DQ_RAYLIB_TARGET raylib)
  endif()
  add_executable(dq_viewer tools/viewer/main.cpp)
  target_compile_features(dq_viewer PRIVATE cxx_std_20)
  target_link_libraries(dq_viewer PRIVATE Dynquora::Physics ${DQ_RAYLIB_TARGET})
endif()

if(DQ_BUILD_TESTS)
  enable_testing()
  add_executable(dq_tests tests/physics_tests.cpp)
  target_link_libraries(dq_tests PRIVATE Dynquora::Physics)
  add_test(NAME dq_tests COMMAND dq_tests)
endif()

include(GNUInstallDirs)
install(TARGETS dynquora_physics EXPORT DynquoraPhysicsTargets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(EXPORT DynquoraPhysicsTargets NAMESPACE Dynquora::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/DynquoraPhysics)
