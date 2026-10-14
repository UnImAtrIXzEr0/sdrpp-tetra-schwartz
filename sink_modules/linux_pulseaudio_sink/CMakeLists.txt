cmake_minimum_required(VERSION 3.13)
project(linux_pulseaudio_sink)

file(GLOB SRC "src/*.cpp")

include(${SDRPP_MODULE_CMAKE})

find_package(PkgConfig REQUIRED)
pkg_check_modules(PULSE REQUIRED libpulse)

target_include_directories(linux_pulseaudio_sink PRIVATE ${PULSE_INCLUDE_DIRS})
target_link_libraries(linux_pulseaudio_sink PRIVATE ${PULSE_LIBRARIES})
