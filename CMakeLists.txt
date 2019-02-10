cmake_minimum_required(VERSION 3.12)
project(raic2018)

set(CMAKE_CXX_STANDARD 17)

SET(CMAKE_CXX_FLAGS "-D_LINUX -O3 -g -Wall -Wno-unknown-pragmas -Werror=return-type -DENABLE_LOGGING -DCUSTOM_RUNNER")

file(GLOB_RECURSE csimplesocket_Sources
        csimplesocket/*.cpp csimplesocket/*.h)


file(GLOB_RECURSE model_Sources
        model/*.cpp model/*.h)

add_executable(raic2018 d_main.cpp d_blockingqueue.hpp d_tcpclient.hpp d_tcpclient.cpp d_format.hpp d_format.cpp myutils.hpp myutils3d.hpp Simulator.cpp Simulator.hpp MyStrat.cpp MyStrat.hpp

        MyStrategy.cpp MyStrategy.h
        RemoteProcessClient.cpp RemoteProcessClient.h
        Runner.cpp Runner.h
        Strategy.cpp Strategy.h
        ${csimplesocket_Sources} ${model_Sources} d_game.hpp d_game.cpp gameUtils.hpp d_oldversions.hpp d_glm_math.hpp d_oldversions.cpp Simulator2d.hpp Simulator2d.cpp)

target_link_libraries(raic2018 pthread boost_system boost_timer boost_iostreams)

add_definitions( -DENABLE_LOGGING=1 )
