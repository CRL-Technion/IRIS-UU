add_executable(build_graph build_graph.cc)
if (USE_C++17)
    target_link_libraries(build_graph crisp_robot planar_robot drone_robot)
else (USE_C++17)
    target_link_libraries(build_graph crisp_robot planar_robot)
endif()

add_executable(search_graph search_graph.cc)
target_link_libraries(search_graph search)

# add_executable(checkPOIandCollision checkPOIandCollision.cc)
# if (USE_C++17)
#     target_link_libraries(checkPOIandCollision crisp_robot planar_robot drone_robot)
# else (USE_C++17)
#     target_link_libraries(checkPOIandCollision crisp_robot planar_robot)
# endif()

add_executable(checkPOIandCollision_simplify checkPOIandCollision_simplify.cc)
if (USE_C++17)
    target_link_libraries(checkPOIandCollision_simplify crisp_robot planar_robot drone_robot)
else (USE_C++17)
    target_link_libraries(checkPOIandCollision_simplify crisp_robot planar_robot)
endif()

add_executable(checkPOIandCollision_simulator checkPOIandCollision_simulator.cc)
if (USE_C++17)
    target_link_libraries(checkPOIandCollision_simulator crisp_robot planar_robot drone_robot)
else (USE_C++17)
    target_link_libraries(checkPOIandCollision_simulator crisp_robot planar_robot)
endif()

add_executable(checkPOIandCollision_toy checkPOIandCollision_toy.cc)
if (USE_C++17)
    target_link_libraries(checkPOIandCollision_toy crisp_robot planar_robot drone_robot)
else (USE_C++17)
    target_link_libraries(checkPOIandCollision_toy crisp_robot planar_robot)
endif()



add_executable(checkProbModel checkProbModel.cc)
target_link_libraries(checkProbModel search)