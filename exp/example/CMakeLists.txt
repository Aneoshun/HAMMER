add_executable(hammerExample ./main.cpp)

target_link_libraries(hammerExample ${TBB_LIBRARIES} ${Boost_LIBRARIES} ${NLOPT_LIBRARIES})
