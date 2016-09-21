add_executable(hier_arm ./hier_arm.cpp ./arm_hori.cpp)
target_link_libraries(hier_arm ${TBB_LIBRARIES} ${Boost_LIBRARIES} ${NLOPT_LIBRARIES})
