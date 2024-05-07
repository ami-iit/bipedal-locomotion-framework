if (FRAMEWORK_COMPILE_Unicycle)

    include_directories(${CMAKE_CURRENT_BINARY_DIR})
    configure_file(${CMAKE_CURRENT_SOURCE_DIR}/FolderPath.h.in ${CMAKE_CURRENT_BINARY_DIR}/FolderPath.h @ONLY)

    # Get the urdf model of the robot
    set(ERGOCUB_MODEL_EXPECTED_MD5 7d24f42cb415e660abc4bbc8a52d355f)
    if (EXISTS "${CMAKE_CURRENT_BINARY_DIR}/model.urdf")
      file(MD5 "${CMAKE_CURRENT_BINARY_DIR}/model.urdf" ERGOCUB_MODEL_CHECKSUM_VARIABLE)
      string(COMPARE EQUAL ${ERGOCUB_MODEL_CHECKSUM_VARIABLE} ${ERGOCUB_MODEL_EXPECTED_MD5} ERGOCUB_MODEL_UPDATED)
    else()
      set(ERGOCUB_MODEL_UPDATED FALSE)
    endif()

    if(NOT ${ERGOCUB_MODEL_UPDATED})
      message(STATUS "Fetching ergoCubSN000 model from icub-tech-iit/ergocub-software...")
      file(DOWNLOAD https://raw.githubusercontent.com/icub-tech-iit/ergocub-software/f28733afcbbfcc99cbac13be530a6a072f832746/urdf/ergoCub/robots/ergoCubSN000/model.urdf
        ${CMAKE_CURRENT_BINARY_DIR}/model.urdf
        EXPECTED_MD5 ${ERGOCUB_MODEL_EXPECTED_MD5})
    endif()

    add_executable(UnicycleTrajectoryGeneratorExample main.cpp)

    target_link_libraries(UnicycleTrajectoryGeneratorExample
    BipedalLocomotion::Unicycle
    BipedalLocomotion::Planners
    matioCpp::matioCpp
    ${YARP_LIBRARIES}
)
endif()

