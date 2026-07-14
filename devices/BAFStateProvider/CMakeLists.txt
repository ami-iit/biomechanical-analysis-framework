# SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
# SPDX-License-Identifier: BSD-3-Clause

yarp_prepare_plugin(baf_state_provider
    TYPE baf::devices::BAFStateProvider
    INCLUDE BAFStateProvider.h
    CATEGORY device
    DEFAULT ON)

yarp_add_plugin(BAFStateProvider
    BAFStateProvider.cpp
    BAFStateProvider.h
    BAFStateEstimator.cpp
    BAFStateEstimator.h)

target_include_directories(BAFStateProvider
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>)

target_link_libraries(BAFStateProvider
    PUBLIC
        YARP::YARP_os
        YARP::YARP_dev
        YARP::YARP_init
        IWear::IWear
        iDynTree::idyntree-model
        iDynTree::idyntree-high-level
    PRIVATE
        BipedalLocomotion::ParametersHandlerYarpImplementation
        BiomechanicalAnalysis::IK
        BipedalLocomotion::ParametersHandlerTomlImplementation
        BiomechanicalAnalysis::Logging
        Eigen3::Eigen)

yarp_install(
    TARGETS BAFStateProvider
    COMPONENT runtime
    LIBRARY DESTINATION ${YARP_DYNAMIC_PLUGINS_INSTALL_DIR}
    ARCHIVE DESTINATION ${YARP_STATIC_PLUGINS_INSTALL_DIR}
    YARP_INI DESTINATION ${YARP_PLUGIN_MANIFESTS_INSTALL_DIR})
