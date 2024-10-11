# Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# BSD-3-Clause license.


set(H_PREFIX include/BiomechanicalAnalysis/bindings/ID)

add_baf_python_module(
    NAME IDBindings
    SOURCES src/Module.cpp src/InverseDynamics.cpp
    HEADERS ${H_PREFIX}/Module.h ${H_PREFIX}/InverseDynamics.h
    LINK_LIBRARIES BiomechanicalAnalysis::ID)
