
set(DATA_SOURCE_H include/BiomechanicalAnalysis/WearablesImplementation/DataSources)

add_biomechanical_analysis_library(
  NAME                   WearablesImplementationDataSources
  SOURCES                src/Force6DSensor.cpp src/VirtualLinkSensor.cpp
  PUBLIC_HEADERS         ${DATA_SOURCE_H}/Force6DSensor.h ${DATA_SOURCE_H}/VirtualLinkSensor.h
  PUBLIC_LINK_LIBRARIES  BiomechanicalAnalysis::DataSources IWear::IWear
  INSTALLATION_FOLDER    WearablesImplementation/DataSources)
