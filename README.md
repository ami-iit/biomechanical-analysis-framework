# biomechanical-analysis-framework

The biomechanical-analysis-framework projects contains a set of C++ libraries that can be used to estimate a human kinematics and dynamics quantities.

## Dependencies

The dependencies can be installed in two ways; the first one is via `conda` creating a conda environment running the following steps.
 
* Clone the repo:

   ```
   git clone https://github.com/ami-iit/biomechanical-analysis-framework
   ```
* Create the conda environment from file to automatically install dependencies:
  ```
   cd biomechanical-analysis-framework
   conda env create -n <conda-environment-name> --file ci_env.yml
   ```
* Note: if you are already inside an existing conda environment, you can type instead the command below AFTER OPENING THE ` ci_env.yml`  FILE AND SUBSTITUTING THE ` name`  FIELD WITH YOUR ENV NAME:
  ```
   cd biomechanical-analysis-framework
   conda env update --file ci_env.yml
   ```

  
The second option is to install the dependencies via [robotology-superbuild](https://github.com/robotology/robotology-superbuild/blob/master/README.md), making sure to enable the following CMake options:
   - `ROBOTOLOGY_ENABLE_DYNAMICS`;
   - `ROBOTOLOGY_ENABLE_DYNAMICS_FULL_DEPS`;
   - `ROBOTOLOGY_ENABLE_ICUB_HEAD`;

## Install

Clone the repo (if not already done to install the dependencies) and create a `build` directory

```
git clone https://github.com/ami-iit/biomechanical-analysis-framework
cd biomechanical-analysis-framework
mkdir build
cd build
```

### Linux/macOS

From the build directory, configure the CMake project and install it in the `build/install` directory:

```bash

cmake -DCMAKE_BUILD_TYPE="Release" -DCMAKE_INSTALL_PREFIX="./install" ..
make
make install
```

Once the installation is completed, append the following lines to your `.bashrc`:
```bash
BAF_INSTALL_DIR=<directory-where-you-downloaded-baf>/build/install
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:${BAF_INSTALL_DIR}/lib
export CMAKE_PREFIX_PATH=$CMAKE_PREFIX_PATH:${BAF_INSTALL_DIR}
```

>:warning: If on macOS, replace LD_LIBRARY_PATH with DYLD_LIBRARY_PATH in your `.bash_profile`


### Windows

From the build directory, configure the CMake project and install it in the `build/install` directory as it follows:

```bash
cmake -G"Visual Studio 17 2022" .. 
cmake -DCMAKE_BUILD_TYPE="Release" -DCMAKE_INSTALL_PREFIX="./install" .
cmake --build . --config Release --target INSTALL
```

>:warning: Change Visual Studio 17 2022 according to your installed version.

Once the installation is completed, update the following environment variables:

- `PATH`  
Add `<directory-where-you-downloaded-BAF>/build/install/bin`


## KPI Computation
This repository also contains Python code for computing Key Performance Indicators (KPI) related to gait analysis, located in the `baf_kpi` folder. KPI refers to a set of biomechanical metrics used to assess human locomotion, providing essential indicators for evaluating gait efficiency, identifying potential abnormalities, and exploring different movement strategies.

For detailed instructions on how to install and use the Python package, as well as information on the KPIs we compute, please refer to the [`README`](.baf_kpi/README.md) in the `baf_kpi` directory.

