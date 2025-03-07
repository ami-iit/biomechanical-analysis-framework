# element_ifeel-aism

In this repository, we will store code, high-level tasks, discussions, and material, related to the research activity conducted together with [AISM - Associazione Italiana Sclerosi Multipla](https://www.aism.it/), under the superfivion of Gianpaolo Brichetto.  This research is developed within the framework of [iFeel](https://github.com/ami-iit/component_ifeel) project and aims to analyze/validate our technology in the rehabilitation context of multiple sclerosis patients.

## Guide to use the code
This project has been updated to make the code more accessible and user-friendly. Below are the steps to execute the code for KPI calculation in Python.

### Setting up the environment
It is recommended to use a clean Conda environment for this project. Run the following command to create and activate a new environment:
```
conda create -n kpi-py idyntree pip
conda activate kpi-py
```

### Cloning the Repository
The first step is to clone the repository, enter in it and switch to the `implementation_KPI_py` branch, which contains the implementation of KPI calculations in Python:

```bash
git clone https://github.com/ami-iit/element_ifeel-aism
cd element_ifeel-aism
git checkout implementation_KPI_py
```



### Installing the package
After completing the setup, navigate to the `KPI_Python` directory and install the package using:
```bash
cd KPI_Python
pip install .
```


Thanks to the `pyproject.toml` file included in the project, all the necessary dependencies for running the code will be installed automatically.


### Modifying the configuration file
Before running the script, you need to modify the .json configuration file with the required information. The file config.json to be updated is located in the following directory:

```plaintext
KPI_Python/code/baf_kpi/configs/
```

<font color="orange">**NOTE:**</font> The most critical variable to update is `experiment_dir`. This should be set to the path where your `dataset` directory is located. 

An example of a `.json file` is as follows:
```bash
{
    "subject_id": add-subject-id,               # (e.g., 1)
    "subject_mass": add-subject-mass,           # (e.g., 70)
    "trial_list": add-trial-list,               # (e.g., [1] or multiples [1, 2, 3, ...])
    "experiment_dir": "my_dir_path/dataset",
    "data_BAF": false,
    "visualize_model": false,
    "plot_kpis": false,
    "save_on": false,
    "video_recording": false,
    "leg_skeleton_viz": false    
}
```
This file includes:
- subject_id: the ID of the subject to analyze.
- trial_list: the list of trials to process.
- experiment_dir: the path to the 'dataset' folder. 
- data_BAF: indicates whether to use BAF data or HDE data.
- visualize_model: enables/disables visualization of the human model.
- plot_kpis: enables/disables the generation of KPI plots.
- save_on: enables/disables saving the analysis results.
- video_recording: enables/disables video recording.
- leg_skeleton_viz: enables/disables visualization of the leg skeleton.

<font color="red">**NOTE:**</font> the `dataset` directory can be placed anywhere, but it must adhere to the following structure:

``` plaintext
dataset/
├── meshes/                 # Contains .stl files for visualizing meshes
│   ├── file1.stl
│   ├── file2.stl
│   └── ...
├── S01/                    # Folder for subject 1
│   ├── S01.urdf            # URDF model for subject 1
│   ├── trial19/            # Folder for trial 19
│   │   └── raw_dataplayer/ # Contains experimental data
│   │       ├── human_Data.mat
│   │       └── ifeel_data.mat
│   └── trial23/            # Folder for trial 23
│       └── raw_dataplayer/
│           ├── human_Data.mat
│           └── ifeel_data.mat
├── S02/                    # Folder for subject 2
│   ├── S02.urdf            # URDF model for subject 2
│   ├── trial01/
│   │   └── raw_dataplayer/
│   │       ├── human_Data.mat
│   │       └── ifeel_data.mat
│   └── ...
└── ...
```




### Running the script
Fi you can run the KPI analysis using the following command:
```bash
baf-kpi-analyzer
```
When you execute the script, a graphical window will open, allowing you to select the `.json` file containing the required information for the KPI analysis that you modified in the previous step.

If no `.json file` is selected and the graphical window is closed, the script will use a default configuration file already present in the KPI_Python directory.
