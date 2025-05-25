# KPI Python Computation

**Key Performance Indicators (KPIs)** are essential metrics used to assess and measure performance in a specific domain. In the context of **gait analysis**, KPIs refer to quantifiable indicators that provide insights into various aspects of human locomotion. These indicators are used to analyze walking patterns, identify abnormalities, and optimize rehabilitation or performance training.

The computed KPI include:

- **Niosh KPI**:An index based on the guidelines of the National Institute for Occupational Safety and Health (NIOSH), used to assess biomechanical risk during occupational tasks, particularly lifting activities. It helps estimate the physical load on body segments (e.g., the lumbar spine) and supports the prevention of musculoskeletal disorders and overexertion injuries.
- **Gait Cycle Duration**: The total time taken to complete a full gait cycle (one complete step with both legs).
- **Cadence**: The number of steps taken per minute, an essential parameter for analyzing walking speed and rhythm.
- **Stride Length**: The distance covered in one full gait cycle, indicating spatial characteristics of walking.
- **Step Length**: The distance covered between consecutive foot contacts, providing insights into step symmetry.
- **Swing Time**: The duration for which a foot is off the ground, crucial for analyzing gait dynamics.
- **Stance Time**: The duration for which a foot is in contact with the ground, useful for stability assessments.
- **Double Support Time**: The period where both feet are simultaneously in contact with the ground, often used to assess balance.
- **Maximum Angular Velocity**: The peak rotational speed of specific joints during gait, relevant for joint kinematics analysis.
- **Center of Pressure (COP) Trajectory**: The path of the ground reaction force application point, useful for balance assessment.
- **Step Width (Swing Width)**: The lateral distance between steps, important for stability evaluation.
- **Path Length**: The total distance traveled by a specific body segment during gait, providing a measure of movement efficiency.
- **Steady-State Gait Events**: Identification of periods of consistent walking patterns, excluding acceleration and deceleration phases.



### Setting up the environment
You should have already cloned the repository [biomechanical-analysis-framework](https://github.com/ami-iit/biomechanical-analysis-framework) and installed the project.  

In the same environment where you are using the `biomechanical-analysis-framework` library, install the following packages:  

```sh
conda install idyntree 
conda install pip
```

**NOTE:** if you want to create a new Conda environment do the following steps:
```
conda create -n kpi-py idyntree pip
conda activate kpi-py
```

### Installing the package
After completing the setup, navigate to the `KPI` directory and install the package using:
```bash
cd biomechanical-analysis-framework/KPI
pip install .
```


Thanks to the `pyproject.toml` file included in the project, all the necessary dependencies for running the code will be installed automatically.


### Modifying the configuration file
Before running the script, you need to modify the .json configuration file with the required information. The file config.json to be updated is located in the following directory:

```plaintext
KPI/code/baf_kpi/configs/
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
    "niosh_kpi": false,
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
- niosh_kpi: enables/disables computation of Niosh KPI
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



<font color="green">**NOTE:**</font> The `.urdf` files can be generated using the [`human-model-generator`](https://github.com/ami-iit/human-model-generator) repository, while the `.stl` files for the `meshes` directory can be downloaded from the same repository.

### Running the script
Now you can run the KPI analysis using the following command:
```bash
baf-kpi-analyzer
```
When you execute the script, a graphical window will open, allowing you to select the `.json` file containing the required information for the KPI analysis that you modified in the previous step.

If no `.json file` is selected and the graphical window is closed, the script will use a default configuration file already present in the KPI directory.
