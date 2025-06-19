# Arena-Models

This repository serves as a development platform for building an asset (vector) database and model querying for Arena with ROS2 scripts.

## How to Test It

1. **Create Workspace:**
   - Create a folder named `arena_models_ws` and a subfolder named `src` inside it.

2. **Clone the Repository:**
   - Navigate to the `src` folder and run:
     ```bash
     git clone https://github.com/your-repo-url.git
     ```

3. **Build the Workspace:**
   - Go back to the `arena_models_ws` directory and run:
     ```bash
     colcon build && source install/setup.bash
     ```

4. **Download Arena-Models Dataset for IsaacSim:**
   - Execute:
     ```bash
     ros2 run arena_models down
     ```

## Usage

### Building the Database
To build the database, run:
```bash
ros2 run arena_models build --buildtypes procthor --input_path Dataset-arena-models/ --output_database_name arena-models
```
To query the database, use:
```bash
ros2 run arena_models query --path_database arena_models_database/ --name_database arena-models --target src/arena_models/ sofa
```
- Note: The query result will be stored in 'config_id_file.txt'

To retrieve an object from the database by its ID, execute:
```bash
ros2 run arena_models get --path_database arena_models_database/ --name_database arena-models --target src/arena_models/ -id 20
```
- Note: The query result will be stored in 'config_result_file.txt'

Note: I have added our database in format of procthor in file `procthor_database.json`

Picture of our dataset for arena-models in isaacsim

![Dataset isaacsim for resedential](https://github.com/Arena-Rosnav/arena-models/blob/isaac-project/resedential_capture5.png)
![Dataset isaacsim for all enviroments](https://github.com/Arena-Rosnav/arena-models/blob/master/Arena-model-dataset1.png)
![Dataset isaacsim for hospital](https://github.com/Arena-Rosnav/arena-models/blob/master/Hospital_dataset2.png)
![Dataset isaacsim for office](https://github.com/Arena-Rosnav/arena-models/blob/master/Office_dataset1.png)
