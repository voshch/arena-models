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

5. **Unzip the Dataset:**
   - Unzip `arena-models.zip`. Create a new folder named `Dataset-arena-models` and move the contents of `arena-models` into the new folder in `arena_models_ws`.

## Usage

### Building the Database
To build the database, run:
```bash
ros2 run arena_models build --buildtypes procthor --input_path Dataset-arena-models/ --output_database_name arena-models
```
To query the database, use:
```bash
ros2 run arena_models query --path_database arena_models_ws/ --name_database arena-models --target src/arena_models/ sofa
```
To retrieve an object from the database by its ID, execute:
```bash
ros2 run arena_models get --path_database arena_models_ws/ --name_database arena-models --target src/arena_models/ -id 20
```

Note: I have added our database in format of procthor in file `procthor_database.json`

Picture of our dataset for arena-models in isaacsim

![Dataset isaacsim for resedential](https://github.com/Arena-Rosnav/arena-models/blob/isaac-project/resedential_capture5.png)
