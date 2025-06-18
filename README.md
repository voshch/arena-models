# Arena-Models
Dev repository for building an asset (vector) database + model querying for arena

## How it works (when finished)
When building arena-models, a script will search for prefabs in the "Objects" and "CharacterModels" folders and search for materials in the "Wall Materials" folder. The path of each found prefab/material will be added to the db and its corresponding annotation.yaml file will be used for additional annotation. The assets will then be build seperately using the Asset Bundle system (Addressables) with predefined groups (groups subject to changes while dev). The database and assetbundles will then be released for use in the arena-unity repo.

## Adding your own Models
To add your own prop assets you will need to add a **folder** for each of your models into one of the folders under arena-unity/Assets/Resources_moved/Objects/Prefabs/_. Each asset folder should contain, on the top level, a configured **prefab** and an **annotation.yaml** file this prefab.
For adding character models or wall materials, create a folder directly under Resources_moved/\[CharacterModels/Wall Materials\] with the prefab/material file + annotation file in it.

It is important that the prefab file (for props and characters) or material file (for wall materials) are on the same directory level as the corresponding annotation.yaml file, so that the database recognizes their connection.
To conform with planned future structure changes, you should put all necessary model, texture and material files of an asset together in one folder.



An example could look like this:


![example_folder_structure](https://github.com/user-attachments/assets/500b22c1-afb8-47ae-b839-154d49c3e44e)

(for a _"Chair"_ Prop)

The **annotation.yaml** should follow this format:

![screenshot_annotation_file](https://github.com/Arena-Rosnav/arena-models/assets/149716284/5e3ec898-5cf6-48e0-960a-2d16eb612918)
