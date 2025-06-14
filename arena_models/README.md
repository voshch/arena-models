# Arena-Models
Dev repository for building an asset (vector) database + model querying for arena

## How it works (when finished)
When building arena-models, a script will search for file .usd in "Objects". The path of each found usd/material will be added to the db and its corresponding annotation.yaml file will be used for additional annotation. The assets will then be build seperately using the Asset Bundle system (Addressables) with predefined groups (groups subject to changes while dev). The database and assetbundles will then be released for use in the arena-isaacsim repo.



An example could look like this:


![example_folder_structure](https://github.com/user-attachments/assets/500b22c1-afb8-47ae-b839-154d49c3e44e)

(for a _"Chair"_ Prop)

The **annotation.yaml** should follow this format:

![screenshot_annotation_file](https://github.com/Arena-Rosnav/arena-models/assets/149716284/5e3ec898-5cf6-48e0-960a-2d16eb612918)
