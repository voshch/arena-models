note: if you want to install arena_models[build], you must use Python 3.11.* (bpy dependency).

## If your materials are missing in the output model
USD files can contain embedded MDL materials, which need to be baked before blender can bake them correctly.
You can specify `-o bake-mdl=/path/to/isaacsim/python.sh` to enable automatic baking of MDL materials during database build.
Specifying `-o bake-mdl` without a path uses a docker image from the (Nvidia nvcr registry)[nvcr.io/nvidia/isaac-sim]. (May not work consistently.)