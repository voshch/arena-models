# arena-models

Build, query, and manage 3D model databases for [Arena](https://github.com/Arena-Rosnav).

`arena-models` converts datasets of 3D assets (objects and materials) into a searchable database: models are converted with Blender, optionally baked with Isaac Sim, and indexed in [ChromaDB](https://www.trychroma.com/) so they can be queried by natural-language description.

## Installation

```sh
pip install ./src
```

Note: if you want to install `arena_models[build]`, you must use Python 3.11.* (bpy dependency).

The repository is also an `ament_cmake` ROS package; building it in a colcon workspace installs the CLI into a dedicated venv.

## Usage

### Fetch prebuilt assets

```sh
# list available assets
arena-models net default list

# download assets into a local directory
arena-models net default fetch -o ./models Common/Material/ABS_Hard_Leather

# only download specific model formats, skip annotations
arena-models net default fetch -o ./models --format usdz --no-annotation
```

`default` resolves to the public GCS bucket; any other bucket name can be passed instead.

### Build a database

```sh
arena-models db ./models build -i ./dataset
```

Build options are passed with `-o key=value`:

| option | asset types | effect |
| --- | --- | --- |
| `formats` | object | comma-separated model formats to export (`*` for all) |
| `previews` | object | render preview images |
| `procthor` | object, material | export a ProcTHOR-compatible `asset-database.json` |
| `bake-mdl` | object | bake embedded MDL materials with Isaac Sim |

`--overwrite` controls re-building: `skip` (default) keeps existing entries, `overwrite` rebuilds them, `annotations` only refreshes annotations.

### Query

```sh
# best match (prints the asset path)
arena-models db ./models query material "light brown wood"

# top 5 matches with distance scores
arena-models db ./models query object "office chair" -n 5 --scores

# list everything of a type
arena-models db ./models list material
```

All logs and progress bars go to stderr; stdout only carries the results, so output can be piped.

## If your materials are missing in the output model

USD files can contain embedded MDL materials, which need to be baked before Blender can bake them correctly.
You can specify `-o bake-mdl=/path/to/isaacsim/python.sh` to enable automatic baking of MDL materials during database build.
Specifying `-o bake-mdl` without a path uses a docker image from the [Nvidia nvcr registry](https://nvcr.io/nvidia/isaac-sim). (May not work consistently.)

## Development

```sh
pip install "./src[test]"
pre-commit install

pytest tests/
ruff check . && ruff format src
```
