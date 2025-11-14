import enum


class ModelFormat(str, enum.Enum):
    OBJ = "obj"
    FBX = "fbx"
    USDZ = "usdz"
    USDA = "usda"
    USDC = "usdc"
    USD = "usd"
    DAE = "dae"
    GLTF = "gltf"
    GLB = "glb"
    SDF = "sdf"
