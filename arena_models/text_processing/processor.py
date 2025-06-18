import os
import glob
import yaml


def processors_object(obj):
    res = []
    with open(obj, "r") as f:
        data = yaml.load(f, Loader=yaml.SafeLoader)

    if "color" in data:
        color_asset = data.get("color")
        for color in color_asset:
            res.append(color.lower())

    if "material" in data:
        material_asset = data.get("material")
        for material in material_asset:
            res.append(material.lower())

    if "tags" in data:
        tags_asset = data.get("tags")
        for tag in tags_asset:
            res.append(tag.lower())

    if "desc" in data:
        desc_asset = data.get("desc")
        res.append(desc_asset.lower())

    # if "hoi" in data:
    #     hoi_asset = data.get("hoi")
    #     for hoi in hoi_asset:
    #         res.append(hoi)

    return " ".join(res)


def processors_charactermodels(models):
    res = []
    with open(models, "r") as f:
        data = yaml.load(f, Loader=yaml.SafeLoader)

    if "age" in data:
        age_asset = data.get("age")
        for age in age_asset:
            res.append(age.lower())

    if "ethnicity" in data:
        ethnicity_asset = data.get("ethnicity")
        for ethnicity in ethnicity_asset:
            res.append(ethnicity.lower())

    if "gender" in data:
        gender_asset = data.get("gender")
        for gender in gender_asset:
            res.append(gender.lower())

    if "hair" in data:
        hair_asset = data.get("hair")
        for hair in hair_asset:
            res.append(hair.lower())
        res.append("hair")

    if "clothing" in data:
        clothing_asset = data.get("clothing")
        for clothing in clothing_asset:
            res.append(clothing)

    return " ".join(res)


def processors_wallmaterials(wall):
    res = []
    with open(wall, "r") as f:
        data = yaml.load(f, Loader=yaml.SafeLoader)

    if "color" in data:
        color_asset = data.get("color")
        for color in color_asset:
            res.append(color.lower())

    if "material" in data:
        material_asset = data.get("material")
        for material in material_asset:
            res.append(material.lower())

    if "tags" in data:
        tags_asset = data.get("tags")
        for tag in tags_asset:
            res.append(tag.lower())

    if "desc" in data:
        desc_asset = data.get("desc")
        res.append(desc_asset.lower())

    # if "hoi" in data:
    #     hoi_asset = data.get("hoi")
    #     for hoi in hoi_asset:
    #         res.append(hoi)

    return " ".join(res)


def main():
    path = "arena-unity/Assets/Resources_moved/Objects/Prefabs/Bedroom/Closet/annotation.yaml"
    if os.path.exists(path):
        test_str = processors_object(path)
        print(test_str)
    else:
        print(f"Error: The file at {path} does not exist.")


if __name__ == "__main__":
    main()
