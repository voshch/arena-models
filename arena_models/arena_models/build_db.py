import rclpy
from rclpy.node import Node
import yaml
import os
import chromadb
import argparse
from text_processing.processor import processors_object
from text_processing.language_processing import load_spacy_model, embed_text, store_embedding, embed_text_with_weight
import sys
import json

class BuildDatabase(Node):
    def __init__(self, input_path, output_database_name, buildtypes):
        super().__init__('build_db_node')
        self.data_list = []
        self.building_database(input_path, output_database_name, buildtypes)
        raise SystemExit
        

    def building_database(self, input_path, output_database_name, buildtypes):

        main_path = os.path.join(
            os.getcwd(), f"{input_path}")
        
        list_type_models = [os.path.join(main_path, dir) for dir in os.listdir(main_path) if os.path.isdir(os.path.join(main_path, dir))]

        modelsDict = dict()
        data_procthor = dict()
        for type_model in list_type_models:
            list_type_env = [os.path.join(type_model, dir) for dir in os.listdir(type_model)]
            for type_env in list_type_env:
                list_models = [os.path.join(type_env, dir) for dir in os.listdir(type_env)]
                env = type_env.split("/", 8)[-1]
                self.data_list = []
                # self.get_logger().info(f"{list_models}")
                for model in list_models:
                    name = model.split("/", 8)[-1]
                    if name != "Materials":
                        # self.get_logger().info(f"{name}")
                        yaml_file_path = f"{model}/annotation.yaml"
                        usd_path = f"{model}/{name}.usd"
                        new_model = self.read_annotation_file(yaml_file_path)
                        new_model["usd_path"] = usd_path.split("/", 6)[-1]
                        modelsDict[yaml_file_path] = new_model
                        if buildtypes == 'procthor':
                            env = type_env.split("/", 8)[-1]
                            desc = new_model['desc']
                            bbox = new_model['bbox']
                            materials = new_model['material'].split(",")
                            material_list = []
                            for material in materials:
                                material_list.append([env,material])
                            max_image_pixel_length = 1
                            primary_property = new_model['hoi']
                            scenes = env
                            split = 'test'
                            self.get_logger().info(f"{primary_property}")
                            self.add_infrastructure(desc,bbox,material_list,max_image_pixel_length,env,primary_property,scenes,split)
                data_procthor[env] = self.data_list

        # self.get_logger().info(f"{data_procthor}")
        if buildtypes == 'procthor':
            with open('Dataset-arena-models/procthor_database.json', 'w') as json_file:
                json.dump(data_procthor, json_file, indent=4)
        
        spacy_model = load_spacy_model()
        client = chromadb.PersistentClient(path="./arena_models_database")
        guid = 0
        for model in modelsDict:
            guid += 1
            model_str = processors_object(model)
            new_embedding = embed_text_with_weight(model_str, spacy_model)
            # self.get_logger().info(f"{modelsDict[model]}")
            store_embedding(model_str, new_embedding,
                            modelsDict[model], guid, client, f"{output_database_name}")
            
        self.get_logger().info("Database built successfully!")
        



    def read_annotation_file(self, file_path: str) -> dict:
        with open(file_path, "r") as yaml_file:
            yaml_content = yaml.safe_load(yaml_file)
            yaml_content: dict
            for key, value in yaml_content.items():
                if value == None:
                    yaml_content[key] = ''
                if value != None:
                    if isinstance(value, list):
                        yaml_content[key] = ",".join(value)
            return yaml_content
    
    def add_infrastructure(self, asset_id, bounding_box, materials, max_image_pixel_length, objectType, primary_property, scenes, split):
        infrastructure_data = {
        "assetId": asset_id,
        "boundingBox": bounding_box,
        "materials": materials,
        "maxImagePixelLength": max_image_pixel_length,
        "objectType": objectType,
        "primaryProperty": primary_property,
        "scenes": scenes,
        "secondaryProperties": [],
        "split": split,
        "states": {}
        }
        self.data_list.append(infrastructure_data)

def main(args=None):
    rclpy.init(args=args)

    # Parse command-line arguments
    parser = argparse.ArgumentParser(description="Run database build node")
    parser.add_argument('--buildtypes', type=str, default="isaacsim",
                        help="Build database type.")
    parser.add_argument('--input_path', type=str, required=True,
                        help="Input dataset path.")
    parser.add_argument('--output_database_name', type=str, required=True,
                        help="Name of database")
    parsed_args = parser.parse_args()
    node = BuildDatabase(parsed_args.input_path, parsed_args.output_database_name, parsed_args.buildtypes)
    rclpy.spin_once(node)
    node.destroy_node()
    rclpy.shutdown()
    
    

if __name__ == '__main__':
    main()
