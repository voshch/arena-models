import rclpy
from rclpy.node import Node
import yaml
import os
import chromadb
from text_processing.processor import processors_object
from text_processing.language_processing import load_spacy_model, embed_text, store_embedding, embed_text_with_weight
import sys

class BuildDatabase(Node):
    def __init__(self, input_path, output_database_name):
        super().__init__('build_db_node')
        self.building_database(input_path, output_database_name)
        raise SystemExit
        

    def building_database(self, input_path, output_database_name):
        main_path = os.path.join(
            os.getcwd(), f"{input_path}")

        fields = [os.path.join(main_path, dir) for dir in os.listdir(
            main_path) if os.path.isdir(os.path.join(main_path, dir))]

        modelsDict = dict()

        for field in fields:
            for dir in os.listdir(field):
                if dir == "Prefabs":
                    name = field.split("/", 9)[-1]
                    yaml_file_path = f"{os.path.join(field,dir)}/annotation.yaml"
                    prefab_meta_path = f"{os.path.join(field,dir)}/{name}.prefab.meta"
                    prefab_path = f"{os.path.join(field,dir)}/{name}.prefab"
                    new_model = self.read_annotation_file(yaml_file_path)
                    new_model["guid"] = self.get_guid_from_prefab_meta(prefab_meta_path)
                    new_model["prefab_path"] = prefab_path.split("/", 5)[-1]
                    modelsDict[yaml_file_path] = new_model


        spacy_model = load_spacy_model()
        client = chromadb.PersistentClient(path="./arena-models/text_processing/database")

        for model in modelsDict:
            model_str = processors_object(model)
            guid = modelsDict[model].pop("guid")
            new_embedding = embed_text_with_weight(model_str, spacy_model)
            store_embedding(model_str, new_embedding,
                            modelsDict[model], guid, client, f"{output_database_name}")
            
        self.get_logger().info("Database built successfully!")
        



    def read_annotation_file(self, file_path: str) -> dict:
        with open(file_path, "r") as yaml_file:
            yaml_content = yaml.safe_load(yaml_file)
            yaml_content: dict
            for key, value in yaml_content.items():
                if isinstance(value, list):
                    yaml_content[key] = ",".join(value)
            return yaml_content
    
    def get_guid_from_prefab_meta(self, prefab_meta_path: str) -> str:
        with open(prefab_meta_path, "r") as file:
            lines = file.readlines()
            for line in lines:
                line = line.strip()
                if line.startswith("guid:"):
                    return line.split(":")[1].strip()
        return "ERROR"

def main(args=None):
    rclpy.init(args=args)
    input_path = sys.argv[1]
    output_database_name = sys.argv[2]
    node = BuildDatabase(input_path, output_database_name)
    rclpy.spin_once(node)
    node.destroy_node()
    rclpy.shutdown()
    
    

if __name__ == '__main__':
    main()