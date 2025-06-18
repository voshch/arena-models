import rclpy
from rclpy.node import Node
import yaml
import os
import chromadb
from text_processing.processor import processors_object
from text_processing.language_processing import load_spacy_model, embed_text, store_embedding, embed_text_with_weight
import sys
import requests

class DownDatabase(Node):
    def __init__(self):
        super().__init__('down_db_node')
        url = "https://drive.google.com/file/d/1CuvsayRBMn2bYxedDITPCxZ22TmA2jCX/view?usp=sharing"
        destination = '/home/kuro/arena_models_ws/src/arena_models/resource'
        self.download_file(url, destination)
        raise SystemExit
        

    def download_file(self, url, destination):
        try:
            # Send a GET request to the URL
            response = requests.get(url, stream=True)
            response.raise_for_status()  # Raise an error for HTTP codes 4xx/5xx

            # Write the content of the response to a file in chunks
            with open(destination, "wb") as file:
                for chunk in response.iter_content(chunk_size=8192):  # 8 KB chunks
                    if chunk:  # Skip empty chunks
                        file.write(chunk)

            print(f"File downloaded successfully and saved to: {destination}")
        except requests.exceptions.RequestException as e:
            print(f"An error occurred: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = DownDatabase()
    rclpy.spin_once(node)
    node.destroy_node()
    rclpy.shutdown()
    
    

if __name__ == '__main__':
    main()