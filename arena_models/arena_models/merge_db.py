import rclpy
import chromadb
from rclpy.node import Node
from text_processing.language_processing import load_spacy_model, embed_text, store_embedding, embed_text_with_weight, querying_embeddings
import argparse
import os
import shutil

class MergeDatabase(Node):
    def __init__(self, database1, database2, outputdb):
        super().__init__('merge_database_node')
        # Copy contents from both folders
        self.copy_folder_contents(database1, outputdb)
        self.copy_folder_contents(database2, outputdb)

        self.get_logger().info(f"Databases merged successfully into: {outputdb}")
        raise SystemExit
    
    def copy_folder_contents(self, src_folder, dest_folder):
        for root, dirs, files in os.walk(src_folder):
            # Calculate the relative path to preserve folder structure
            relative_path = os.path.relpath(root, src_folder)
            target_folder = os.path.join(dest_folder, relative_path)
            
            # Create target folder if it doesn't exist
            if not os.path.exists(target_folder):
                os.makedirs(target_folder)

            # Copy each file
            for file in files:
                src_file = os.path.join(root, file)
                dest_file = os.path.join(target_folder, file)
                
                # Handle file name conflicts by renaming
                if os.path.exists(dest_file):
                    base, ext = os.path.splitext(file)
                    counter = 1
                    while os.path.exists(dest_file):
                        dest_file = os.path.join(target_folder, f"{base}_{counter}{ext}")
                        counter += 1
                
                shutil.copy2(src_file, dest_file)

def main(args=None):
    rclpy.init(args=args)

    # Parse command-line arguments
    parser = argparse.ArgumentParser(description="Run the database query node.")
    parser.add_argument('database1', type=str,
                        help="Path to the database file.")
    parser.add_argument('database2', type=str,
                        help="Path to the configuration file.")
    parser.add_argument('outputdb', type=str,
                        help="The specific id to get the path file (e.g., 'white shelf').")
    parsed_args = parser.parse_args()

    # Pass the database path to the node
    database_node = MergeDatabase(parsed_args.database1, parsed_args.database2, parsed_args.outputdb)

    rclpy.spin(database_node)
    database_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()