import rclpy
import chromadb
from rclpy.node import Node
from text_processing.language_processing import load_spacy_model
import argparse
import os

class GetDatabase(Node):
    def __init__(self, database_path, database_name, target_path, id):
        super().__init__('get_database_node')
        client = chromadb.PersistentClient(path=f"{database_path}")
        spacy_model = load_spacy_model()
        collection = client.get_collection(f"{database_name}")
        result = collection.get(ids = [f'{id}'])
        prefix_path = f'{target_path}'
        filename = 'config_result_file.txt'
        full_path = os.path.join(prefix_path, filename)
        with open(full_path, 'w') as file:
            file.write(f"result:{result}\n")
        self.get_logger().info(f"file: {result}")
        raise SystemExit

def main(args=None):
    rclpy.init(args=args)

    # Parse command-line arguments
    parser = argparse.ArgumentParser(description="Run the database get node.")
    parser.add_argument('--path_database', type=str, required=True,
                        help="Path to the database file.")
    parser.add_argument('--name_database', type=str, required=True,
                        help="Path to the database file.")
    parser.add_argument('--target_path', type=str, required=True,
                        help="Path to the configuration file.")
    parser.add_argument('-id', type=str, required=True,
                        help="The specific id to get the path file (e.g., 'white shelf').")
    parsed_args = parser.parse_args()

    # Pass the database path to the node
    database_node = GetDatabase(parsed_args.path_database, parsed_args.name_database, parsed_args.target_path, parsed_args.id)

    rclpy.spin(database_node)
    database_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()