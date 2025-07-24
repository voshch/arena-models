import rclpy
import chromadb
from rclpy.node import Node
from text_processing.language_processing import load_spacy_model, embed_text, store_embedding, embed_text_with_weight, querying_embeddings
import argparse
import os

class QueryDatabase(Node):
    def __init__(self, database_path, name_database, target_path, query_target):
        super().__init__('query_database_node')
        client = chromadb.PersistentClient(path=f"{database_path}")
        spacy_model = load_spacy_model()
        embedding_text = embed_text_with_weight(query_target, spacy_model)
        result = querying_embeddings(
            embedding_text, client, f"{name_database}", 1)
        id = result['ids'][0][0]

        prefix_path = f'{target_path}'
        filename = 'config_id_file.txt'
        full_path = os.path.join(prefix_path, filename)
        with open(full_path, 'w') as file:
            file.write(f"id:{id}\n")

        self.get_logger().info(f"ID: {id}")
        raise SystemExit

def main(args=None):
    rclpy.init(args=args)

    # Parse command-line arguments
    parser = argparse.ArgumentParser(description="Run the database query node.")
    parser.add_argument('--path_database', type=str, required=True,
                        help="Path to the database file.")
    parser.add_argument('--name_database', type=str, required=True,
                        help="Name of the database.")
    parser.add_argument('--target', type=str, required=True,
                        help="Path to the configuration file.")
    parser.add_argument('query_target', type=str, 
                        help="The specific target to query (e.g., 'white shelf').")
    parsed_args = parser.parse_args()

    # Pass the database path to the node
    database_node = QueryDatabase(parsed_args.path_database, parsed_args.name_database, parsed_args.target, parsed_args.query_target)

    rclpy.spin(database_node)
    database_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
