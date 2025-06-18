import rclpy
import chromadb
from rclpy.node import Node
from text_processing.language_processing import load_spacy_model, embed_text, store_embedding, embed_text_with_weight, querying_embeddings
import argparse

class GetDatabase(Node):
    def __init__(self, database_path, target_path, id):
        super().__init__('get_database_node')
        client = chromadb.PersistentClient(path=f"{database_path}")
        spacy_model = load_spacy_model()
        collection = client.get_collection("objectsDB")
        result = collection.get(ids = [f'{id}'])
        self.get_logger().info(f"file: {result}")
        raise SystemExit

def main(args=None):
    rclpy.init(args=args)

    # Parse command-line arguments
    parser = argparse.ArgumentParser(description="Run the database query node.")
    parser.add_argument('--database', type=str, required=True,
                        help="Path to the database file.")
    parser.add_argument('--target', type=str, required=True,
                        help="Path to the configuration file.")
    parser.add_argument('-id', type=str, required=True,
                        help="The specific id to get the path file (e.g., 'white shelf').")
    parsed_args = parser.parse_args()

    # Pass the database path to the node
    database_node = GetDatabase(parsed_args.database, parsed_args.target, parsed_args.id)

    rclpy.spin(database_node)
    database_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()