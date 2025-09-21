import chromadb
from text_processing.language_processing import load_spacy_model
import os

import logging
logger = logging.getLogger()


def get_database(database_path, database_name, target_path, id):
    client = chromadb.PersistentClient(path=f"{database_path}")
    spacy_model = load_spacy_model()
    collection = client.get_collection(f"{database_name}")
    result = collection.get(ids=[f'{id}'])
    prefix_path = f'{target_path}'
    filename = 'config_result_file.txt'
    full_path = os.path.join(prefix_path, filename)
    with open(full_path, 'w') as file:
        file.write(f"result:{result}\n")
    logger.info(f"file: {result}")
    raise SystemExit
