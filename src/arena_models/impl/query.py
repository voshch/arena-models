import chromadb
from text_processing.language_processing import load_spacy_model, embed_text, store_embedding, embed_text_with_weight, querying_embeddings
import os


def query_database(database_path, name_database, target_path, query_target):
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

    raise SystemExit
