import argparse
import chromadb
import uuid
import spacy
import numpy as np


def load_spacy_model():
    """Load the spaCy language model."""
    return spacy.load("en_core_web_md")


def embed_text(text, model):
    """Generate a text embedding using the spaCy model."""
    doc = model(text)
    return doc.vector.tolist()


def embed_text_with_weight(text, model):
    """Generate a text embedding with weight using the spaCy model."""
    doc = model(text)
    total_vector = np.zeros(300)
    desc_root_text = None
    desc_root = np.zeros(300)
    count = 0
    for chunk in doc.noun_chunks:
        desc_root_text = chunk.root.text
        desc_root = chunk.root.vector
    for token in doc:
        if token.pos_ == "ADJ":
            adj_word = token.vector
            total_vector += adj_word
            count += 1
        elif token.text == desc_root_text:
            desc_word = desc_root
            total_vector += 4*desc_word
            count += 4
        else:
            total_vector += 2*token.vector
            count += 2
    average_vector = total_vector/count

    return average_vector.tolist()
    # return doc.vector.tolist()


def store_embedding(text, embedding, metadatas, id, client, name):
    """Store the text embedding in a ChromaDB collection."""
    collection = client.get_or_create_collection(name=name)
    unique_id = str(id)
    collection.add(
        documents=[text],
        embeddings=[embedding],
        metadatas=[metadatas],
        ids=[unique_id]
    )


def querying_embeddings(embedding, client, name, num_results):
    """Querying text embedding"""
    collection = client.get_collection(name=name)
    results = collection.query(
        query_embeddings=[embedding],
        n_results=num_results
    )
    return results


def get_distance(text1, text2, model):
    """Get distance between two vectors"""
    # embedding_text_1 = embed_text(text1, model)
    # embedding_text_2 = embed_text(text2, model)
    embedding_text_1 = embed_text_with_weight(text1, model)
    embedding_text_2 = embed_text_with_weight(text2, model)
    distance = 0
    for count in range(len(embedding_text_1)):
        distance += (embedding_text_1[count] - embedding_text_2[count]) * \
            (embedding_text_1[count] - embedding_text_2[count])
    return round(distance/100, 3)


def delete_collection(client, name):
    """Delete a collection"""
    client.delete_collection(name=name)


def main():
    """Main function that handles the CLI."""
    parser = argparse.ArgumentParser(description="Text processing utility")
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("-t", "--text", help="Text to process")
    group.add_argument(
        "-f", "--file", type=argparse.FileType('r'), help="Text file to process")
    args = parser.parse_args()

    if args.text:
        text = args.text
    elif args.file:
        text = args.file.read()

    # Load the spaCy model
    spacy_model = load_spacy_model()

    # Generate the text embedding
    embedding_1 = embed_text(text, spacy_model)

    # Set up ChromaDB and store the embedding
    client_test = chromadb.PersistentClient(path="./database")
    metadatas = [{"color": "brown"}]
    id = ["1"]
    doc_text = [text]

    store_embedding(doc_text, embedding_1, metadatas, id,
                    client_test, name="arena-model")

    print(f"Storing text: {text}")


if __name__ == "__main__":
    main()
