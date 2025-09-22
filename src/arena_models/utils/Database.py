import argparse
import functools
import typing
import uuid

import chromadb
import chromadb.api.models.Collection
import numpy as np
import spacy

from ..impl import Annotation

TextOrEmbedding = typing.Text | typing.List[float]


def as_embedding(fn: typing.Callable[[typing.Text], typing.List[float]], v: TextOrEmbedding, /) -> typing.List[float]:
    if isinstance(v, str):
        return fn(v)
    return v


class Database:
    def __init__(self, path: str, model: typing.Optional[str] = None, *, embedder: typing.Optional[typing.Callable[[str], list[float]]] = None):
        if model is None:
            model = "en_core_web_md"

        self._client = chromadb.PersistentClient(path=path)
        self._model = spacy.load(model)

        if embedder is None:
            embedder = self.embed
        self.as_embedding = functools.partial(as_embedding, embedder)

    def collection(self, name) -> chromadb.api.models.Collection.Collection:
        """Get or create a ChromaDB collection."""
        return self._client.get_or_create_collection(name=name)

    def store(self, collection: str, annotation: Annotation):
        """Store the text embedding in a ChromaDB collection."""
        unique_id = uuid.uuid4().hex
        self.collection(collection).add(
            documents=[annotation.as_text],
            embeddings=[self.as_embedding(annotation.as_text)],
            metadatas=[annotation.as_metadata],
            ids=[unique_id]
        )

    def query(self, collection: str, embedding: TextOrEmbedding, num_results: int = 1):
        """Query the collection for similar embeddings."""
        results = self.collection(collection).query(
            query_embeddings=[self.as_embedding(embedding)],
            n_results=num_results
        )
        return results

    def embed_raw(self, text: str) -> list[float]:
        """Generate a text embedding using the spaCy model."""
        doc = self._model(text)
        return doc.vector.tolist()

    def embed(self, text: str) -> list[float]:
        """Generate a text embedding."""
        if not text.strip():
            return np.zeros(self._model.vocab.vectors.shape[1]).tolist()

        doc = self._model(text)
        tokens = [token for token in doc if token.has_vector and not token.is_stop and not token.is_punct]

        if not tokens:
            return doc.vector.tolist()

        total_vector: np.ndarray = np.zeros(self._model.vocab.vectors.shape[1])
        total_weight = 0

        for i, token in enumerate(tokens):
            if token.pos_ == "NOUN":
                base_weight = 8
            elif token.pos_ == "ADJ":
                base_weight = 5
            else:
                base_weight = 2

            if len(tokens) > 1:
                position_multiplier = 1 + 0.5 * (i / (len(tokens) - 1))
            else:
                position_multiplier = 1

            final_weight = base_weight * position_multiplier
            total_vector += token.vector * final_weight
            total_weight += final_weight

        total_vector /= total_weight
        return total_vector.tolist()

    def get_distance(self, text1: TextOrEmbedding, text2: TextOrEmbedding):
        """Get distance between two vectors"""
        embedding_text_1 = self.as_embedding(text1)
        embedding_text_2 = self.as_embedding(text2)
        distance = 0
        for i, val1 in enumerate(embedding_text_1):
            val2 = embedding_text_2[i]
            distance += (val1 - val2) ** 2
        return round(distance / 100, 3)
