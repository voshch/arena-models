import functools
import typing
import uuid
from pathlib import Path

import chromadb
import chromadb.api.models.Collection
import numpy as np
import spacy

import re

from ..impl import Annotation

TextOrEmbedding = typing.Text | typing.List[float]


def as_embedding(fn: typing.Callable[[typing.Text], typing.List[float]], v: TextOrEmbedding, /) -> typing.List[float]:
    if isinstance(v, str):
        return fn(v)
    return v


class Database:
    def __init__(self, path: Path, model: typing.Optional[str] = None, *, embedder: typing.Optional[typing.Callable[[str], list[float]]] = None):
        if model is None:
            model = "en_core_web_md"

        self._client = chromadb.PersistentClient(path=str(path))
        self._model = spacy.load(model)

        if embedder is None:
            embedder = self.embed
        self.as_embedding = functools.partial(as_embedding, embedder)
    
    def _normalize_and_split(self, text: str) -> list[str]:
        """Normalize, split underscores/hyphens/camelCase and return simple words."""
        if not text:
            return []
        s = text.replace("_", " ").replace("-", " ")
        # split camelCase -> add spaces between lowerUpper boundaries
        s = re.sub(r"([a-z0-9])([A-Z])", r"\1 \2", s)
        # tokenize by whitespace and punctuation
        parts = re.findall(r"\w+", s.lower())
        return parts
    
    def tokenize(self, text: str) -> list[str]:
        """Return normalized keyword lemmas (nouns preferred)."""
        parts = self._normalize_and_split(text)
        doc = self._model(" ".join(parts))
        tokens = []
        for i, token in enumerate(doc):
            if not token.is_alpha or token.is_stop:
                continue
            lemma = token.lemma_.lower()
            # prefer nouns by duplicating them once to indicate importance
            if token.pos_ == "NOUN":
                tokens.append(lemma)
            tokens.append(lemma)
        # preserve order, unique
        seen = set()
        result = []
        for t in tokens:
            if t not in seen:
                seen.add(t)
                result.append(t)
        return result

    def collection(self, name) -> chromadb.api.models.Collection.Collection:
        """Get or create a ChromaDB collection."""
        return self._client.get_or_create_collection(name=name)

    def store(self, collection: str, annotation: Annotation):
        """Store the text embedding in a ChromaDB collection."""
        unique_id = uuid.uuid4().hex
        # self.collection(collection).add(
        #     documents=[annotation.as_text],
        #     embeddings=[self.as_embedding(annotation.as_text)],
        #     metadatas=[annotation.as_metadata],
        #     ids=[unique_id]
        # )
        metadata = dict(annotation.as_metadata)
        # add tokens for exact matching fallback; include name/tags if present
        # annotation.as_text already contains desc/material/color/tags, but include name split for "Bed_Side_Table"
        metadata_tokens = self.tokenize(annotation.as_text)
        if metadata.get("name"):
            metadata_tokens += self._normalize_and_split(metadata["name"])
        if metadata.get("tags"):
            # tags might be list or comma string
            tags = metadata["tags"] if isinstance(metadata["tags"], list) else str(metadata["tags"]).split(",")
            for t in tags:
                metadata_tokens += self._normalize_and_split(t)
        # keep unique-order tokens
        seen = set()
        tokens = []
        for t in metadata_tokens:
            if t and t not in seen:
                seen.add(t)
                tokens.append(t)
        metadata["tokens"] = tokens

        self.collection(collection).add(
            documents=[annotation.as_text],
            embeddings=[self.as_embedding(annotation.as_text)],
            metadatas=[metadata],
            ids=[unique_id]
        )

    def list_all(self, collection: str):
        """List all paths in the collection."""
        return self.collection(collection).get()

    def query(self, collection: str, embedding: TextOrEmbedding, num_results: int = 1):
        """Query the collection for similar embeddings."""
        # results = self.collection(collection).query(
        #     query_embeddings=[self.as_embedding(embedding)],
        #     n_results=num_results
        # )
        # return results
        # If the query is a short/single token string, try exact-token fallback first
        if isinstance(embedding, str):
            q = embedding.strip().lower()
            # single-word queries get exact-token lookup (fast and precise)
            if q and len(q.split()) == 1:
                try:
                    all_docs = self.collection(collection).get()
                    ids = []
                    metadatas = []
                    documents = []
                    embeddings = []

                    candidates: list[tuple[int, int]] = []  # (score, index)
                    for i, m in enumerate(all_docs.get("metadatas", [])):
                        tokens = [t.lower() for t in (m.get("tokens") or [])]
                        name_raw = (m.get("name") or "").strip()
                        name = name_raw.lower()
                        name_parts = self._normalize_and_split(name_raw)
                        tags = m.get("tags") or []
                        if isinstance(tags, str):
                            tags = [t.strip().lower() for t in tags.split(",")]
                        else:
                            tags = [str(t).strip().lower() for t in tags]

                        score = 0
                        # highest: exact name match
                        if name == q:
                            score += 200
                        # name begins with query (e.g. "bed_a", "bed b") -> strong signal
                        if name_parts and name_parts[0] == q:
                            # prefer short suffixes (Bed_A/B) over long ones (Bed_Side_Table)
                            suffix_penalty = max(0, len(name_parts) - 1) * 10
                            score += 120 - suffix_penalty
                        # token-level match (from description/tags split/lemmatized tokens)
                        if q in tokens:
                            score += 60
                        # tag match is weaker
                        if q in tags:
                            score += 20

                        if score > 0:
                            candidates.append((score, i))

                    # sort by score desc, stable by index for ties
                    candidates.sort(key=lambda x: (-x[0], x[1]))

                    for score, i in candidates:
                        if len(ids) >= num_results:
                            break
                        ids.append(all_docs["ids"][i])
                        metadatas.append(all_docs["metadatas"][i])
                        documents.append(all_docs["documents"][i])
                        embeddings.append(all_docs.get("embeddings", [None])[i])

                    if ids:
                        return {"ids": ids, "metadatas": metadatas, "documents": documents, "embeddings": embeddings}
                except Exception:
                    # If exact fallback fails, continue to embedding search
                    pass

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

        # Tunable parameters
        noun_boost = 2.0          # multiply noun base_weight by this
        last_token_bonus = 0.7    # extra fractional boost applied to the final token

        for i, token in enumerate(tokens):
            if token.pos_ == "NOUN":
                base_weight = 8
            elif token.pos_ == "ADJ":
                base_weight = 5
            else:
                base_weight = 2
            
            # boost nouns
            if token.pos_ == "NOUN":
                base_weight *= noun_boost


            if len(tokens) > 1:
                position_multiplier = 1 + 0.5 * (i / (len(tokens) - 1))
            else:
                position_multiplier = 1
            
            # extra boost for the last token (makes trailing words more important)
            end_multiplier = 1.0 + (last_token_bonus if i == (len(tokens) - 1) else 0.0)

            final_weight = base_weight * position_multiplier
            final_weight *= end_multiplier
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
