import chromadb
from text_processing.language_processing import load_spacy_model, embed_text, store_embedding, embed_text_with_weight, querying_embeddings
import argparse
import os
import shutil

import logging
logger = logging.getLogger()


def copy_folder_contents(src_folder, dest_folder):
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


def merge_database(database1, database2, outputdb):
    # Copy contents from both folders
    copy_folder_contents(database1, outputdb)
    copy_folder_contents(database2, outputdb)

    logger.info(f"Databases merged successfully into: {outputdb}")
    raise SystemExit
