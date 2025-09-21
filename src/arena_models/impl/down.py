import gdown
import zipfile
import os
import logging

logger = logging.getLogger()


class DownDatabase:
    def __init__(self):
        shared_link = 'https://drive.google.com/file/d/17P3VIh4dVY0lu0ftonYckU3gpMJe_rp0/view?usp=sharing'
        destination = 'Dataset-arena-models.zip'  # Change the file name and extension as needed
        self.download_file(shared_link, destination)
        self.unzip_file(destination)
        self.cleanup_zipfile(destination)
        raise SystemExit

    def download_file(self, shared_link, destination):
        # Extract the file ID from the shared link
        file_id = shared_link.split('/')[-2]

        # Create the download URL
        download_url = f'https://drive.google.com/uc?id={file_id}'

        gdown.download(download_url, destination, quiet=False)

        logger.info("Downloaded successfully!")

    def unzip_file(self, zip_file):
        with zipfile.ZipFile(zip_file, 'r') as zip_ref:
            zip_ref.extractall()  # Extract to the current directory or specify a path
        logger.info("Unzipped successfully!")

    def cleanup_zipfile(self, zip_file):
        if os.path.exists(zip_file):
            os.remove(zip_file)
            logger.info("Cleaned up the ZIP file!")


def download_database():
    """Download the database from the remote source."""
    DownDatabase()
