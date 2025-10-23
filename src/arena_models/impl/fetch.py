import os

from google.cloud import storage

from arena_models.impl import ANNOTATION_NAME
from arena_models.utils.logging import format_file_size, get_logger, get_manager


class Bucket:
    def __init__(self, bucket: str):
        client = storage.Client.create_anonymous_client()
        self.bucket = client.bucket(bucket)

    def listdir(self, prefix: str):
        return self.bucket.list_blobs(prefix=prefix)

    def asset_exists(self, asset: str) -> bool:
        if not os.path.basename(asset) == ANNOTATION_NAME:
            asset = os.path.join(asset, ANNOTATION_NAME)
        blob = self.bucket.blob(asset)
        return blob.exists()


def asset_exists(bucket: str, asset: str) -> bool:
    """Check if an asset exists in the Google Cloud Storage bucket.

    Args:
        bucket: The Google Cloud Storage bucket name
        asset: The asset path to check
    Returns:
        True if the asset exists, False otherwise
    """
    bucket_obj = Bucket(bucket)
    return bucket_obj.asset_exists(asset)


def fetch_database(bucket: str, bucket_path: str, destination: str, relative_path: str = "", annotations: bool = True) -> None:
    """Download a specific path from Google Cloud Storage bucket.

    Args:
        bucket: The Google Cloud Storage bucket name to download from
        bucket_path: The path inside the bucket to download
        destination: The local directory path to download to
        relative_path: The relative path within destination to save the files (optional)
    """
    logger = get_logger('fetch')
    try:
        bucket_obj = Bucket(bucket)

        bucket_path = bucket_path.strip('/')
        if bucket_path and not bucket_path.endswith('/'):
            bucket_path += '/'

        full_destination = os.path.join(destination, relative_path) if relative_path else destination
        os.makedirs(full_destination, exist_ok=True)

        logger.info("Scanning bucket path '%s' for files...", bucket_path)
        blobs_list = bucket_obj.listdir(bucket_path)
        file_blobs = [blob for blob in blobs_list if not blob.name.endswith('/') and (annotations or not os.path.basename(blob.name) == ANNOTATION_NAME)]
        total_files = len(file_blobs)
        total_size = sum(blob.size or 0 for blob in file_blobs)

        if total_files == 0:
            logger.warning("No files found at path '%s' in bucket '%s'", bucket_path, bucket)
            return

        logger.info("Found %d files (%s) to download", total_files, format_file_size(total_size))

        downloaded_count = 0
        downloaded_size = 0
        skipped_size = 0

        with get_manager() as manager:
            status_bar = manager.status_bar(
                status_format='Downloading: {current_file}{fill}',
                current_file='Initializing...',
                color='cyan'
            )

            data_progress = manager.counter(
                total=total_size,
                desc=f"[{1:>{len(str(total_files))}d}/{total_files}]",
                color="blue",
                counter_format='{desc}{desc_pad}{count_formatted}/{total_formatted} [{elapsed}, {rate_formatted}/s]{fill}',
                bar_format='{desc}{desc_pad}{percentage:3.0f}%|{bar}| {count_formatted}/{total_formatted} [{elapsed}<{eta}, {rate_formatted}/s]',
                fields={
                    'count_formatted': format_file_size(0),
                    'total_formatted': format_file_size(total_size),
                    'rate_formatted': format_file_size(0)
                }
            )

            for blob in file_blobs:
                blob_size = blob.size or 0
                size_str = format_file_size(blob_size)

                status_bar.update(current_file=f"{blob.name} ({size_str})")

                relative_blob_path = blob.name[len(bucket_path):] if bucket_path else blob.name
                local_file_path = os.path.join(full_destination, relative_blob_path)
                os.makedirs(os.path.dirname(local_file_path), exist_ok=True)

                # Skip if file already exists and has the same size
                if os.path.exists(local_file_path):
                    local_file_size = os.path.getsize(local_file_path)
                    if local_file_size == blob_size:
                        logger.info("Skipping '%s' - already exists with correct size", blob.name)
                        downloaded_count += 1
                        skipped_size += blob_size

                        new_total = total_size - skipped_size
                        data_progress.total = new_total

                        elapsed_time = data_progress.elapsed
                        current_rate = downloaded_size / elapsed_time if elapsed_time > 0 else 0

                        data_progress.desc = f"[{downloaded_count:>{len(str(total_files))}d}/{total_files}]"
                        data_progress.update(
                            0,  # Don't add to downloaded count for skipped files
                            count_formatted=format_file_size(downloaded_size),
                            total_formatted=format_file_size(new_total),
                            rate_formatted=format_file_size(current_rate)
                        )
                        continue

                blob.download_to_filename(local_file_path)

                downloaded_count += 1
                downloaded_size += blob_size

                # Calculate rate as count/elapsed and format it
                elapsed_time = data_progress.elapsed
                current_rate = downloaded_size / elapsed_time if elapsed_time > 0 else 0

                data_progress.desc = f"[{downloaded_count:>{len(str(total_files))}d}/{total_files}]"
                data_progress.update(
                    blob_size,
                    count_formatted=format_file_size(downloaded_size),
                    total_formatted=format_file_size(total_size - skipped_size),
                    rate_formatted=format_file_size(current_rate)
                )

            status_bar.update(current_file='Complete!')

        logger.info(
            "Downloaded %d files (%s) successfully to: %s",
            downloaded_count,
            format_file_size(downloaded_size),
            full_destination
        )

    except Exception as e:
        logger.error("Fetch error: %s", e)
        raise
