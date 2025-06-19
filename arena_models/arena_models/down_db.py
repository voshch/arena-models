import rclpy
from rclpy.node import Node
import requests
import gdown
import zipfile
import os

class DownDatabase(Node):
    def __init__(self):
        super().__init__('down_db_node')
        shared_link = 'https://drive.google.com/file/d/1YwoGqwcEAKoc1OEELblp-7XusLBO3iEU/view?usp=sharing'
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

        self.get_logger().info("Downloaded successfully!")

    def unzip_file(self, zip_file):
        with zipfile.ZipFile(zip_file, 'r') as zip_ref:
            zip_ref.extractall()  # Extract to the current directory or specify a path
        self.get_logger().info("Unzipped successfully!")

    def cleanup_zipfile(self, zip_file):
        if os.path.exists(zip_file):
            os.remove(zip_file)
            self.get_logger().info("Cleaned up the ZIP file!")

def main(args=None):
    rclpy.init(args=args)
    node = DownDatabase()
    rclpy.spin_once(node)
    node.destroy_node()
    rclpy.shutdown()
    
    

if __name__ == '__main__':
    main()