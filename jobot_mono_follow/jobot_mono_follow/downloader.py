import os
import urllib.request

class ModelDownloader:
    DOWNLOAD_URLS = [
        "https://archive.spacemit.com/ros2/brdk_models/detection/yolov8n.q.onnx"
    ]
    
    LOCAL_ROOT = os.path.expanduser("~/.brdk_models")

    MODEL_PATHS = [
        "jobot_mono_follow/yolov8n.q.onnx"
    ]
        

    def __init__(self):
        all_exist = all(
        os.path.exists(os.path.join(self.LOCAL_ROOT, path)) 
        for path in self.MODEL_PATHS
        )

        if all_exist:
            print("All model files already exist and do not need to be downloaded")
            return

        print(f"The model downloader is being initialized. Target directory: {self.LOCAL_ROOT}")
        os.makedirs(self.LOCAL_ROOT, exist_ok=True)

        for relative_path, download_url in zip(self.MODEL_PATHS, self.DOWNLOAD_URLS):
            self._download_model(relative_path, download_url)

    def _download_model(self, relative_path, download_url):
        local_path = os.path.join(self.LOCAL_ROOT, relative_path)
        if os.path.exists(local_path):
            print(f"Existing: {relative_path}, skip download")
            return

        os.makedirs(os.path.dirname(local_path), exist_ok=True)

        print(f"downloading: {download_url}")
        try:
            urllib.request.urlretrieve(download_url, local_path)
            print(f"Successful download: {relative_path}")
        except Exception as e:
            print(f"Download failed: {relative_path}, error: {e}")
