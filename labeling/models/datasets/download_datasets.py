"""
Download `product_classification-1` dataset from Roboflow.

Usage:
    export ROBOFLOW_API_KEY=your_key_here
    python3 download_roboflow.py

The API key is read from the ROBOFLOW_API_KEY environment variable.
NEVER commit your real key to the repository.
"""

from roboflow import Roboflow
rf = Roboflow(api_key="78XdufTTXYJ9iMC5BBWn")
project = rf.workspace("s-workspace-orwiy").project("product_classification-vo9mz")
version = project.version(10)
dataset = version.download("yolov8")
                
