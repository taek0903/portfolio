from roboflow import Roboflow

rf = Roboflow(api_key="78XdufTTXYJ9iMC5BBWn")
project = rf.workspace("s-workspace-orwiy").project("product_classification-vo9mz")
version = project.version(10)

# location 파라미터로 다른 폴더명 지정
dataset = version.download("florence2-od", location="./product_classification-10-florence2")