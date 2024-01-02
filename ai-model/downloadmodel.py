from roboflow import Roboflow

rf = Roboflow(api_key="aYdh91VIndPSOuVsPH5j")

project = rf.workspace().project("redprop")
dataset = project.version(2).download("yolov5")