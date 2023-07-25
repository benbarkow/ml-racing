import onnx
from onnx import parser
from onnx import checker
import onnxruntime as ort
import numpy as np

onnx_path = "rightNormalized.onnx"
net = onnx.load(onnx_path)

net = onnx.version_converter.convert_version(net, 18)
net.ir_version = 8
checker.check_model(net)
