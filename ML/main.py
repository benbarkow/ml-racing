import onnx
import onnxruntime as ort
import numpy as np
from onnx import checker
import onnxruntime as ort
import numpy as np
import cv2

onnx_path = "models/weights/resnet18-avgpool.onnx"
resnet = onnx.load(onnx_path)

resnet = onnx.version_converter.convert_version(resnet, 18)
resnet.ir_version = 8
checker.check_model(resnet)

from PIL import Image

def get_image(path, show=False):
    with Image.open(path) as img:
        img = np.array(img.convert('RGB'))
    return img


def preprocess(img):
    img = img / 255.
    img = cv2.resize(img, (256, 256))
    h, w = img.shape[0], img.shape[1]
    y0 = (h - 224) // 2
    x0 = (w - 224) // 2
    img = img[y0 : y0+224, x0 : x0+224, :]
    img = (img - [0.485, 0.456, 0.406]) / [0.229, 0.224, 0.225]
    img = np.transpose(img, axes=[2, 0, 1])
    img = img.astype(np.float32)
    img = np.expand_dims(img, axis=0)
    return img


def predict(path):
    img = get_image(path, show=True)
    img = preprocess(img)
    ort_inputs = {session.get_inputs()[0].name: img}
    preds = session.run(None, ort_inputs)[0]
    return preds


session = ort.InferenceSession(resnet.SerializeToString())

preds = predict("test_images/16.png")
observation = np.concatenate((preds, [[0, 0]]), axis=1).astype(np.float32)
print(observation.shape)

onnx_path = "my_ppo_model.onnx"
onnx_model = onnx.load(onnx_path)
onnx.checker.check_model(onnx_model)

ort_sess = ort.InferenceSession(onnx_path)
action, value = ort_sess.run(None, {"input": observation})
print(action)