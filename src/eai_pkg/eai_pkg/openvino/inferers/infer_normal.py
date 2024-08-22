import openvino as ov
import cv2
import numpy as np


class NormalInferer:
    def __init__(self, ir_model_path, imgsz, device):
        self.ir_model_path = ir_model_path
        self.imgsz = imgsz
        self.device = device

    def preprocess_once(self, input_img):
        if isinstance(input_img, str):
            img = cv2.imread(input_img)
        else:
            img = input_img
        # preprocess
        img = cv2.resize(img, self.imgsz)
        im = img.transpose((2, 0, 1))[::-1]  # HWC to CHW, BGR to RGB
        im = np.ascontiguousarray(im)  # contiguous
        im = im.astype(np.float32)
        im /= 255  # 0 - 255 to 0.0 - 1.0
        if len(im.shape) == 3:
            im = im[None]  # expand for batch dim
        return im

    def infer_once(self, input_img):
        im = self.preprocess_once(input_img)
        input_tensor = ov.Tensor(array=im)

        core = ov.Core()
        compiled_model_ir = core.compile_model(self.ir_model_path, self.device)
        infer_request = compiled_model_ir.create_infer_request()

        # Set input tensor for model with one input
        infer_request.set_input_tensor(input_tensor)
        infer_request.start_async()
        infer_request.wait()

        # Get output tensor for model with one output
        output = infer_request.get_output_tensor()
        output_buffer = output.data

        # output_buffer[] - accessing output tensor data
        infer_1 = [output_buffer]

        return infer_1
