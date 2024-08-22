import openvino as ov


class ONNX2IR:
    def __init__(self, onnx_model_path="yolov5/runs/train/exp16/weights/best.onnx",
                 ir_model_path="exported_onnx_model.xml"):
        self.onnx_model_path = onnx_model_path
        self.ir_model_path = ir_model_path

    def convert(self):
        core = ov.Core()
        model_onnx = core.read_model(model=self.onnx_model_path)
        ov.save_model(model_onnx, output_model=self.ir_model_path)
