# 1.模型转换器
from converters.onnx2ir import ONNX2IR
# 2.模型优化器(not yet)
# 3.基于IR的推理器
from inferers.infer_normal import NormalInferer
# 4.推理解析及可视化
from analyzers.analyzer_yolo import AnalyzerYolo


input_img_path = "blocks_t.png"
# 1.convert:将yolo导出的onnx模型转换为ir模型
# converter = ONNX2IR(onnx_model_path="blocks.onnx",
#                     ir_model_path="blocks.xml")
# converter.convert()

# 2.optimize:优化ir模型(可选）
pass

# 3.infer:推理ir模型
inferer = NormalInferer(ir_model_path="blocks.xml", imgsz=(640, 640), device="CPU")
infer1 = inferer.infer_once(input_img_path)

# 4.analyze:解析推理结果并可视化
analyzer_yolo = AnalyzerYolo(classes={0: 'r', 1: "p", 2: 'g', 3:'b', 4:'y'}, imgsz=(640, 640))
pred = analyzer_yolo.infer_analyse(infer1)
analyzer_yolo.pred_visualize(pred, origin_pic=input_img_path, res_pic_path="result.jpg")
