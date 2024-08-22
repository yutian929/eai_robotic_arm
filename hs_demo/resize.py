from PIL import Image

def crop_and_resize(image_path, output_size, vertical_shift):
    try:
        # 打开图像文件
        image = Image.open(image_path)
        
        # 获取图像的宽度和高度
        width, height = image.size
        
        # 计算要裁剪的区域的左上角和右下角坐标
        left = (width - output_size[0]) / 2
        top = ((height - output_size[1]) / 2) + vertical_shift
        right = (width + output_size[0]) / 2
        bottom = (height + output_size[1]) / 2 + vertical_shift
        
        # 裁剪图像
        cropped_image = image.crop((left, top, right, bottom))
        
        # 调整图像大小
        resized_image = cropped_image.resize(output_size)
        
        # 显示图像
        resized_image.show()
        
        # 保存图像
        resized_image.save("resized_image.jpg")

    except Exception as e:
        print(f"ERROR --- {e}")

# 图像路径
image_path = "vlm_color_image.jpg"

# 输出大小
output_size = (300, 300)

# 垂直移动量
vertical_shift = 50  # 向下移动50像素

# 调用函数
crop_and_resize(image_path, output_size, vertical_shift)
