import os
import random
import shutil

# 原始数据存放路径
data_dir = "/datasets/rgb\\"
images_dir = os.path.join(data_dir, "images")
labels_dir = os.path.join(data_dir, "labels")

# 划分后的训练集和验证集路径
train_dir = "../rgb/train"
train_images_dir = os.path.join(train_dir, "images")
train_labels_dir = os.path.join(train_dir, "labels")

val_dir = "../rgb/val"
val_images_dir = os.path.join(val_dir, "images")
val_labels_dir = os.path.join(val_dir, "labels")

# 创建训练集和验证集目录
os.makedirs(train_images_dir, exist_ok=True)
os.makedirs(train_labels_dir, exist_ok=True)
os.makedirs(val_images_dir, exist_ok=True)
os.makedirs(val_labels_dir, exist_ok=True)

# 获取所有图片文件名
image_files = os.listdir(images_dir)

# 随机打乱文件列表
random.shuffle(image_files)

# 计算划分的数量
total_images = len(image_files)
train_ratio = 0.9
num_train = int(total_images * train_ratio)

# 将图片和标签文件划分到训练集和验证集
train_file_list = []
val_file_list = []

for i, image_file in enumerate(image_files):
    label_file = image_file.replace(".jpg", ".txt")

    if i < num_train:
        # 划分到训练集
        shutil.copy(os.path.join(images_dir, image_file), os.path.join(train_images_dir, image_file))
        shutil.copy(os.path.join(labels_dir, label_file), os.path.join(train_labels_dir, label_file))
        train_file_list.append(os.path.join(data_dir, "train", "images", image_file))
    else:
        # 划分到验证集
        shutil.copy(os.path.join(images_dir, image_file), os.path.join(val_images_dir, image_file))
        shutil.copy(os.path.join(labels_dir, label_file), os.path.join(val_labels_dir, label_file))
        val_file_list.append(os.path.join(data_dir, "val", "images", image_file))

# 创建train.txt和val.txt文件
with open(os.path.join(data_dir, "train.txt"), "w") as train_txt_file:
    lines = "\n".join(train_file_list)
    train_txt_file.write(lines)

with open(os.path.join(data_dir, "val.txt"), "w") as val_txt_file:
    val_txt_file.write("\n".join(val_file_list))

print(f"划分完成，训练集包含 {num_train} 个样本，验证集包含 {total_images - num_train} 个样本。")
