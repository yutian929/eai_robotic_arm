import xml.etree.ElementTree as ET
import os
import random
import shutil
import yaml

class Xml2Yolo:
    def __init__(self, imgs_folder_path: str, xml_folder_path: str, yolo_folder_path: str, classes: list):
        self.imgs_folder_path = imgs_folder_path
        self.xml_folder_path = xml_folder_path
        self.yolo_folder_path = yolo_folder_path
        self.classes = classes

        if os.path.exists(self.imgs_folder_path):
            self.imgs_folder_abspath = os.path.abspath(self.imgs_folder_path)
            print(f"  ..Xml2Yolo:abspath of imgs_folder_path is {self.imgs_folder_abspath}")
        else:
            raise FileNotFoundError("imgs_folder_path is not existed")

        if os.path.exists(self.xml_folder_path):
            self.xml_folder_abspath = os.path.abspath(self.xml_folder_path)
            print(f"  ..Xml2Yolo:abspath of xml_folder_path is {self.xml_folder_abspath}")
        else:
            raise FileNotFoundError("xml_folder_path is not existed")

        if os.path.exists(self.yolo_folder_path):
            self.yolo_folder_abspath = os.path.abspath(self.yolo_folder_path)
        else:
            os.mkdir(self.yolo_folder_path)
            self.yolo_folder_abspath = os.path.abspath(self.yolo_folder_path)
        print(f"  ..Xml2Yolo:abspath of yolo_folder_path is {self.yolo_folder_abspath}")

        self.root_folder_abspath = os.path.dirname(self.imgs_folder_abspath)

    def convert_bbox_once(self, size, box):
        x_center = (box[0] + box[1]) / 2.0
        y_center = (box[2] + box[3]) / 2.0
        x = x_center / size[0]
        y = y_center / size[1]
        w = (box[1] - box[0]) / size[0]
        h = (box[3] - box[2]) / size[1]
        return x, y, w, h

    def convert(self):
        xml_files = os.listdir(self.xml_folder_abspath)
        for xml_name in xml_files:
            # print(xml_name)
            xml_file = os.path.join(self.xml_folder_abspath, xml_name)
            out_txt_path = os.path.join(self.yolo_folder_path, xml_name.split('.')[0] + '.txt')
            out_txt_f = open(out_txt_path, 'w')
            tree = ET.parse(xml_file)
            root = tree.getroot()
            size = root.find('size')
            w = int(size.find('width').text)
            h = int(size.find('height').text)

            for obj in root.iter('object'):
                difficult = obj.find('difficult').text
                cls = obj.find('name').text
                if cls not in self.classes or int(difficult) == 1:
                    continue
                cls_id = self.classes.index(cls)
                xmlbox = obj.find('bndbox')
                b = (float(xmlbox.find('xmin').text), float(xmlbox.find('xmax').text), float(xmlbox.find('ymin').text),
                     float(xmlbox.find('ymax').text))
                # print(w, h, b)
                bb = self.convert_bbox_once((w, h), b)
                out_txt_f.write(str(cls_id) + " " + " ".join([str(a) for a in bb]) + '\n')
        print("  ..Xml2Yolo:convert finished")

    def split_yolo_train_val(self, ratio=0.8):
        # train_val_root_folder = os.path.dirname(self.imgs_folder_abspath)
        train_images_dir = os.path.join(self.root_folder_abspath, "train", "images")
        train_labels_dir = os.path.join(self.root_folder_abspath, "train", "labels")
        val_images_dir = os.path.join(self.root_folder_abspath, "val", "images")
        val_labels_dir = os.path.join(self.root_folder_abspath, "val", "labels")
        os.makedirs(train_images_dir, exist_ok=True)
        os.makedirs(train_labels_dir, exist_ok=True)
        os.makedirs(val_images_dir, exist_ok=True)
        os.makedirs(val_labels_dir, exist_ok=True)

        image_files_names = os.listdir(self.imgs_folder_abspath)
        random.shuffle(image_files_names)
        total_images = len(image_files_names)
        num_train = int(total_images * ratio)

        train_file_list = []
        val_file_list = []

        for i, image_file in enumerate(image_files_names):
            label_file = image_file.replace(".jpg", ".txt")
            if i < num_train:
                # 划分到训练集
                shutil.copy(os.path.join(self.imgs_folder_abspath, image_file), os.path.join(train_images_dir, image_file))
                shutil.copy(os.path.join(self.yolo_folder_abspath, label_file), os.path.join(train_labels_dir, label_file))
                train_file_list.append(os.path.join(self.root_folder_abspath, "train", "images", image_file))
            else:
                # 划分到验证集
                shutil.copy(os.path.join(self.imgs_folder_abspath, image_file), os.path.join(val_images_dir, image_file))
                shutil.copy(os.path.join(self.yolo_folder_abspath, label_file), os.path.join(val_labels_dir, label_file))
                val_file_list.append(os.path.join(self.root_folder_abspath, "val", "images", image_file))

            # 创建train.txt和val.txt文件
            with open(os.path.join(self.root_folder_abspath, "train.txt"), "w") as train_txt_file:
                lines = "\n".join(train_file_list)
                train_txt_file.write(lines)

            with open(os.path.join(self.root_folder_abspath, "val.txt"), "w") as val_txt_file:
                val_txt_file.write("\n".join(val_file_list))

        print(f"  ..Xml2Yolo:split train val sets finished")

    def create_yaml(self, yaml_name='default.yaml'):
        class_index = list(range(len(self.classes)))
        class_dic = dict(zip(class_index, self.classes))
        yaml_data = {
            'path': self.root_folder_abspath,
            'train': 'train.txt',
            'val': 'val.txt',

            'names': class_dic
        }
        yaml_path = os.path.join(self.root_folder_abspath, yaml_name)
        with open(yaml_path, 'w', encoding='utf-8') as f:
            yaml.dump(data=yaml_data, stream=f, allow_unicode=True)
        print(f"  ..Xml2Yolo:create yaml file {yaml_name} finished")


xml2yolo = Xml2Yolo(imgs_folder_path='rgb3/images', xml_folder_path='rgb3/annotations', yolo_folder_path='rgb3/labels',
                    classes=['r', 'g', 'b'])
xml2yolo.convert()
xml2yolo.split_yolo_train_val()
xml2yolo.create_yaml(yaml_name='rgb3.yaml')


