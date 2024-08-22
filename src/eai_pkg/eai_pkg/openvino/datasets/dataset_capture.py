import cv2
import os


class DatasetsCapture:
    def __init__(self, cap_id=1, save_path="defaultSavePath", prefix_name="default"):
        self.save_path = save_path
        if not os.path.exists(self.save_path):
            os.makedirs(self.save_path)
        print(f"  ..DatasetCapture:Save path set to {self.save_path}")
        self.prefix_name = prefix_name
        print(f"  ..DatasetCapture:Prefix name set to {self.prefix_name}")
        self.cap = cv2.VideoCapture(cap_id)
        if not self.cap.isOpened():
            print(f"  ..DatasetCapture:Cannot open camera with id {cap_id}")
            exit(0)

    def set_save_path(self, path):
        self.save_path = path
        if not os.path.exists(path):
            os.makedirs(path)
        print(f"  ..DatasetCapture:Save path set to {self.save_path}")
        return self.save_path

    def set_prefix_name(self, name):
        self.prefix_name = name
        print(f"  ..DatasetCapture:Prefix name set to {self.prefix_name}")
        return self.prefix_name

    def mainloop(self, quit_char=ord('q'), save_char=13):
        print("  ..DatasetCapture:type in q to quit and enter to save")
        while True:
            ret, frame = self.cap.read()
            if not ret:
                print("  ..DatasetCapture:Cannot read frame")
                break
            cv2.imshow("DatasetCapture", frame)
            if cv2.waitKey(5) & 0xFF == quit_char:
                break
            elif cv2.waitKey(5) & 0xFF == save_char:  # enter
                num_existed_files = len(os.listdir(self.save_path))
                save_name = f"{self.prefix_name}_{num_existed_files}.jpg"
                cv2.imwrite(os.path.join(self.save_path, save_name), frame)
                print(f"  ..DatasetCapture:picture saved with name {save_name}")
        self.cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    dc = DatasetsCapture(cap_id=1, save_path="rgb", prefix_name="rgb")
    dc.mainloop()
    print("  ..DatasetCapture:Done")
    exit(0)