import cv2
import ai_utils
import os
from ultralytics import YOLO


MODEL_NAME = 'light_bulb_yolov8m'

CURRENT_EPOCHS = 300
CURRENT_IMGSZ = 640
CURRENT_BATCH = 8

start_model = "yolov8m.pt"
model_name = MODEL_NAME + '_' + str(CURRENT_IMGSZ)

train_folder_path = os.path.join(ai_utils.current_folder,'model_training')
model_pt_file = os.path.join(train_folder_path,start_model)
train_yaml_file = os.path.join(train_folder_path,"data_custom.yaml")



if __name__ == "__main__":
    cur_folder = os.getcwd()
    try:
        print("Changing to training folder:", train_folder_path)
        os.chdir(train_folder_path)  
        cur_folder = os.getcwd()
        print("Starting training in folder:", cur_folder)
    except Exception as e:
        print("Error: The specified training folder was not found: " + str(e))
    if cur_folder == train_folder_path:
        model = YOLO(model_pt_file)
        results = model.train(data=train_yaml_file, epochs=CURRENT_EPOCHS, imgsz=CURRENT_IMGSZ, batch=CURRENT_BATCH, name=model_name)
