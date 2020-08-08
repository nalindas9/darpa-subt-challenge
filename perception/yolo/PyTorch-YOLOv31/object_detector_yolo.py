"""
Object detector trained on Darknet-YOLOv3

Reference:
1. https://github.com/eriklindernoren/PyTorch-YOLOv3

Authors:
Nalin Das (nalindas9@gmail.com)
Graduate Student pursuing Masters in Robotics,
University of Maryland, College Park
"""

import sys
sys.path.append('/opt/ros/melodic/lib/python2.7/dist-packages')
from utils.utils import *
import torch
import argparse
from models import *
from torchvision import transforms
from torch.autograd import Variable
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np
sys.path.remove('/opt/ros/melodic/lib/python2.7/dist-packages')
from cv_bridge import CvBridge
sys.path.append('/opt/ros/melodic/lib/python2.7/dist-packages')
from std_msgs.msg import String
from sensor_msgs.msg import Image as Imagemsg
import rospy
import cv2
from PIL import Image

im_pil = None

# Function to initialize the model
def init_model():
    # Parse script arguments
    parser = argparse.ArgumentParser()
    parser.add_argument("--model_def", type=str, default = "config/yolov3-custom.cfg", help="path to model definition file")
    parser.add_argument("--img_size", type=int, default = 416, help="shape of i/p image")
    parser.add_argument("--weights_path", type=str, default = "checkpoints/yolov3_ckpt_99.pth", help="paths to weight file")
    parser.add_argument("--conf_thres", type=float, default=0.8, help="object confidence threshold")
    parser.add_argument("--class_path", type=str, default="data/custom/classes.names", help="path to class label file")
    parser.add_argument("--nms_thres", type=float, default=0.4, help="iou thresshold for non-maximum suppression")
    opt = parser.parse_args()
    print('CONFIGURATION: ')
    print('Cuda Status: ', torch.cuda.is_available())
    print(opt)
    
    classes = load_classes(opt.class_path)  # Extracts class labels from file
    
    # Configure GPU or CPU
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print('Device loaded: ', device)  
    
    # Setup the model
    model = Darknet(opt.model_def, img_size=opt.img_size).to(device)
    print('Model setup done.')
    
    # Load weights
    if opt.weights_path.endswith(".weights"):
        # Load darknet weights
        model.load_darknet_weights(opt.weights_path)
    else:
        # Load checkpoint weights
        model.load_state_dict(torch.load(opt.weights_path, map_location = torch.device('cpu')))
    print('Loaded model weights.')
    
    model.eval() # set in evaluation model
    return model, opt, classes
    
def inference(orig_img, img, model, opt, classes):
    Tensor = torch.cuda.FloatTensor if torch.cuda.is_available() else torch.FloatTensor
    print("Performing object detection ...")
    img = Variable(img.type(Tensor))
    # Forward pass of image to get detections and performing non-max suppression
    with torch.no_grad():
        detections = model(img)
        detections = non_max_suppression(detections, opt.conf_thres, opt.nms_thres)
    # Draw bounding box and label of detection
    detections = detections[0]
    
    if detections is not None:
        detections = rescale_boxes(detections, opt.img_size, np.array(orig_img).shape[:2])
        #print(detections)
        unique_labels = detections[:, -1].cpu().unique()
        for x1, y1, x2, y2, conf, cls_conf, cls_pred in detections:
            print(classes[int(cls_pred)], cls_conf.item())
        box_w = x2 - x1
        box_h = y2 - y1
        #print(x1, y1, x2, y2, box_w, box_h)
        orig_img =  cv2.cvtColor(np.array(orig_img), cv2.COLOR_RGB2BGR)
        infer_image = cv2.rectangle(orig_img, (int(x1), int(y1)), (int(x2), int(y2)), (36,255,12), 1)
        cv2.putText(infer_image, str(classes[int(cls_pred)]) + ' ' + str(cls_conf.item()), (x1-70, y1), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (36,255,12), 2)
        #cv2.imshow("Image", infer_image)
        #cv2.waitKey(1)
        return str(classes[int(cls_pred)])
        
def img_callback(msg):
    global im_pil
    br = CvBridge()
    image = br.imgmsg_to_cv2(msg, desired_encoding='rgb8')
    simage = br.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    im_pil = Image.fromarray(image)
    cv2.imshow("Image stream", simage) 
    cv2.waitKey(1)
    #rospy.sleep(1)
      
        
        
def main():
    # Initialize ros node
    rospy.init_node("object_detector", anonymous=True)
    # Publish detected artifact to "artifact" topic
    artifact_detector = rospy.Publisher("artifact", String, queue_size=10)
    # Subrscribe to incoming image stream
    rospy.Subscriber("/COSTAR_HUSKY/front/image_raw", Imagemsg, img_callback)
    # Initialize model
    model, opt, classes = init_model()
    #path = "/home/developer/perception/yolo/PyTorch-YOLOv3/data/test/frame0012.jpg"
    #img = Image.open(path)
    #img.show()
    while not rospy.is_shutdown():
        transform = transforms.Compose([transforms.Resize((opt.img_size,opt.img_size)), transforms.ToTensor()])
        trans_img = transform(im_pil).float()
        trans_img = trans_img.unsqueeze(0)
        #print(trans_img)
        #detections = model(batch)
        detected_artifact = inference(im_pil, trans_img, model, opt, classes)
        artifact_detector.publish(detected_artifact)
    #rospy.sleep(1)
    rospy.spin()

if __name__ == '__main__':
    main()
