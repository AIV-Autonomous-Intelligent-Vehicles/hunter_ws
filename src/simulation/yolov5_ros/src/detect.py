#!/usr/bin/env python3

import rospy
import cv2
import torch
import torch.backends.cudnn as cudnn
import numpy as np
from cv_bridge import CvBridge
from pathlib import Path
import os
import sys
from rostopic import get_topic_type
import time
from collections import Counter
from sensor_msgs.msg import Image, CompressedImage
# from detection_msgs.msg import BoundingBox, BoundingBoxes
from cob_perception_msgs.msg import Detection, DetectionArray
from geometry_msgs.msg import PoseStamped, Point
from cob_object_detection_msgs.srv import DetectObjects, DetectObjectsResponse


# add yolov5 submodule to path 
# to find python subdir
FILE = Path(__file__).resolve()
ROOT = FILE.parents[0] / "yolov5"
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))  # add ROOT to PATH
ROOT = Path(os.path.relpath(ROOT, Path.cwd()))  # relative path

# import from yolov5 submodules
from models.common import DetectMultiBackend
from utils.general import (
    check_img_size,
    check_requirements,
    non_max_suppression,
    scale_coords
)
from utils.plots import Annotator, colors
from utils.torch_utils import select_device
from utils.augmentations import letterbox


@torch.no_grad() # to improve inference speed(exclude gradient calculation)
class Yolov5Detector:
    def __init__(self):
        self.conf_thres = rospy.get_param("~confidence_threshold")
        self.iou_thres = rospy.get_param("~iou_threshold")
        self.agnostic_nms = rospy.get_param("~agnostic_nms")
        self.max_det = rospy.get_param("~maximum_detections")
        self.classes = rospy.get_param("~classes", None)
        self.line_thickness = rospy.get_param("~line_thickness")
        self.view_image = rospy.get_param("~view_image")
        self.view_image2 = rospy.get_param("~view_image2")
        
        # Initialize weights 
        weights = rospy.get_param("~weights")
        
        
        # Initialize model(To put the weight into the initialized model)
        self.device = select_device(str(rospy.get_param("~device","auto")))
        self.model = DetectMultiBackend(weights, device=self.device, dnn=rospy.get_param("~dnn"), data=rospy.get_param("~data"))
        self.stride, self.names, self.pt, self.jit, self.onnx, self.engine = (
            self.model.stride,
            self.model.names,
            self.model.pt,
            self.model.jit,
            self.model.onnx,
            self.model.engine,
        )
    

        # Setting inference size
        self.img_size = [rospy.get_param("~inference_size_w", 512), rospy.get_param("~inference_size_h",512)]
        self.img_size = check_img_size(self.img_size, s=self.stride)

        # Half
        # Enable half-precision (FP16) if the parameter is set to True. FP16 is a technology that uses 16-bit floating-point numbers
        # to reduce the weight and precision of the model's operations, reducing memory usage and increasing inference speed.
        self.half = rospy.get_param("~half", False)
        self.half &= (
            self.pt or self.jit or self.onnx or self.engine
        ) and self.device.type != "cpu"  # FP16 supported on limited backends with CUDA
        if self.pt or self.jit:
            self.model.model.half() if self.half else self.model.model.float()
        bs = 1  # batch_size
        cudnn.benchmark = True  # set True to speed up constant image size inference
        self.model.warmup()  # warmup        
        
        # Initialize subscriber to Image/CompressedImage topic
        input_image_type, input_image_topic_R, _ = get_topic_type(rospy.get_param("~input_image_topic_R"), blocking = True)
        self.compressed_input = input_image_type == "sensor_msgs/CompressedImage"

        if self.compressed_input:
            self.image_sub = rospy.Subscriber(
                input_image_topic_R, CompressedImage, self.callback_r, queue_size=1
            )
        else:
            self.image_sub = rospy.Subscriber(
                input_image_topic_R, Image, self.callback_r, queue_size=1
            )

        input_image_type2, input_image_topic_L, _ = get_topic_type(rospy.get_param("~input_image_topic_L"), blocking = True)
        self.compressed_input2 = input_image_type2 == "sensor_msgs/CompressedImage"

       

        if self.compressed_input2:
            self.image_sub2 = rospy.Subscriber(
                input_image_topic_L, CompressedImage, self.callback_l, queue_size=1
            )
        else:
            self.image_sub2 = rospy.Subscriber(
                input_image_topic_L, Image, self.callback_l, queue_size=1
            )

        

        # add
        self.cob_detection_pub_r = rospy.Publisher(
            rospy.get_param("~yolo_output_topic_to_matlab_r"), DetectionArray, queue_size=10
        )

        self.cob_detection_pub_l = rospy.Publisher(
            rospy.get_param("~yolo_output_topic_to_matlab_l"), DetectionArray, queue_size=10
        )


        # Initialize image publisher
        self.publish_image = rospy.get_param("~publish_image")
        if self.publish_image:
            self.image_pub_r = rospy.Publisher(
                rospy.get_param("~output_image_topic_r"), Image, queue_size=10
            )
            
        self.publish_image2 = rospy.get_param("~publish_image2")
        if self.publish_image2:
            self.image_pub_l = rospy.Publisher(
                rospy.get_param("~output_image_topic_l"), Image, queue_size=10
            )
        
        
        # Initialize CV_Bridge
        self.bridge = CvBridge()

        # Initialize service server 
        self.service = rospy.Service('Activate_yolo', DetectObjects, self.activate_yolo_service_callback)


    def activate_yolo_service_callback(self, req):
        print("Callback")
        if req.object_name.data == 'right':
            if self.compressed_input:
                im = self.bridge.compressed_imgmsg_to_cv2(self.data_r, desired_encoding="bgr8")
                header = self.data_r.header
            else:
                im = self.bridge.imgmsg_to_cv2(self.data_r, desired_encoding="bgr8")
                header = self.data_r.header

        else:
            if self.compressed_input2:
                im = self.bridge.compressed_imgmsg_to_cv2(self.data_l, desired_encoding="bgr8")
                header = self.data_l.header
            else:
                im = self.bridge.imgmsg_to_cv2(self.data_l, desired_encoding="bgr8")
                header = self.data_l.header

        im, im0 = self.preprocess(im)

        # Run inference
        im = torch.from_numpy(im).to(self.device) 
        im = im.half() if self.half else im.float()
        im /= 255
        if len(im.shape) == 3:
            im = im[None]

        pred = self.model(im, augment=False, visualize=False)
        pred = non_max_suppression(
            pred, self.conf_thres, self.iou_thres, self.classes, self.agnostic_nms, max_det=self.max_det
        )

        ### To-do move pred to CPU and fill BoundingBox messages
        
        # Process predictions 
        det = pred[0].cpu().numpy()


        detection_array = DetectionArray()
        detection_array.header = header

        annotator = Annotator(im0, line_width=self.line_thickness, example=str(self.names))
        
        
        if len(det):

            # Rescale boxes from img_size to im0 size
            det[:, :4] = scale_coords(im.shape[2:], det[:, :4], im0.shape).round()

            # Write results
            
            for *xyxy, conf, cls in reversed(det):
                # bounding_box = BoundingBox()
                c = int(cls)

                # Fill in cob_perception_msgs/Detection message
                cob_detections = Detection()
                cob_detections.header = header
                cob_detections.label = self.names[c]
                cob_detections.detector = "yolov5" # detector's name
                cob_detections.score = conf

                # xmin, ymin, width, height 
                cob_detections.mask.roi.x = int(xyxy[0]) # xmin
                cob_detections.mask.roi.y = int(xyxy[1]) # ymin
                cob_detections.mask.roi.width = int(xyxy[2]) - int(xyxy[0])
                cob_detections.mask.roi.height = int(xyxy[3]) - int(xyxy[1])
                
                # x,y coord
                cob_detections.bounding_box_lwh = Point()
                cob_detections.bounding_box_lwh.x = float((int(xyxy[2]) + int(xyxy[0])) / 2)
                cob_detections.bounding_box_lwh.y = float((int(xyxy[3]) + int(xyxy[1])) / 2)
        
                detection_array.detections.append(cob_detections)

                # Annotate the image
                if self.publish_image or self.view_image:  # Add bbox to image
                    # integer class
                    label = f"{self.names[c]} {conf:.2f}"
                    annotator.box_label(xyxy, label, color=colors(c, True))       

                
            # display class and number of objects 
            det_classes = det[:, -1].tolist()
            class_counts = Counter(det_classes)

            rs = ""
            for class_index, count in class_counts.items():
                class_name = self.names[int(class_index)]
                if count > 1:
                    plus_s = 's'
                else :
                    plus_s = ''
                rs += f"Detect : {count} {class_name}{plus_s}, "
            rs = rs[:-2]
            print(rs + '\t')


            im0 = annotator.result()
        


        if req.object_name.data == 'right':
            self.image_pub_r.publish(self.bridge.cv2_to_imgmsg(im0, "bgr8"))
            self.cob_detection_pub_r.publish(detection_array)
        else:
            self.image_pub_l.publish(self.bridge.cv2_to_imgmsg(im0, "bgr8"))
            self.cob_detection_pub_l.publish(detection_array)

        if len(det)==0:
            print("NO OBJECT DETECTION\n")
        

        
        response = DetectObjectsResponse() 
        response.object_list = detection_array
        return response

    def callback_r(self, data):
        self.data_r = data
        

    def callback_l(self, data):
        self.data_l = data




    def preprocess(self, img):
        """
        Adapted from yolov5/utils/datasets.py LoadStreams class
        """
        img0 = img.copy()
        img = np.array([letterbox(img, self.img_size, stride=self.stride, auto=self.pt)[0]])
        # Convert
        img = img[..., ::-1].transpose((0, 3, 1, 2))  # BGR to RGB, BHWC to BCHW
        img = np.ascontiguousarray(img)

        return img, img0 


if __name__ == "__main__":

    check_requirements(exclude=("tensorboard", "thop"))
    
    rospy.init_node("yolov5", anonymous=True)
    detector = Yolov5Detector()
    
    rospy.spin()