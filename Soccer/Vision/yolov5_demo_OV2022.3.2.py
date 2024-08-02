#!/usr/bin/env python
"""
 Copyright (C) 2018-2019 Intel Corporation

 Licensed under the Apache License, Version 2.0 (the "License");
 you may not use this file except in compliance with the License.
 You may obtain a copy of the License at

      http://www.apache.org/licenses/LICENSE-2.0

 Unless required by applicable law or agreed to in writing, software
 distributed under the License is distributed on an "AS IS" BASIS,
 WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 See the License for the specific language governing permissions and
 limitations under the License.
"""
from __future__ import print_function, division

import os
import sys
#sos.system("ldconfig")
#os.environ["LD_LIBRARY_PATH"] = str(os.environ.get("LD_LIBRARY_PATH")) + ":/opt/intel/openvino_2022.3.2/runtime/3rdparty/hddl/lib:/opt/intel/openvino_2022.3.2/runtime/lib/armv7l"
sys.path.append("/opt/intel/openvino_2022.3.2/python/python3.9")

from picamera2 import Picamera2
import logging
import os
import sys
from argparse import ArgumentParser, SUPPRESS
from math import exp as exp
from time import time
import numpy as np

import cv2
from openvino.runtime import Core

from openvino.preprocess import PrePostProcessor
from openvino.preprocess import ColorFormat

from openvino.runtime import Layout, Type

from copy import copy

import sys
sys.path.append("/home/pi/Desktop/Roki_2_Soccer/Soccer/Vision")

logging.basicConfig(format="[ %(levelname)s ] %(message)s", level=logging.INFO, stream=sys.stdout)
log = logging.getLogger()


def build_argparser():
    parser = ArgumentParser(add_help=False)
    args = parser.add_argument_group('Options')
    args.add_argument('-h', '--help', action='help', default=SUPPRESS, help='Show this help message and exit.')
    args.add_argument("-m", "--model", help="Required. Path to an .xml file with a trained model.",
                      #required=False, default="/home/pi/yolo_roma/s640_30_epochs.xml", type=str)
                      #required=False, default="/home/pi/yolo_roma/yolov5_a_fp32.xml", type=str)
                        #required=False, default="/home/pi/yolo_roma/yolov5_a.xml", type=str)
                        required=False, default="/home/pi/yolo_roma/yolov5_b.xml", type=str)
    args.add_argument("-d", "--device",
                      help="Optional. Specify the target device to infer on; CPU, GPU, FPGA, HDDL or MYRIAD is"
                           " acceptable. The sample will look for a suitable plugin for device specified. "
                           "Default value is CPU", default="MYRIAD", type=str)
    return parser

def letterbox(img, size=(640, 640), color=(114, 114, 114), auto=True, scaleFill=False, scaleup=True):
    # Resize image to a 32-pixel-multiple rectangle https://github.com/ultralytics/yolov3/issues/232
    shape = img.shape[:2]  # current shape [height, width]
    w, h = size

    # Scale ratio (new / old)
    r = min(h / shape[0], w / shape[1])
    if not scaleup:  # only scale down, do not scale up (for better test mAP)
        r = min(r, 1.0)

    # Compute padding
    ratio = r, r  # width, height ratios
    new_unpad = int(round(shape[1] * r)), int(round(shape[0] * r))
    dw, dh = w - new_unpad[0], h - new_unpad[1]  # wh padding
    if auto:  # minimum rectangle
        dw, dh = np.mod(dw, 64), np.mod(dh, 64)  # wh padding
    elif scaleFill:  # stretch
        dw, dh = 0.0, 0.0
        new_unpad = (w, h)
        ratio = w / shape[1], h / shape[0]  # width, height ratios

    dw /= 2  # divide padding into 2 sides
    dh /= 2

    if shape[::-1] != new_unpad:  # resize
        img = cv2.resize(img, new_unpad, interpolation=cv2.INTER_LINEAR)
    top, bottom = int(round(dh - 0.1)), int(round(dh + 0.1))
    left, right = int(round(dw - 0.1)), int(round(dw + 0.1))
    img = cv2.copyMakeBorder(img, top, bottom, left, right, cv2.BORDER_CONSTANT, value=color)  # add border

    top2, bottom2, left2, right2 = 0, 0, 0, 0
    if img.shape[0] != h:
        top2 = (h - img.shape[0])//2
        bottom2 = top2
        img = cv2.copyMakeBorder(img, top2, bottom2, left2, right2, cv2.BORDER_CONSTANT, value=color)  # add border
    elif img.shape[1] != w:
        left2 = (w - img.shape[1])//2
        right2 = left2
        img = cv2.copyMakeBorder(img, top2, bottom2, left2, right2, cv2.BORDER_CONSTANT, value=color)  # add border
    return img

def nms_postprocess(predictions):
    boxes = []
    class_ids = []
    confidences = []
    conf_threshold = 0.4
    score_threshold = 0.4
    nms_threshold = 0.6
    for prediction in predictions:
        confidence = prediction[4].item()
        if confidence >= conf_threshold:
            classes_scores = prediction[5:]
            _, _, _, max_indx = cv2.minMaxLoc(classes_scores)
            class_id = max_indx[1]
            if (classes_scores[class_id] > .25):
                confidences.append(confidence)
                class_ids.append(class_id)
                x, y, w, h = prediction[0].item(), prediction[1].item(), prediction[2].item(), prediction[3].item()
                xmin = x - (w / 2)
                ymin = y - (h / 2)
                xmax = xmin + w
                ymax = ymin + h
                box = np.array([xmin, ymin, xmax, ymax])
                boxes.append(box)
    indexes = cv2.dnn.NMSBoxes(boxes, confidences, score_threshold, nms_threshold)
    
    detections = []
    for i in indexes:
        j = i.item()
        detections.append({"class_index": class_ids[j], "confidence": confidences[j], "box": boxes[j]})

    detections = sorted(detections, key=lambda detections: detections["confidence"], reverse=True)

    return detections


def main():
    args = build_argparser().parse_args()


    # ------------- 1. Plugin initialization for specified device and load extensions library if specified -------------
    log.info("Creating Inference Engine...")
    # ie = IECore()
    ie = Core()

    # ------------- 2. Reading the IR generated by the Model Optimizer (.xml and .bin files) and Preprocessing----------
    model_path = args.model
    log.info(f"Loading network: {model_path}")

    core = Core()
    # Step 2. Read a model
    model = core.read_model(model_path)
    # Step 4. Inizialize Preprocessing for the model
    ppp = PrePostProcessor(model)
    # Specify input image format
    ppp.input().tensor().set_element_type(Type.u8).set_layout(Layout("NHWC")).set_color_format(ColorFormat.BGR)
    #  Specify preprocess pipeline to input image without resizing
    ppp.input().preprocess().convert_element_type(Type.f32).convert_color(ColorFormat.RGB).scale([255., 255., 255.])
    # Specify model's input layout
    ppp.input().model().set_layout(Layout("NCHW"))
    #  Specify output results format
    ppp.output().tensor().set_element_type(Type.f32)
    # Embed above steps in the graph
    model = ppp.build()

    device_name = args.device
    log.info(f"Loading network to device: {device_name}")

    compiled_model = core.compile_model(model, device_name)

    # -------------------------------------------------- 3. Open Camera ------------------------------------------------
    #path = "/home/pi/Desktop/Roki_2_Soccer/Soccer/Vision/camera_params.json"
    #sensor = KondoCameraSensor(path)
    picam2 = Picamera2(camera_num=0)
    picam2.configure(picam2.create_preview_configuration(main={"format": 'RGB888', "size": (1600, 1300)}, lores={"format": 'YUV420', "size": (800, 650)})) 
    picam2.start()

    # ----------------------------------------------- 6. Doing inference -----------------------------------------------
    log.info("Starting inference...")
    start_time = time()
    while True:
        #img = sensor.snapshot()
        #frame = img.img
        request = picam2.capture_request()
        frame = request.make_array("lores")  
        request.release()
        frame = cv2.cvtColor(frame, cv2.COLOR_YUV420p2RGB)
        frame = frame[0:650,0:800,0:3]

        # in_frame = letterbox(frame, (w, h))
        frame = letterbox(frame)

        # resize input_frame to network size
        # in_frame = in_frame.transpose((2, 0, 1))  # Change data layout from HWC to CHW
        # in_frame = in_frame.reshape((n, c, h, w))

        in_frame = np.expand_dims(frame, 0)

        # Start inference
        start_time = time()

        infer_request = compiled_model.create_infer_request()
        infer_request.infer({0: in_frame})


        # Step 7. Retrieve inference results 
        output = infer_request.get_output_tensor()

        # Step 8. Postprocessing including NMS
        detections = nms_postprocess(output.data[0])
        
        inf_time = time() - start_time
        print(f"inf_time: {inf_time}")

        

        for result in detections:
            class_label = result["class_index"]
            confidence = result["confidence"]
            box = result["box"]
            box = np.clip(box, 0, 640)
            x_min = int(box[0].item())
            y_min = int(box[1].item())
            x_max = int(box[2].item())
            y_max = int(box[3].item())
            
            # Draw bounding box on the image
            cv2.rectangle(frame, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)

            # Write class label and confidence score
            cv2.putText(frame, f"{class_label}: {confidence:.2f}", (x_min, y_min),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
        visualize_time = int(1/ (time() - start_time))
        print(f"visualize_time: {visualize_time}")
        start_time = time()
        # Display the image with bounding boxes
        cv2.imshow("Output", frame)
        key = cv2.waitKey(1)
        if key == ord('q'):
            break
    cv2.destroyAllWindows()

if __name__ == '__main__':
    sys.exit(main() or 0)