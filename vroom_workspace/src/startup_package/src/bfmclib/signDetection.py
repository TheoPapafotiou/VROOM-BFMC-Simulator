import cv2
import numpy as np
import math

class SignDetection:
    
    """
    This class implements the sign & traffic lights detection procedure in our vehicle.

    For the detection a trained tiny-YOLO deep neural network is used.
    """

    def __init__(self):
        self.detections = {
            0: {
            "class": ['ParkingSpot','Crosswalk','Ahead','HighwayEnd','HighwayStart','PriorityRoad','Stop','NoEntry','Roundabout','TrafficLights'],
                "net": cv2.dnn.readNetFromDarknet("yolov10_tiny-custom.cfg",r"weights/yolov3_tiny-custom_total.weights")
            }
        }
        self.label = "Something"
        self.distance = 0
        self.x_camera = 320
        self.y_camera = 480
        self.center_x = 0
        self.center_y = 0
        self.confidence_limit = 0.9
        self.nms_param1 = .8
        self.nms_param2 = .4

    def detectSignProcedure(self, net, classes, blob, img, hight, width):
        net.setInput(blob)
        output_layers_name = net.getUnconnectedOutLayersNames()
        layerOutputs = net.forward(output_layers_name)
        self.label = 'Something'
        self.distance = 0

        boxes =[]
        confidences = []
        class_ids = []

        for output in layerOutputs:
            for detection in output:
                score = detection[5:]
                class_id = np.argmax(score)
                confidence = score[class_id]
                if confidence >= self.confidence_limit:
                    self.center_x = int(detection[0] * width)
                    self.center_y = int(detection[1] * hight)
                    w = int(detection[2] * width)
                    h = int(detection[3]* hight)
                    x = int(self.center_x - w/2)
                    y = int(self.center_y - h/2)
                    boxes.append([x,y,w,h])
                    confidences.append((float(confidence)))
                    class_ids.append(class_id)

        indexes = cv2.dnn.NMSBoxes(boxes,confidences,self.nms_param1,self.nms_param2)
        font = cv2.FONT_HERSHEY_PLAIN
        colors = np.random.uniform(0,255,size =(len(boxes),3))
        if  len(indexes)>0:
            for i in indexes.flatten():
                x,y,w,h = boxes[i]
                self.label = str(classes[class_ids[i]])
                confidence = str(round(confidences[i],2))
                self.distance = self.y_camera - self.center_y
                #print("I found a " + self.label + " sign with confidence " + confidence + " at a distance: " + str(self.distance))
                color = colors[i]
                cv2.rectangle(img,(x,y),(x+w,y+h),color,2)
                cv2.putText(img,self.label, (x-int(self.x_camera/2),y+int(self.y_camera/4)),font,2,color,2)
                cv2.putText(img,"Confidence: " + confidence, (x-int(self.x_camera/2),y+int(self.y_camera/3)),font,2,color,2)

        return self.label, self.distance

    def detectSign(self, img, hight, width):
        blob = cv2.dnn.blobFromImage(img, 1/255,(416,416),(0,0,0),swapRB = True,crop= False)
        cfs = 0
        label, distance = self.detectSignProcedure(
            self.detections[cfs]['net'],
            self.detections[cfs]['class'],
            blob,
            img,
            hight,
            width
        )
        return label, distance

