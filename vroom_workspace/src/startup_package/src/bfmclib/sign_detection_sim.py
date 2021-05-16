import cv2
import numpy as np
import os
import time
import redis
import base64
from signDetection import SignDetection
from time import sleep
from redisComm import RedisComm

SignsD = SignDetection()
RedisC = RedisComm()

r = redis.Redis(host='localhost', port=6379, db=0)
p = redis.Redis(host='localhost', port=6379, db=0)
t = redis.Redis(host='localhost', port=6379, db=0)
sim = r.pubsub()
sim.subscribe('simulation-channel')

while True:
    message = sim.get_message()
    if message is not None:
        data_from_dict = message['data']
        if data_from_dict != 1:
            
            imgSignD_binary = base64.b64decode(data_from_dict)
            imgSignD_np = np.frombuffer(imgSignD_binary, dtype=np.uint8)
            imgSignD = cv2.imdecode(imgSignD_np, cv2.IMREAD_COLOR)
            dim = imgSignD[:,:,0].shape
            framewidth = dim[1]
            frameheight = dim[0]

            label, distance = SignsD.detectSign(imgSignD, frameheight, framewidth)

            retval, buffer = cv2.imencode('.jpg', imgSignD)
            imgSignD_as_text = base64.b64encode(buffer)

            r.publish('detection-channel', imgSignD_as_text)

            p.publish('label-channel', label)

            t.publish('distance-channel', distance)
