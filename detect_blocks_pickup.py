#!/usr/bin/env python3

import numpy as np
import cv2 as cv
import rospy
import rospkg
from sensor_msgs.msg import Image

opencv_python_version = lambda str_version: tuple(map(int, (str_version.split("."))))
assert opencv_python_version(cv.__version__) >= opencv_python_version("4.10.0"), \
       "Please install more recent opencv-python `python3 -m pip install --upgrade opencv-python`"

class YuNet:
    def __init__(self, modelPath, inputSize=[320, 320], confThreshold=0.6, nmsThreshold=0.3, topK=5000, backendId=0, targetId=0):
        self._modelPath = modelPath
        self._inputSize = tuple(inputSize) # [w, h]
        self._confThreshold = confThreshold
        self._nmsThreshold = nmsThreshold
        self._topK = topK
        self._backendId = backendId
        self._targetId = targetId

        self._model = cv.FaceDetectorYN.create(
            model=self._modelPath,
            config="",
            input_size=self._inputSize,
            score_threshold=self._confThreshold,
            nms_threshold=self._nmsThreshold,
            top_k=self._topK,
            backend_id=self._backendId,
            target_id=self._targetId)

    @property
    def name(self):
        return self.__class__.__name__

    def setBackendAndTarget(self, backendId, targetId):
        self._backendId = backendId
        self._targetId = targetId
        self._model = cv.FaceDetectorYN.create(
            model=self._modelPath,
            config="",
            input_size=self._inputSize,
            score_threshold=self._confThreshold,
            nms_threshold=self._nmsThreshold,
            top_k=self._topK,
            backend_id=self._backendId,
            target_id=self._targetId)

    def setInputSize(self, input_size):
        self._model.setInputSize(tuple(input_size))

    def infer(self, image):
        # Forward
        faces = self._model.detect(image)
        return np.empty(shape=(0, 5)) if faces[1] is None else faces[1]
    

class FaceStitcher:
    #not needed 
    def __init__(self):
        self.model = YuNet(modelPath=f"{rospkg.RosPack().get_path('cu_sawyer_demo')}/data/face_detection_yunet_2023mar.onnx",
                inputSize=[320, 320],
                backendId=cv.dnn.DNN_BACKEND_OPENCV,
                targetId=cv.dnn.DNN_TARGET_CPU)

        self.pub = rospy.Publisher('faces', Image, queue_size=1)
        self.sub = rospy.Subscriber('/io/internal_camera/head_camera/image_rect_color', Image, queue_size=1, callback=self.cb)

    #FLAGGED conversion, check if cvbridge can be used instead
    def cb(self, msg):
        # cv_bridge is not available, so convert to bgr8 manually
        np_arr = np.frombuffer(msg.data, dtype=np.uint8)
        image = np_arr.reshape((msg.height, msg.width, -1))[:, :, 2::-1]  # Drop alpha channel if present, invert RGB to BGR for opencv

        h, w, _ = image.shape
        self.model.setInputSize([w, h])
        faces = self.model.infer(image)

        crops = []
        for idx, det in enumerate(faces):
            bbox = det[0:4].astype(np.int32)
            face = image[bbox[1]:bbox[1]+bbox[3], bbox[0]:bbox[0]+bbox[2]]
            resized_face = cv.resize(face, (128, 128))
            crops.append(resized_face)

        crops = crops[:25] # upper sanity limit

        if not crops:
            return
        
        num_faces = len(crops)
        grid_cols = min(5, num_faces)
        grid_rows = (num_faces + grid_cols - 1) // grid_cols
        stitched_image = np.zeros((grid_rows * 128, grid_cols * 128, 3), dtype=np.uint8)

        for idx, face in enumerate(crops):
            row = idx // grid_cols
            col = idx % grid_cols
            stitched_image[row*128:(row+1)*128, col*128:(col+1)*128] = face

        # manually convert to ROS image
        image_msg = Image()
        image_msg.height = stitched_image.shape[0]
        image_msg.width = stitched_image.shape[1]
        image_msg.encoding = "bgra8"
        image_msg.is_bigendian = 0
        image_msg.step = stitched_image.shape[1] * 4
        stitched_image_rgba = cv.cvtColor(stitched_image, cv.COLOR_BGR2RGBA)
        image_msg.data = stitched_image_rgba.tobytes()
        self.pub.publish(image_msg)

def main():
    rospy.init_node('face_detection')
    fs = FaceStitcher()
    rospy.spin()

if __name__ == '__main__':
    main()