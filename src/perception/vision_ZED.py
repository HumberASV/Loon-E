########################################################################
#
# Copyright (c) 2022, STEREOLABS.
#
# All rights reserved.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
########################################################################

from ultralytics import YOLO
import cv2
import pyzed.sl as sl
import math
import numpy as np
import sys
import math

def object_detection(image):
    MODEL = YOLO("") #model name here

    image = cv2.cvtColor(image, cv2.COLOR_RGBA2BGR)
    result = MODEL(image)[0] #Run model
    type = result.boxes.cls #Get classifiers
    position = result.boxes.xywh #Get positions of bounding boxes in ()

    return type, position

def object_location():
    objects = []
    locations = []
    # Create a Camera object
    zed = sl.Camera()

    # Create a InitParameters object and set configuration parameters
    init_params = sl.InitParameters()
    init_params.depth_mode = sl.DEPTH_MODE.NEURAL  # Use NEURAL depth mode
    init_params.coordinate_units = sl.UNIT.MILLIMETER  # Use meter units (for depth measurements)

    # Open the camera
    status = zed.open(init_params)
    if status > sl.ERROR_CODE.SUCCESS: #Ensure the camera has opened succesfully
        print("Camera Open : "+repr(status)+". Exit program.")
        exit()

    # Create and set RuntimeParameters after opening the camera
    runtime_parameters = sl.RuntimeParameters()
    
    image = sl.Mat()
    point_cloud = sl.Mat()

    mirror_ref = sl.Transform()
    mirror_ref.set_translation(sl.Translation(2.75,4.0,0))

    # A new image is available if grab() returns ERROR_CODE.SUCCESS or a WARNING (an error_code lower than ERROR_CODE.SUCCESS)
    if zed.grab(runtime_parameters) <= sl.ERROR_CODE.SUCCESS:
        # Retrieve left image
        zed.retrieve_image(image, sl.VIEW.LEFT)
        # Retrieve colored point cloud. Point cloud is aligned on the left image.
        zed.retrieve_measure(point_cloud, sl.MEASURE.XYZRGBA)
        
        type, position = object_detection(image)

        for i in range(len(type)):
            # Get and print distance value in mm at the center of the image
            # We measure the distance camera - object using Euclidean distance
            x = position[i][0]
            y = position[i][1]
            err, point_cloud_value = point_cloud.get_value(x, y)

            if math.isfinite(point_cloud_value[2]):
                objects.append(type[i])
                locations.append((point_cloud_value[0], point_cloud_value[1]))          

    # Close the camera
    zed.close()
    return objects