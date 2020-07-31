"""
Script to classify artifacts

Reference:
1. https://docs.microsoft.com/en-us/azure/cognitive-services/custom-vision-service/export-model-python

Authors:
Nalin Das (nalindas9@gmail.com)
Graduate Student pursuing Masters in Robotics,
University of Maryland, College Park
"""

import tensorflow as tf
print(tf.__version__)
import os
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '2'
from PIL import Image
import numpy as np
import cv2

graph_def = tf.compat.v1.GraphDef()
labels = []

# These are set to the default names from exported models, update as needed.
filename = "/home/developer/subt_ws/src/perception/tensorflow/subt-artf/model/frozen_inference_graph.pb"
labels_filename = "/home/developer/subt_ws/src/perception/tensorflow/subt-artf/model/subt_label_map.pbtxt"

def crop_center(img,cropx,cropy):
    h, w = img.shape[:2]
    startx = w//2-(cropx//2)
    starty = h//2-(cropy//2)
    return img[starty:starty+cropy, startx:startx+cropx]
    
# Import the TF graph
with tf.io.gfile.GFile(filename, 'rb') as f:
    graph_def.ParseFromString(f.read())
    tf.import_graph_def(graph_def, name='')

# Create a list of labels.
with open(labels_filename, 'rt') as lf:
    for l in lf:
        labels.append(l.strip())

graph_nodes=[n for n in graph_def.node]
names = []
for t in graph_nodes:
  names.append(t.name)
print(names[-1])

# Get the input size of the model
with tf.compat.v1.Session() as sess:
    input_tensor_shape = sess.graph.get_tensor_by_name('image_tensor:0').shape.as_list()
network_input_size = input_tensor_shape[1]

print("network_input_size: ", network_input_size)

image = cv2.imread("/home/developer/subt_ws/src/perception/yolo/data/artifact_dataset/Images/backpackSurvivor/frame0031.jpg")

h, w = image.shape[:2]
min_dim = min(w,h)
max_square_image = crop_center(image, min_dim, min_dim)

# Resize that square down to 256x256
augmented_image = cv2.resize(max_square_image, (256, 256), interpolation = cv2.INTER_LINEAR)

# Crop the center for the specified network_input_Size
#augmented_image = crop_center(augmented_image, network_input_size, network_input_size)

# These names are part of the model and cannot be changed.
output_layer = 'raw_detection_scores:0'
input_node = 'image_tensor:0'

with tf.compat.v1.Session() as sess:
    try:
        prob_tensor = sess.graph.get_tensor_by_name(output_layer)
        predictions, = sess.run(prob_tensor, {input_node: [augmented_image] })
    except KeyError:
        print ("Couldn't find classification output layer: " + output_layer + ".")
        print ("Verify this a model exported from an Object Detection project.")
        exit(-1)

# Print the highest probability label
highest_probability_index = np.argmax(predictions)
print("predictions: ", predictions)
print('')
print('labels: ', labels)
print('')
print("highest_probability_index: ", highest_probability_index)
print('')
print('Classified as: ' + labels[highest_probability_index])
print()

# Or you can print out all of the results mapping labels to probabilities.
label_index = 0
for p in predictions:
    truncated_probablity = np.float64(np.round(p,8))
    print (labels[label_index], truncated_probablity)
    label_index += 1
             
cv2.imshow('Image', augmented_image)
cv2.waitKey(0)


