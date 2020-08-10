"""
Inference code for running Robotika's pretrained model

Reference:
1.https://github.com/leimao/Frozen_Graph_TensorFlow/blob/d8450e8d584fee7d1bb82477b461a71d49e42937/TensorFlow_v1/test_pb.py#L86

Authors:
Nalin Das (nalindas9@gmail.com)
Graduate Student pursuing Masters in Robotics,
University of Maryland, College Park
"""

import os
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '2'
import tensorflow as tf 
import numpy as np
import argparse
import cv2

class CNN:
    def __init__(self, path):
        self.path = path
        self.load_graph(self.path)
        
    def load_graph(self, model_path):
        print('Loading model ...')
        print('')
        self.graph = tf.Graph()
        
        with tf.io.gfile.GFile(model_path, 'rb') as f:
            graph_def = tf.compat.v1.GraphDef()
            graph_def.ParseFromString(f.read())
        
        print('Checkout the i/p placeholders:')
        print('')
        nodes = [
            n.name + '=>' + n.op for n in graph_def.node
            if n.op in ('Placeholder')
        ]   
        
        for node in nodes:
            print(node)
        
        with self.graph.as_default():
            # Define the i/p tensor
            self.input = tf.compat.v1.placeholder(np.uint8,
                                        shape=[240, 320, 3],
                                        name = 'image_tensor'
                                        )  
            tf.import_graph_def(
                graph_def, 
                {
                    'image_tensor': self.input
                }
            )

        self.graph.finalize()
        
        print('Model loading complete!')
        

        # Get layer names
        layers = [op.name for op in self.graph.get_operations()]
        for layer in layers:
            print(layer)
           
        self.sess = tf.compat.v1.Session(graph=self.graph)
        
    def test(self, data):
        output_tensor = self.graph.get_tensor_by_name("import/detection_classes:0")  
        output = self.sess.run(output_tensor,
                                feed_dict = {
                                    self.input: data
                                }
                               )              
        return output
            
                
def frozenGraphInference(path):
    tf.compat.v1.reset_default_graph()
    print("******** Graph reset! ***********")
    print('')
    
    image = cv2.imread("/home/developer/subt_ws/src/perception/yolo/data/artifact_dataset/Images/backpackSurvivor/frame0031.jpg")
    
    model = CNN(path)
    
    #cv2.resize(image, ((32, 32))
    #cv2.imshow('Image: ', image)
    #cv2.waitKey(0)
    test_predictions = model.test(image)
    print("test_predictions: ", test_predictions)
    
def main():
    model_path = '/home/developer/subt_ws/src/perception/tensorflow/subt-artf/model/frozen_inference_graph.pb'
    
    frozenGraphInference(model_path)

if __name__ == '__main__':
    main()
