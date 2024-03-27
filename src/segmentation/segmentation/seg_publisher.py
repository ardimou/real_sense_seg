import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import yaml
from pycocotools import mask
import os

# Import SAM related modules
from segment_anything import SamAutomaticMaskGenerator, sam_model_registry

class SegmentationPublisher(Node):
    def __init__(self):
        super().__init__('segmentation_publisher')
        self.publisher = self.create_publisher(Image, 'segmented_image', 10)
        self.bridge = CvBridge() 
        with open('params.yaml', 'rb') as file:              
            params = yaml.safe_load(file)
        	
        sam = sam_model_registry[params['model_type']](checkpoint=params['checkpoint']) 
        _ = sam.to(device=params['device'])
        output_mode = params['output_mode']
        amg_kwargs = self.get_args(params)
        self.generator = SamAutomaticMaskGenerator(sam, output_mode=output_mode, **amg_kwargs)
        # Load SAM model
        #self.sam_model = SamAutomaticMaskGenerator(...)
        self.image_dir = "./dataset"
        # Publisher for segmentation masks
        

    def segment_images(self):
        # Convert ROS Image message to OpenCV image
        self.get_logger().info('Segmenting image...')
        for filename in os.listdir(self.image_dir):
            print(f"filename = {filename}")
            image_path = os.path.join(self.image_dir, filename)
            cv_image = cv2.imread(image_path)
            #cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    
            # Perform segmentation with SAM model
            result = self.perform_segmentation(cv_image)
            
            segmented_image = self.combine_masks(result, cv_image)
            
            blended_image = cv2.addWeighted(cv_image, 0.2, segmented_image, 0.8, 0)
    
            # Publish segmented image
            segmented_msg = self.bridge.cv2_to_imgmsg(blended_image, encoding='bgr8')
            self.segmentation_publisher.publish(segmented_msg)


    def perform_segmentation(self, image):
        # Perform segmentation using SAM model
        masks = self.generator.generate(image)

        # Post-processing if necessary

        return masks
        
    def combine_masks(self, result, cv_image):
        # Combine masks into a single segmented image
        # Assign different colors to different objects
        segmented_image = np.zeros_like(cv_image, dtype=np.uint8)
        idx = 0
        for mask_item in result:
            rle = mask_item['segmentation']
            binary_mask = mask.decode(rle) 
        if np.sum(binary_mask) > 3000:
                idx += 1 
                color = np.array(np.random.rand(3) * 255, np.uint8) # Generate a random color
                ind = binary_mask > 0
                segmented_image[ind, :] = color
			# Expand the binary mask to three channels (RGB)
			#mask_rgb = np.repeat(binary_mask[:, :, np.newaxis], 3, axis=2)
			#ax.imshow(mask_col, alpha = 0.1)        	
        	
        
        #colors = np.random.randint(0, 255, size=(len(masks), 3), dtype=np.uint8)

        #for idx, mask in enumerate(masks):
            #segmented_image[mask > 0] = colors[idx]

        return segmented_image
    
    def get_args(self, params):
        args = {
            "points_per_side": params["points_per_side"],
            "points_per_batch": params["points_per_batch"],
            "pred_iou_thresh": params["pred_iou_thresh"],
            "stability_score_thresh": params["stability_score_thresh"],
            "stability_score_offset": params["stability_score_offset"],
            "box_nms_thresh": params["box_nms_thresh"],
            "crop_n_layers": params["crop_n_layers"],
            "crop_nms_thresh": params["crop_nms_thresh"],
            "crop_overlap_ratio": params["crop_overlap_ratio"],
            "crop_n_points_downscale_factor": params["crop_n_points_downscale_factor"],
            "min_mask_region_area": params["min_mask_region_area"]
        } 
        
        amg_kwargs = {k: v for k, v in args.items() if v != 'None'}
        
        for i in amg_kwargs:
            print(f"{i} : {amg_kwargs[i]}")
             
        return amg_kwargs

def main(args=None):
    rclpy.init(args=args)
    segmentation_publisher = SegmentationPublisher()
    segmentation_publisher.segment_images()  # Segment images and publish them
    rclpy.spin_once(segmentation_publisher)  # Spin once to allow messages to be published
    rclpy.shutdown()

if __name__ == '__main__':
    main()
