import cv2
from segment_anything import SamPredictor, sam_model_registry, SamAutomaticMaskGenerator
import argparse
import json
import os
import numpy as np
import torch
from typing import Any, Dict, List

def write_masks_to_folder(masks: List[Dict[str, Any]], path: str) -> None:
    header = "id,area,bbox_x0,bbox_y0,bbox_w,bbox_h,point_input_x,point_input_y,predicted_iou,stability_score,crop_box_x0,crop_box_y0,crop_box_w,crop_box_h"  # noqa
    metadata = [header]
    for i, mask_data in enumerate(masks):
        mask = mask_data["segmentation"]
        filename = f"{i}.png"
        cv2.imwrite(os.path.join(path, filename), mask * 255)
        mask_metadata = [
            str(i),
            str(mask_data["area"]),
            *[str(x) for x in mask_data["bbox"]],
            *[str(x) for x in mask_data["point_coords"][0]],
            str(mask_data["predicted_iou"]),
            str(mask_data["stability_score"]),
            *[str(x) for x in mask_data["crop_box"]],
        ]
        row = ",".join(mask_metadata)
        metadata.append(row)
    metadata_path = os.path.join(path, "metadata.csv")
    with open(metadata_path, "w") as f:
        f.write("\n".join(metadata))

    return


image = cv2.imread('img_00024.jpg')
image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
# with open('../viewerout/landmarks/img_00024.json') as f:
#   data = json.load(f) 
# lanndmarks3D =np.array(data['frontLandmarks3d'])
# lanndmarks2D = lanndmarks3D[:,0:2].astype(int)
# promptPts0=lanndmarks2D
# promptLabels0=np.ones(promptPts0.shape[0],dtype=int) 
 


sam = sam_model_registry["vit_h"](checkpoint="sam_vit_h_4b8939.pth")
predictor = SamPredictor(sam)
predictor.set_image(image)

promptPts0=np.array([[364,254],[456,289]])
promptLabels0=np.array([1,1])
input_box = np.array([278,192, 472,459])

masks, scores, logits = predictor.predict(promptPts0,promptLabels0,box=input_box[None, :]) 
print('scores=',scores) 
cv2.imwrite("00.png", masks[0] * 255)
cv2.imwrite("01.png", masks[1] * 255) 
cv2.imwrite("02.png", masks[2] * 255) 

#mask_generator = SamAutomaticMaskGenerator(sam)
#masks = mask_generator.generate(image)
#write_masks_to_folder(masks,'./1')


 
