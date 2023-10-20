import cv2
from segment_anything import SamPredictor, sam_model_registry, SamAutomaticMaskGenerator
import argparse
import json
import os
import numpy as np
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

sam = sam_model_registry["vit_h"](checkpoint="sam_vit_h_4b8939.pth")
image = cv2.imread('1.jpg')
image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)


# predictor = SamPredictor(sam)
# predictor.set_image(image)
# promptPts0=np.array([[50,240],[170,416]])
# promptLabels0=np.array([1,1])
# promptPts1=np.array([[273,50],[217,633]])
# promptLabels1=np.array([2,2])
# masks0, _, _ = predictor.predict(promptPts0,promptLabels0)
# masks1, _, _ = predictor.predict(promptPts1,promptLabels1)
# filename = "0.png"
# print(len(masks0))
# cv2.imwrite("00.png", masks0[0] * 255)
# cv2.imwrite("01.png", masks0[1] * 255)
# cv2.imwrite("02.png", masks0[2] * 255)
# cv2.imwrite("10.png", masks1[0] * 255)
# cv2.imwrite("11.png", masks1[1] * 255)
# cv2.imwrite("12.png", masks1[2] * 255)

mask_generator = SamAutomaticMaskGenerator(sam)
masks = mask_generator.generate(image)
write_masks_to_folder(masks,'./1')
