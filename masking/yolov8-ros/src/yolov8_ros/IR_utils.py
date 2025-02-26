import torch
from torch import nn
import cv2
from torchvision.ops import nms

def custom_post_process(outputs, img_height, img_width, confidence_threshold=0.5, iou_threshold=0.7):
    """
    Post-process the model outputs to convert normalized bounding boxes
    to real coordinates (xmin, ymin, xmax, ymax).

    Args:
        outputs (torch.Tensor): The model's output predictions of shape (batch_size, 100, 4).
        img_height (int): Height of the image.
        img_width (int): Width of the image.
        confidence_threshold (float): Minimum confidence score to consider a bounding box.
        iou_threshold (float): IoU threshold for NMS.

    Returns:
        List of dictionaries containing boxes and scores for each image in the batch.
    """
    device = outputs.pred_boxes.device  # Ensure we match the device of the outputs
    out_logits, out_bbox = outputs.logits, outputs.pred_boxes
    batch_boxes = []  # Store results for the entire batch

    for img_idx in range(out_bbox.shape[0]):  # Iterate over each image in the batch
        image_boxes = []
        for box in out_bbox[img_idx]:  # Iterate over each predicted bounding box (100 per image)
            x_center, y_center, bbox_width, bbox_height = box

            # Convert normalized [x_center, y_center, width, height] to [xmin, ymin, xmax, ymax]
            xmin = int(x_center * img_width - 0.5 * bbox_width * img_width)
            ymin = int(y_center * img_height - 0.5 * bbox_height * img_height)
            xmax = int(x_center * img_width + 0.5 * bbox_width * img_width)
            ymax = int(y_center * img_height + 0.5 * bbox_height * img_height)

            # Ensure bounding box coordinates are within image boundaries
            xmin = max(0, min(xmin, img_width - 1))
            ymin = max(0, min(ymin, img_height - 1))
            xmax = max(0, min(xmax, img_width - 1))
            ymax = max(0, min(ymax, img_height - 1))

            # Append the processed box to the image's list
            # print(f"image_boxes: {(xmin, ymin, xmax, ymax)}")
            image_boxes.append((xmin, ymin, xmax, ymax))

        batch_boxes.append(image_boxes)
    
    batch_boxes = torch.tensor(batch_boxes, dtype=torch.float32, device=device) # Convert to tensor (Otherwise TypeError: only integer tensors of a single element can be converted to an index (b))
    
    # print(out_logits.shape) [8,100,3]    
    prob = nn.functional.softmax(out_logits, -1)
    scores, labels = prob[..., :-1].max(-1)
    # print(scores.shape, labels.shape) [8,100], [8,100]
    
    results = []
    for s, l, b in zip(scores, labels, batch_boxes):
        # print(s.shape, l.shape) # [100], [100]
        score = s[s > confidence_threshold]
        label = l[s > confidence_threshold]
        # print(f"------------------------{b.shape}")
        box = b[s > confidence_threshold]
        # print(f"::::::::::::::::::::::::{box.shape}")
        
        # Apply Non-Maximum Suppression (NMS)
        nms_indices = nms(box, score, iou_threshold)
        score = score[nms_indices]
        label = label[nms_indices]
        box = box[nms_indices]
        results.append({"scores": score, "labels": label, "boxes": box})

    return results

def process_val_gt_bboxes(raw_true_boxes, img_height, img_width):
    resulted_bboxes = []
    for idx in range(len(raw_true_boxes)):
        # Retrieve image and labels from the dataset
        box = raw_true_boxes[idx]
        
        # [x_center, y_center, width, height]
        x_center = box[0]
        y_center = box[1]
        bbox_width = box[2]
        bbox_height = box[3]
        xmin, ymin, xmax, ymax = (
            int(x_center * img_width - 0.5 * bbox_width * img_width),
            int(y_center * img_height - 0.5 * bbox_height * img_height),
            int(x_center * img_width + 0.5 * bbox_width * img_width),
            int(y_center * img_height + 0.5 * bbox_height * img_height),
        )
        resulted_bboxes.append([xmin, ymin, xmax, ymax])
    
    return resulted_bboxes       
    
