from typing import Optional, Sequence, Dict, Union

import torch
from PIL import ImageGrab as Screenshot
from time import sleep
import pyautogui

class DetectionUtilities:
    def __init__(self):
        pass

    def findLargest(self, array: Sequence) -> Optional[int]:
        if len(array) == 0:
            return None

        max_index = 0
        for i in range(1, len(array)):
            if array[i] > array[max_index]:
                max_index = i

        return max_index

    def getBestObject(self, detections: Dict[str, list]) -> Optional[Dict[str, Union[int, str]]]:
        obj_index = self.findLargest(detections['confidence'])
        if obj_index is None:
            return obj_index

        data = {
            'xmin': round(detections['xmin'][obj_index]),
            'xmax': round(detections['xmax'][obj_index]),
            'ymin': round(detections['ymin'][obj_index]),
            'ymax': round(detections['ymax'][obj_index]),
            'confidence': round(detections['confidence'][obj_index], 2),
            'name': detections['name'][obj_index]
        }
        return data

class LabelingUtilities:
    def __init__(self) -> None:
        self.window: Dict[str, Optional[tuple[int, int]]] = {
            'top_left': None,
            'bottom_right': None
        }

    def clamp(self, x: int, y: int) -> tuple[int, int]:
        if None in self.window.values():
            return x, y

        x = max(min(x, self.window['bottom_right'][0]), self.window['top_left'][0])
        y = max(min(y, self.window['bottom_right'][1]), self.window['top_left'][1])

        return x, y

    def setWindow(self, top_left: tuple[int, int], bottom_right: tuple[int, int]):
        self.window = {
            'top_left': top_left,
            'bottom_right': bottom_right
        }

    def drawBoundingBox(self, detection: Optional[Dict[str, Union[int, str]]]) -> None:
        if detection is None:
            return

        pyautogui.moveTo(self.clamp(detection['xmin'], detection['ymin']), duration=0.2)
        pyautogui.mouseDown(button='left')
        pyautogui.moveTo(self.clamp(detection['xmax'], detection['ymax']), duration=0.33)
        pyautogui.mouseUp(button='left')
        pyautogui.typewrite(detection_data['name'][0], interval=0.16)

if __name__ == '__main__':
    model = torch.hub.load('ultralytics/yolov5', 'custom', 'best.pt', force_reload=True, verbose=False, _verbose=False)
    model.conf = 0.3
    detectionUtils = DetectionUtilities()
    labelUtils = LabelingUtilities()
    labelUtils.setWindow((159, 110), (955, 707))

    for _ in range(350):
        results = model(Screenshot.grab(), size=640)
        detection_data = detectionUtils.getBestObject(results.pandas().xyxy[0])
        labelUtils.drawBoundingBox(detection_data)

        pyautogui.click(385, 775)
        sleep(1)