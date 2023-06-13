from typing import Tuple
import re
import os
import sys
import numpy as np
import cv2
from common.logger import logger
from common.type_define import Empty


def image_down_sampling(image:np.ndarray, width:int=128) -> np.ndarray:
    # 获取原图像的形状
    height, org_width= image.shape[:2]

    # 计算缩放比例
    scale = width / org_width
    new_height = int(height * scale)

    # 使用 OpenCV 进行图像缩放
    import cv2
    new_image = cv2.resize(image, (width, new_height), cv2.INTER_AREA)

    return new_image
    

def load_image(filepath: str) -> Tuple[np.ndarray, float]:
    """load image from filepath, return image in np array and return resolution.
    Args:
        filepath (str): file path

    Returns:Union[np.ndarray, int, bool]
        img[np.ndarray[np.uint8]] 
        resulotion[int, unit: m]
    """
    print(filepath)
    if not re.search(pattern=r'resolution_0\.[0-9]+|resolution_[1-9]\d*\.\d+', string=filepath):
        logger.fatal('the filename is not correct!')
        raise RuntimeError()
    if not os.path.exists(filepath):
        logger.fatal('the file doesn\'t exist!')
        raise RuntimeError()
        
    # read image using imread function from scikit-image library
    image = cv2.imread(filepath)
    image = image_down_sampling(image)
    # 将 RGBA 图像转换为二值图像
    gray = cv2.cvtColor(image, cv2.COLOR_RGBA2GRAY)
    _, binary = cv2.threshold(gray, thresh=127, maxval=255, type=cv2.THRESH_BINARY)
    binary_image = np.where(binary > 0.5, 1, 0)
    # extract resolution from filepath
    resolution = float(re.findall(pattern=r'\d+\.\d+', string=filepath.split('/')[-1])[0])
    return binary_image, resolution


if __name__ == '__main__':
    image_path = 'files/map_files/map_resolution_0.05.png'
    load_image(image_path)
    
    