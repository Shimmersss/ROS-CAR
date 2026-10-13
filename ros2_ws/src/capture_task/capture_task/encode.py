import cv2
import numpy as np
from .protocol import MAX_IMAGE


def encode_jpeg(rgb, quality=95):
    if not isinstance(rgb,np.ndarray) or rgb.dtype != np.uint8 or rgb.ndim != 3 or rgb.shape[2] != 3 or not 0 < rgb.shape[0]*rgb.shape[1] <= 40000000:
        raise ValueError('invalid_rgb')
    ok, encoded = cv2.imencode('.jpg', cv2.cvtColor(rgb,cv2.COLOR_RGB2BGR),
                               [cv2.IMWRITE_JPEG_QUALITY,int(quality)])
    if not ok or encoded.size > MAX_IMAGE:
        raise ValueError('jpeg_encoding_or_size')
    return encoded.tobytes()
