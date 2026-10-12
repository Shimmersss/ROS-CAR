"""Readable labels drawn at the final preview resolution."""
import cv2


def draw_labels(image, box, lines):
    """Keep one person's label rows together and inside the image."""
    height, width = image.shape[:2]
    font, size, thickness = cv2.FONT_HERSHEY_SIMPLEX, .45, 1
    row_height, padding = 19, 3
    fitted = []
    for text, color in lines:
        while text and cv2.getTextSize(text, font, size, thickness)[0][0] > width - 2 * padding:
            text = text[:-1]
        fitted.append((text, color))
    block_width = min(width, max(cv2.getTextSize(t, font, size, thickness)[0][0]
                                 for t, _ in fitted) + 2 * padding)
    block_height = len(fitted) * row_height
    x = max(0, min(int(box[0]), width - block_width))
    top = int(box[1]) - block_height
    if top < 0:
        top = int(box[1]) + 2
    top = max(0, min(top, height - block_height))
    cv2.rectangle(image, (x, top), (x + block_width - 1, top + block_height - 1), (25, 25, 25), -1)
    for row, (text, color) in enumerate(fitted):
        cv2.putText(image, text, (x + padding, top + row * row_height + 14),
                    font, size, color, thickness, cv2.LINE_AA)
