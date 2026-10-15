"""Original-pixel annotations; no input image persistence."""
import cv2


def annotate(image, kind, result, status):
    canvas = image.copy()
    rows = result.get('defects' if kind == 'pcb' else 'detections', [])
    for row in rows:
        x1, y1, x2, y2 = [round(v) for v in row['xyxy']]
        cv2.rectangle(canvas, (x1, y1), (x2, y2), (0, 180, 255), 2)
        cv2.putText(canvas, '%s %.2f' % (row['name'], row['score']),
                    (max(0, x1), max(16, y1)), cv2.FONT_HERSHEY_SIMPLEX, .5, (0, 180, 255), 1)
    roi = result.get('roi')
    box = roi.get('box_original') if isinstance(roi, dict) else None
    if isinstance(box, list) and len(box) == 4:
        x1, y1, x2, y2 = [round(v) for v in box]
        cv2.rectangle(canvas, (x1, y1), (x2, y2), (0, 255, 0), 2)
    cv2.putText(canvas, kind + ': ' + status, (8, 22), cv2.FONT_HERSHEY_SIMPLEX,
                .6, (0, 255, 0), 2)
    height, width = canvas.shape[:2]
    if max(height, width) > 1280:
        ratio = 1280 / max(height, width)
        canvas = cv2.resize(canvas, (round(width * ratio), round(height * ratio)))
    ok, encoded = cv2.imencode('.jpg', canvas, [cv2.IMWRITE_JPEG_QUALITY, 90])
    if not ok:
        raise ValueError('annotation_encoding_failed')
    return encoded.tobytes(), canvas.shape[1], canvas.shape[0]
