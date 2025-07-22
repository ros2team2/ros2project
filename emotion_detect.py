from ultralytics import YOLO
import cv2

model = YOLO('best.onnx') 

cap = cv2.VideoCapture(0)

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        print("Failed to grab frame.")
        break  

    gray_image = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # gray_image = cv2.cvtColor(frame, cv2.COLOR_BayerRG2BGR)
    
    gray_image_3d = cv2.merge([gray_image, gray_image, gray_image]) 
    
    results = model(gray_image_3d)
    result = results[0]

    for box in result.boxes:
        # 좌표 추출
        conf = float(box.conf[0])
        class_id = int(box.cls[0])
        class_name = model.names[class_id]

        # 예측률 출력
        print(f'표정 이름: {class_name}, 예측율: {conf:.2f}')

    try:
        annotated_frame = result.plot()
    except AttributeError:
        print("Error: plot() method not available for results.")
        break
    
    cv2.imshow('YOLO Inference', annotated_frame)
    
    if cv2.waitKey(1) == 27: 
        break

cap.release()
cv2.destroyAllWindows()
