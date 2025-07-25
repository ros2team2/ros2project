import os
import sys
import threading
import pyaudio
import json
import socket
from vosk import Model, KaldiRecognizer
from PyQt5.QtGui import QIcon
from PyQt5.QtCore import Qt, QSize
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QPushButton, QLabel
import time
import cv2
from ultralytics import YOLO

class VoiceRecognizer(QWidget):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("당신의 친구 BOOGIE")
        self.setFixedSize(400, 400)

        self.init_ui()

        # 음성 인식 설정
        self.model_path = os.path.join(os.path.dirname(__file__), "models/vosk-model-en-us-0.42-gigaspeech")
        self.yolo = YOLO(os.path.join(os.path.dirname(__file__), "models/best.onnx"))
        self.sample_rate = 8000
        self.chunk_size = 1024
        self.running = False

        # TCP 클라이언트 설정
        self.server_host = "192.168.0.95"  # 실제 서버 IP
        self.server_port = 12345
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        max_attempts = 3
        for attempt in range(max_attempts):
            try:
                self.client_socket.connect((self.server_host, self.server_port))
                print(f"Connected to server at {self.server_host}:{self.server_port}")
                break
            except Exception as e:
                print(f"Attempt {attempt + 1} failed: {e}")
                if attempt == max_attempts - 1:
                    self.label.setText(f"서버 연결 실패: {e}")
                    self.mic_button.setEnabled(False)
                time.sleep(1)

        if not os.path.exists(self.model_path):
            self.label.setText("모델 경로가 존재하지 않습니다.")
            self.mic_button.setEnabled(False)
        else:
            self.model = Model(self.model_path)
            self.rec = KaldiRecognizer(self.model, self.sample_rate)

    def init_ui(self):
        layout = QVBoxLayout()
        layout.setAlignment(Qt.AlignCenter)

        self.label = QLabel("아래 버튼을 눌러 음성 인식을 시작하세요.")
        self.label.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.label)

        self.mic_button = QPushButton()
        self.mic_button.setIcon(QIcon.fromTheme("microphone"))
        self.mic_button.setText("마이크")
        self.mic_button.setIconSize(QSize(50, 50))
        self.mic_button.setFixedHeight(40)
        self.mic_button.clicked.connect(self.toggle_recognition)
        self.mic_button.setStyleSheet("""
            QPushButton {
                background-color: #2196F3;
                color: white;
                border: none;
                border-radius: 20px;
                font-weight: bold;
                padding: 8px 16px;
            }
            QPushButton:hover {
                background-color: #1976D2;
            }
        """)
        layout.addWidget(self.mic_button, alignment=Qt.AlignCenter)

        self.result_label = QLabel("")
        self.result_label.setAlignment(Qt.AlignCenter)
        self.result_label.setStyleSheet("color: green; font-size: 16px;")
        layout.addWidget(self.result_label)

        self.setLayout(layout)

    def toggle_recognition(self):
        if not self.running:
            self.start_recognition()
        else:
            self.stop_recognition()

    def start_recognition(self):
        self.running = True
        self.mic_button.setText("중지")
        self.label.setText("음성 인식 중... 마이크에 말하세요.")
        self.result_label.setText("")
        
        # 서버에 음성 인식 시작 신호 전송
        try:
            self.client_socket.send(json.dumps({"command": "start_recognition"}).encode('utf-8'))
            print("Sent start_recognition signal")
        except Exception as e:
            print(f"Failed to send start_recognition signal: {e}")
            self.label.setText(f"신호 전송 실패: {e}")
            self.running = False
            self.mic_button.setText("🎙️")
            return

        self.thread = threading.Thread(target=self.recognize)
        self.thread.start()

    def stop_recognition(self):
        self.running = False
        self.mic_button.setText("마이크")
        self.label.setText("음성 인식이 중지되었습니다.")

    def recognize(self):
        p = pyaudio.PyAudio()
        stream = p.open(format=pyaudio.paInt16,
                        channels=1,
                        rate=self.sample_rate,
                        input=True,
                        frames_per_buffer=self.chunk_size)

        stream.start_stream()

        try:
            while self.running:
                data = stream.read(self.chunk_size, exception_on_overflow=False)
                if self.rec.AcceptWaveform(data):
                    result_json = self.rec.Result()
                    result_dict = json.loads(result_json)
                    self.text_process(result_dict.get("text", ""))
        except Exception as e:
            print(f"오류 발생: {e}")
            self.result_label.setText(f"오류: {e}")
        finally:
            stream.stop_stream()
            stream.close()
            p.terminate()

    def text_process(self, text):
        print(f"인식된 텍스트: {text}")
        token = text.split()
        for word in token:
            if word in ["boogie", "cookie", "pookie", "ookie"]:
                self.result_label.setText("주인님 오늘의 느낌은?")
                emotion_data = self.yolo_emotion_detection()
                if emotion_data:
                    try:
                        # 감정 데이터 전송
                        emotion_json = json.dumps(emotion_data)
                        self.client_socket.send(emotion_json.encode('utf-8'))
                        print(f"감정 전송: {emotion_json}")
                        # 감정 전송 후 종료 신호 전송
                        time.sleep(0.1)  # 짧은 지연
                        self.client_socket.send(json.dumps({"command": "end_recognition"}).encode('utf-8'))
                        print("Sent end_recognition signal")
                    except Exception as e:
                        print(f"감정 전송 실패: {e}")
                        self.result_label.setText(f"전송 실패: {e}")
                self.stop_recognition()
                return
        self.result_label.setText("")

    def yolo_emotion_detection(self):
        cap = cv2.VideoCapture(0)
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame.")
            self.result_label.setText("카메라 프레임 캡처 실패")
            cap.release()
            return None

        gray_image = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray_image_3d = cv2.merge([gray_image, gray_image, gray_image])

        results = self.yolo(gray_image_3d)
        result = results[0]

        emotion_data = None
        for box in result.boxes:
            conf = float(box.conf[0])
            class_id = int(box.cls[0])
            class_name = self.yolo.names[class_id]
            emotion_data = {
                "emotion": class_name,
                "confidence": conf
            }
            if class_name == "Sad":
                self.result_label.setText("너무 슬픈 표정 짓지 말아주세요.")
            elif class_name == "Happy":
                self.result_label.setText("오늘도 화이팅입니다.")
            elif class_name == "Neutral":
                self.result_label.setText("웃어보면 어떨까요???")

            break

        cap.release()
        return emotion_data

    def closeEvent(self, event):
        self.running = False
        try:
            self.client_socket.send(json.dumps({"command": "end_recognition"}).encode('utf-8'))
            print("Sent end_recognition signal on close")
        except:
            pass
        self.client_socket.close()
        event.accept()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = VoiceRecognizer()
    window.show()
    sys.exit(app.exec_())