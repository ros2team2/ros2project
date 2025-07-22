import os
import sys
import threading
import pyaudio
from vosk import Model, KaldiRecognizer

from PyQt5.QtGui import QIcon
from PyQt5.QtCore import Qt, QSize

from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QPushButton,
    QTextEdit, QLabel, QHBoxLayout, QSizePolicy
)


class VoiceRecognizer(QWidget):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("VOSK 음성 인식기")
        self.setFixedSize(400, 400)  # 창 크기 고정

        self.init_ui()

        # 음성 인식 관련 변수
        self.model_path = "models/vosk-model-en-us-0.42-gigaspeech"
        self.sample_rate = 8000
        self.chunk_size = 1024
        self.running = False

        if not os.path.exists(self.model_path):
            self.label.setText("❌ 모델 경로가 존재하지 않습니다.")
            self.mic_button.setEnabled(False)
        else:
            self.model = Model(self.model_path)
            self.rec = KaldiRecognizer(self.model, self.sample_rate)

    def init_ui(self):
        layout = QVBoxLayout()
        layout.setAlignment(Qt.AlignCenter)

        self.label = QLabel("🎤 아래 버튼을 눌러 음성 인식을 시작하세요.")
        self.label.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.label)

        self.text_edit = QTextEdit()
        self.text_edit.setReadOnly(True)
        layout.addWidget(self.text_edit)

        # 마이크 버튼 (중앙 배치)
        self.mic_button = QPushButton()
        self.mic_button.setIcon(QIcon.fromTheme("microphone"))
        self.mic_button.setText("마이크")
        self.mic_button.setIconSize(QSize(24, 24))
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

        self.setLayout(layout)

    def toggle_recognition(self):
        if not self.running:
            self.start_recognition()
        else:
            self.stop_recognition()

    def start_recognition(self):
        self.running = True
        self.mic_button.setText(" 🔇 중지")
        self.label.setText("🟢 음성 인식 중... 마이크에 말하세요.")
        self.text_edit.clear()

        self.thread = threading.Thread(target=self.recognize)
        self.thread.start()

    def stop_recognition(self):
        self.running = False
        self.mic_button.setText(" 음성 인식 시작")
        self.label.setText("⏹️ 음성 인식이 중지되었습니다.")

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
                    result = self.rec.Result()
                    text = eval(result)["text"]
                    if text.strip():
                        self.text_edit.append(f"{text}")
        except Exception as e:
            self.text_edit.append(f"❗ 오류 발생: {e}")
        finally:
            stream.stop_stream()
            stream.close()
            p.terminate()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = VoiceRecognizer()
    window.show()
    sys.exit(app.exec_())
