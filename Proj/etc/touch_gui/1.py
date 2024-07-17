"""import sys
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QPushButton, QLabel
from gpiozero import DigitalOutputDevice, DistanceSensor
from threading import Thread
import time

# GPIO 제어 앱 클래스 정의
class GPIOControlApp(QWidget):
    def __init__(self):
        super().__init__()
        self.initUI()  # UI 초기화
        self.initGPIO()  # GPIO 초기화
        self.startDistanceThread()  # 거리 측정 스레드 시작

    # UI 초기화 메소드
    def initUI(self):
        self.setWindowTitle('GPIO Control')  # 창 제목 설정
       
        layout = QVBoxLayout()  # 레이아웃 설정
       
        self.upButton = QPushButton('UP', self)  # 'UP' 버튼 생성
        self.upButton.clicked.connect(self.handleUpButton)  # 버튼 클릭 시 handleUpButton 메소드 호출
        layout.addWidget(self.upButton)  # 레이아웃에 버튼 추가
       
        self.downButton = QPushButton('DOWN', self)  # 'DOWN' 버튼 생성
        self.downButton.clicked.connect(self.handleDownButton)  # 버튼 클릭 시 handleDownButton 메소드 호출
        layout.addWidget(self.downButton)  # 레이아웃에 버튼 추가

        self.stopButton = QPushButton('STOP', self)  # 'STOP' 버튼 생성
        self.stopButton.clicked.connect(self.handleStopButton)  # 버튼 클릭 시 handleStopButton 메소드 호출
        layout.addWidget(self.stopButton)  # 레이아웃에 버튼 추가
       
        self.autoButton = QPushButton('Auto Adjust', self)  # 'Auto Adjust' 버튼 생성
        self.autoButton.clicked.connect(self.handleAutoButton)  # 버튼 클릭 시 handleAutoButton 메소드 호출
        layout.addWidget(self.autoButton)  # 레이아웃에 버튼 추가
       
        self.distanceLabel = QLabel('Distance: -- cm', self)  # 거리 표시 라벨 생성
        layout.addWidget(self.distanceLabel)  # 레이아웃에 라벨 추가
       
        self.setLayout(layout)  # 레이아웃 설정
       
    # GPIO 초기화 메소드
    def initGPIO(self):
        self.pin23 = DigitalOutputDevice(23)  # 핀 23을 디지털 출력으로 설정
        self.pin24 = DigitalOutputDevice(24)  # 핀 24을 디지털 출력으로 설정
        self.sensor = DistanceSensor(echo=27, trigger=17)  # 초음파 거리 센서 설정 (에코 핀 27, 트리거 핀 17)
        self.measurementActive = False  # 거리 측정 활성화 플래그
        self.distanceThreadActive = True  # 거리 측정 스레드 활성화 플래그
       
    # 'UP' 버튼 클릭 핸들러
    def handleUpButton(self):
        self.pin23.on()  # 핀 23 켜기
        self.pin24.off()  # 핀 24 끄기

    # 'DOWN' 버튼 클릭 핸들러
    def handleDownButton(self):
        self.pin23.off()  # 핀 23 끄기
        self.pin24.on()  # 핀 24 켜기

    # 'STOP' 버튼 클릭 핸들러
    def handleStopButton(self):
        self.pin23.off()  # 핀 23 끄기
        self.pin24.off()  # 핀 24 끄기
       
    # 'Auto Adjust' 버튼 클릭 핸들러
    def handleAutoButton(self):
        self.measurementActive = True  # 거리 측정 활성화
        self.autoThread = Thread(target=self.autoAdjust)  # 거리 자동 조절 스레드 생성
        self.autoThread.start()  # 스레드 시작

    # 거리 자동 조절 메소드
    def autoAdjust(self):
        try:
            self.handleUpButton()  # 'UP' 동작 시작
            while self.measurementActive:
                distance = self.sensor.distance * 100  # 거리 측정 (센티미터로 변환)
                if 16 <= distance <= 17:  # 거리가 16~17cm 이내일 경우
                    self.handleStopButton()  # 'STOP' 동작
                    self.measurementActive = False  # 거리 측정 비활성화
                time.sleep(0.1)  # 0.1초 대기
        except KeyboardInterrupt:
            self.measurementActive = False  # 거리 측정 비활성화

    # 거리 측정 스레드 시작 메소드
    def startDistanceThread(self):
        self.distanceThread = Thread(target=self.updateDistance)  # 거리 업데이트 스레드 생성
        self.distanceThread.start()  # 스레드 시작

    # 거리 업데이트 메소드
    def updateDistance(self):
        while self.distanceThreadActive:
            distance = self.sensor.distance * 100  # 거리 측정 (센티미터로 변환)
            self.distanceLabel.setText(f'Distance: {distance:.2f} cm')  # 거리 라벨 업데이트
            time.sleep(0.1)  # 0.1초 대기
       
    # 창 닫기 이벤트 핸들러
    def closeEvent(self, event):
        self.measurementActive = False  # 거리 측정 비활성화
        self.distanceThreadActive = False  # 거리 측정 스레드 비활성화
        if hasattr(self, 'autoThread'):
            self.autoThread.join()  # 자동 조절 스레드 종료 대기
        if hasattr(self, 'distanceThread'):
            self.distanceThread.join()  # 거리 업데이트 스레드 종료 대기
        self.pin23.close()  # 핀 23 닫기
        self.pin24.close()  # 핀 24 닫기
        self.sensor.close()  # 센서 닫기
        event.accept()  # 이벤트 수락

# 메인 함수
if __name__ == '__main__':
    app = QApplication(sys.argv)  # QApplication 객체 생성
    ex = GPIOControlApp()  # GPIOControlApp 객체 생성
    ex.show()  # 창 표시
    sys.exit(app.exec_())  # 이벤트 루프 실행

"""
import sys
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QPushButton, QLabel
import gpiod
from threading import Thread
import time

# GPIO 제어 앱 클래스 정의
class GPIOControlApp(QWidget):
    def __init__(self):
        super().__init__()
        self.initUI()  # UI 초기화
        self.initGPIO()  # GPIO 초기화
        self.startDistanceThread()  # 거리 측정 스레드 시작

    # UI 초기화 메소드
    def initUI(self):
        self.setWindowTitle('GPIO Control')  # 창 제목 설정

        layout = QVBoxLayout()  # 레이아웃 설정

        self.upButton = QPushButton('UP', self)  # 'UP' 버튼 생성
        self.upButton.clicked.connect(self.handleUpButton)  # 버튼 클릭 시 handleUpButton 메소드 호출
        layout.addWidget(self.upButton)  # 레이아웃에 버튼 추가

        self.downButton = QPushButton('DOWN', self)  # 'DOWN' 버튼 생성
        self.downButton.clicked.connect(self.handleDownButton)  # 버튼 클릭 시 handleDownButton 메소드 호출
        layout.addWidget(self.downButton)  # 레이아웃에 버튼 추가

        self.stopButton = QPushButton('STOP', self)  # 'STOP' 버튼 생성
        self.stopButton.clicked.connect(self.handleStopButton)  # 버튼 클릭 시 handleStopButton 메소드 호출
        layout.addWidget(self.stopButton)  # 레이아웃에 버튼 추가

        self.autoButton = QPushButton('Auto Adjust', self)  # 'Auto Adjust' 버튼 생성
        self.autoButton.clicked.connect(self.handleAutoButton)  # 버튼 클릭 시 handleAutoButton 메소드 호출
        layout.addWidget(self.autoButton)  # 레이아웃에 버튼 추가

        self.distanceLabel = QLabel('Distance: -- cm', self)  # 거리 표시 라벨 생성
        layout.addWidget(self.distanceLabel)  # 레이아웃에 라벨 추가

        self.setLayout(layout)  # 레이아웃 설정

    # GPIO 초기화 메소드
    def initGPIO(self):
        self.chip = gpiod.Chip('0')  # 'gpiochip0'을 '0'으로 변경
        self.pin23 = self.chip.get_line(23)
        self.pin24 = self.chip.get_line(24)
        self.pin23.request(consumer='app', type=gpiod.LINE_REQ_DIR_OUT)
        self.pin24.request(consumer='app', type=gpiod.LINE_REQ_DIR_OUT)
        # 거리 센서 초기화 부분은 생략 (추가로 구현 필요)
        self.measurementActive = False  # 거리 측정 활성화 플래그
        self.distanceThreadActive = True  # 거리 측정 스레드 활성화 플래그

    # 'UP' 버튼 클릭 핸들러
    def handleUpButton(self):
        self.pin23.set_value(1)  # 핀 23 켜기
        self.pin24.set_value(0)  # 핀 24 끄기

    # 'DOWN' 버튼 클릭 핸들러
    def handleDownButton(self):
        self.pin23.set_value(0)  # 핀 23 끄기
        self.pin24.set_value(1)  # 핀 24 켜기

    # 'STOP' 버튼 클릭 핸들러
    def handleStopButton(self):
        self.pin23.set_value(0)  # 핀 23 끄기
        self.pin24.set_value(0)  # 핀 24 끄기

    # 'Auto Adjust' 버튼 클릭 핸들러
    def handleAutoButton(self):
        self.measurementActive = True  # 거리 측정 활성화
        self.autoThread = Thread(target=self.autoAdjust)  # 거리 자동 조절 스레드 생성
        self.autoThread.start()  # 스레드 시작

    # 거리 자동 조절 메소드
    def autoAdjust(self):
        try:
            self.handleUpButton()  # 'UP' 동작 시작
            while self.measurementActive:
                # 거리 측정 (센티미터로 변환)
                distance = 0  # 거리 센서 측정값 (추가 구현 필요)
                if 16 <= distance <= 17:  # 거리가 16~17cm 이내일 경우
                    self.handleStopButton()  # 'STOP' 동작
                    self.measurementActive = False  # 거리 측정 비활성화
                time.sleep(0.1)  # 0.1초 대기
        except KeyboardInterrupt:
            self.measurementActive = False  # 거리 측정 비활성화

    # 거리 측정 스레드 시작 메소드
    def startDistanceThread(self):
        self.distanceThread = Thread(target=self.updateDistance)  # 거리 업데이트 스레드 생성
        self.distanceThread.start()  # 스레드 시작

    # 거리 업데이트 메소드
    def updateDistance(self):
        while self.distanceThreadActive:
            # 거리 측정 (센티미터로 변환)
            distance = 0  # 거리 센서 측정값 (추가 구현 필요)
            self.distanceLabel.setText(f'Distance: {distance:.2f} cm')  # 거리 라벨 업데이트
            time.sleep(0.1)  # 0.1초 대기

    # 창 닫기 이벤트 핸들러
    def closeEvent(self, event):
        self.measurementActive = False  # 거리 측정 비활성화
        self.distanceThreadActive = False  # 거리 측정 스레드 비활성화
        if hasattr(self, 'autoThread'):
            self.autoThread.join()  # 자동 조절 스레드 종료 대기
        if hasattr(self, 'distanceThread'):
            self.distanceThread.join()  # 거리 업데이트 스레드 종료 대기
        self.pin23.set_value(0)  # 핀 23 끄기
        self.pin24.set_value(0)  # 핀 24 끄기
        event.accept()  # 이벤트 수락

# 메인 함수
if __name__ == '__main__':
    app = QApplication(sys.argv)  # QApplication 객체 생성
    ex = GPIOControlApp()  # GPIOControlApp 객체 생성
    ex.show()  # 창 표시
    sys.exit(app.exec_())  # 이벤트 루프 실행