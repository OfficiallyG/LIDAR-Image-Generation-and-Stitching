import sys
from PyQt6.QtWidgets import QApplication, QMainWindow


class LidarWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("LIDAR Image Generation & Stitching")
        self.setGeometry(250,250,500,500)

def main():
    #creating the application
    lidar_app = QApplication(sys.argv)

    #create application window
    lidar_main = LidarWindow()
    lidar_main.show()
    sys.exit(lidar_app.exec())

if __name__ == "__main__":
    main()
#test comment