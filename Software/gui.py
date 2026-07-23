import sys
import numpy as np
import pyqtgraph.opengl as gl
from PyQt6.QtCore import QSize, Qt
from PyQt6.QtWidgets import QApplication, QGridLayout, QHBoxLayout, QMainWindow, QPushButton, QWidget, QVBoxLayout, QGroupBox, QFormLayout, QLabel
from PyQt6.QtGui import QFont

font = QFont()
font.setPointSize(12)
buttonFont = QFont()
buttonFont.setPointSize(14)

# Subclass QMainWindow to customize your application's main window
class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("Acculux")

        layout = QVBoxLayout()

        self.plot3D = PyQtGraph3DWindow()
        layout.addWidget(self.plot3D, stretch=50)

        self.bottom = bottomWindow()
        layout.addWidget(self.bottom)

        widget = QWidget()
        widget.setLayout(layout)
        self.setCentralWidget(widget)

class bottomWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.setFont(font)
        layout = QHBoxLayout(self)

        self.statusLight = QLabel()
        self.statusLight.setFixedSize(30, 30)
        self.statusLight.setStyleSheet("""
        background-color: green;
        border-radius: 15px;
        border: 2px solid black;
    """)
        self.connectedLabel = QLabel("Connected:")
        self.connectedLabel.setFont(buttonFont)
        layout.addWidget(self.connectedLabel)
        layout.addWidget(self.statusLight)
        self.calibrateButton = QPushButton("Calibrate")
        self.calibrateButton.setStyleSheet("""
            QPushButton {
                background-color: blue;
                color: white;
                font-size: 24px;
                border: 2px solid black;                       
            }
        """)
        self.calibrateButton.setFixedSize(250, 100)
        # self.calibrateButton.clicked.connect(self.calibrate) 
        layout.addWidget(self.calibrateButton)

        self.scanButton = QPushButton("Scan")
        self.scanButton.setStyleSheet("""
            QPushButton {
                background-color: orange;
                color: black;
                font-size: 24px;
                border: 2px solid black;                       
            }
        """)
        self.scanButton.setFixedSize(250, 100)
        # self.scanButton.clicked.connect(self.scan) 
        layout.addWidget(self.scanButton)

        self.saveButton = QPushButton("Save")
        self.saveButton.setStyleSheet("""
            QPushButton {
                background-color: green;
                color: black;
                font-size: 24px;
                border: 2px solid black;                       
            }
        """)
        self.saveButton.setFixedSize(250, 100)
        # self.saveButton.clicked.connect(self.save)
        layout.addWidget(self.saveButton)

        self.sensorPanel = SensorPanel()
        layout.addWidget(self.sensorPanel, stretch=2)

        # def calibrate(self):
        #     print("You clicked the calibrate button!")

        # def scan(self):
        #     print("You clicked the scan button!")
            
        # def save(self): # Exports results as a .csv
        #     print("You clicked the save button!")


class PyQtGraph3DWindow(QWidget):
    def __init__(self):
        super().__init__()
        layout = QVBoxLayout(self)

        # 1. Create the 3D View Widget
        self.view = gl.GLViewWidget()
        self.view.setCameraPosition(
            distance=14,
            elevation=45,
            azimuth=0
        )
        layout.addWidget(self.view)

        # 2. Add a spatial grid for visual reference
        grid = gl.GLGridItem()
        grid.setSize(10, 10, 10)
        grid.setSpacing(1, 1, 1)
        self.view.addItem(grid)

        # Add reference semi-circle
        radius = 3
        n_theta = 20      # Around z-axis
        n_phi = 12        # From top to equator
        theta = np.linspace(0, 2*np.pi, n_theta)
        phi = np.linspace(0, np.pi/2, n_phi)   # Only upper half
        theta, phi = np.meshgrid(theta, phi)

        # Cartesian coordinates
        x = radius * np.sin(phi) * np.cos(theta)
        y = radius * np.sin(phi) * np.sin(theta)
        z = radius * np.cos(phi)

        # Convert grid to vertices
        verts = np.vstack([x.ravel(), y.ravel(), z.ravel()]).T

        # Build triangular faces
        faces = []
        for i in range(n_phi - 1):
            for j in range(n_theta - 1):
                a = i * n_theta + j
                b = a + 1
                c = a + n_theta
                d = c + 1

                faces.append([a, c, b])
                faces.append([b, c, d])

        faces = np.array(faces)

        mesh = gl.MeshData(vertexes=verts, faces=faces)

        item = gl.GLMeshItem(
            meshdata=mesh,
            smooth=True,
            drawEdges=True,
            drawFaces=True,
            color=(1.0, 0.76, 0.667, 0.6),
            edgeColor=(0, 0, 0, 0.6),
            glOptions='translucent'
        )
        self.view.addItem(item)

        self.sensor_point = gl.GLScatterPlotItem(
            pos=np.array([[0, 0, radius]]),   # Initial position
            color=(0.5, 0, 0, 1),          # Red
            size=20
        )
        self.view.addItem(self.sensor_point)

        self.radius = radius

    def update_orientation(self, roll, pitch):
        """
        Update sensor position using orientation angles.

        roll:
            rotation about x-axis (degrees)

        pitch:
            rotation about y-axis (degrees)
        """

        roll = np.deg2rad(roll)
        pitch = np.deg2rad(pitch)

        r = self.radius

        # Convert spherical coordinates
        x = r * np.sin(pitch)
        y = r * np.sin(roll)
        z = r * np.sqrt(
            max(0, 1 - np.sin(pitch)**2 - np.sin(roll)**2)
        )

        self.sensor_point.setData(
            pos=np.array([[x, y, z]])
        )



class SensorPanel(QGroupBox):
    def __init__(self):
        super().__init__("Sensor Readings")
        self.setFont(font)
        layout = QGridLayout()

        # Orientation
        layout.addWidget(QLabel("Roll:"), 0, 0)
        layout.addWidget(QLabel("Pitch:"), 1, 0)
        layout.addWidget(QLabel("Yaw:"), 2, 0)
        
        self.rollLabel = QLabel("0.0°")
        self.pitchLabel = QLabel("0.0°")
        self.yawLabel = QLabel("0.0°")

        layout.addWidget(self.rollLabel, 0, 1)
        layout.addWidget(self.pitchLabel, 1, 1)
        layout.addWidget(self.yawLabel, 2, 1)

        # Position
        layout.addWidget(QLabel("X:"), 0, 2)
        layout.addWidget(QLabel("Y:"), 1, 2)
        layout.addWidget(QLabel("Z:"), 2, 2)

        self.XLabel = QLabel("0.00")
        self.YLabel = QLabel("0.00")
        self.ZLabel = QLabel("0.00")

        layout.addWidget(self.XLabel, 0, 3)
        layout.addWidget(self.YLabel, 1, 3)
        layout.addWidget(self.ZLabel, 2, 3)

        layout.addWidget(QLabel("Force:"), 0, 4)
        self.ForceLabel = QLabel("0.00 N")
        layout.addWidget(self.ForceLabel, 1, 4)

        self.setLayout(layout)

    def update_angles(self, roll, pitch, yaw):
        self.rollLabel.setText(f"{roll:.2f}°")
        self.pitchLabel.setText(f"{pitch:.2f}°")
        self.yawLabel.setText(f"{yaw:.2f}°")

# # You need one (and only one) QApplication instance per application.
# # Pass in sys.argv to allow command line arguments for your app.
# # If you know you won't use command line arguments QApplication([]) works too.
# app = QApplication(sys.argv)

# # Create a Qt widget, which will be our window.
# window = MainWindow()
# window.resize(1200, 800)
# window.showMaximized()  # IMPORTANT!!!!! Windows are hidden by default.

# # Start the event loop.
# app.exec()


# # Your application won't reach here until you exit and the event
# # loop has stopped.

def create_gui():

    app = QApplication(sys.argv)

    window = MainWindow()

    window.resize(1200,800)
    window.showMaximized()

    return app, window