#!/usr/bin/python
from __future__ import print_function, division
import os
import shutil
import yaml
from cv_bridge import CvBridge
import numpy as np
import cv2
from sensor_msgs.msg import Image

import sys
from PySide.QtCore import *
from PySide.QtGui import *
import rospy
import rospkg
import time


class CameraCalibrator(QMainWindow):
    images_updated = Signal()
    status_bar_update = Signal(str)

    def __init__(self, img_topic=None):
        super(CameraCalibrator, self).__init__()

        self.img_cnt = 0
        self.recording = False
        self.corners = []
        self.last_time = time.time()
        self.object_points = []

        self.image = None

        self.image_label = None

        self.chb_size_x_spin = None
        self.chb_size_y_spin = None

        self.chb_dimensions_spin = None
        self.start_calib_btn = None

        self.images_updated.connect(self.redraw_images)
        self.status_bar_update.connect(self.statusBar().showMessage)

        self.initUI()
        if img_topic is None:
            img_topic = '/images'
            rospy.logwarn('No image topic was provided. Listening to {}'.format(img_topic))
        else:
            rospy.loginfo('Listening to {}'.format(img_topic))
            self.status_bar_update.emit('Listening to {}'.format(img_topic))
        rospy.Subscriber(img_topic, Image, self.image_cb, queue_size=1)

    def initUI(self):

        vbox_top = QVBoxLayout()

        self.start_calib_btn = QPushButton('Start recording')
        self.start_calib_btn.clicked.connect(self.recording_button_clicked)

        hbox = QHBoxLayout()
        hbox.addStretch(1)
        hbox.addWidget(self.start_calib_btn)
        vbox_top.addLayout(hbox)

        self.chb_size_x_spin = QSpinBox()
        self.chb_size_x_spin.setMinimum(3)
        self.chb_size_x_spin.setValue(5)
        self.chb_size_y_spin = QSpinBox()
        self.chb_size_y_spin.setMinimum(3)
        self.chb_size_y_spin.setValue(8)
        chb_size_label = QLabel('Checkerboard size:')
        chb_size_label.setToolTip('Number of inner corners of the checker borad.')
        chb_size_x = QLabel('x')
        self.chb_dimensions_spin = QSpinBox()
        self.chb_dimensions_spin.setMaximum(999)
        self.chb_dimensions_spin.setMinimum(1)
        self.chb_dimensions_spin.setValue(120)
        chb_dim_label = QLabel('Checkerboard dimensions:')
        chb_dim_unit_label = QLabel('mm')

        hbox = QHBoxLayout()
        hbox.addWidget(chb_size_label)
        hbox.addWidget(self.chb_size_x_spin)
        hbox.addWidget(chb_size_x)
        hbox.addWidget(self.chb_size_y_spin)
        hbox.addStretch(1)
        hbox.addWidget(chb_dim_label)
        hbox.addWidget(self.chb_dimensions_spin)
        hbox.addWidget(chb_dim_unit_label)

        vbox_top.addLayout(hbox)
        vbox_top.addStretch(1)

        self.image_label = QLabel(self)

        hbox = QHBoxLayout()
        hbox.addStretch(1)
        hbox.addWidget(self.image_label)
        hbox.addStretch(1)
        vbox_top.addLayout(hbox)

        central_widget = QWidget()

        central_widget.setLayout(vbox_top)
        self.setCentralWidget(central_widget)

        self.setGeometry(300, 300, 300, 300)
        self.setWindowTitle('Camera Calibrator')
        self.show()

    def keyPressEvent(self, e):

        if e.key() == Qt.Key_Escape:
            self.close()

        super(CameraCalibrator, self).keyPressEvent(e)

    def extract_checkerboard_and_draw_corners(self, image, chbrd_size):
        image = CvBridge().imgmsg_to_cv2(image, 'mono8')
        image_color = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
        ret, corners = cv2.findChessboardCorners(image_color, chbrd_size)

        if not ret:
            cv2.putText(image_color, 'Checkerboard not found', (0, image.shape[0] - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255))

        cv2.drawChessboardCorners(image_color, chbrd_size, corners, ret)

        return ret, corners, image_color

    def image_cb(self, data):

        self.img_cnt += 1

        chbrd_size = (self.chb_size_y_spin.value(), self.chb_size_x_spin.value())

        if not self.img_cnt % 10:
            # left image
            ret_left, corners_left, self.image = self.extract_checkerboard_and_draw_corners(data, chbrd_size)

            if self.recording and ret_left:
                elapsed_time = time.time() - self.last_time
                if elapsed_time > 1:
                    self.last_time = time.time()
                    self.corners.append(corners_left)
                    objp = np.zeros((chbrd_size[0] * chbrd_size[1], 3), np.float32)
                    objp[:, :2] = np.mgrid[0:chbrd_size[0], 0:chbrd_size[1]].T.reshape(-1, 2) * self.chb_dimensions_spin.value() / 1000.0
                    # objp[:, :2] = np.mgrid[0:chbrd_size[0], 0:chbrd_size[1]].T.reshape(-1, 2)
                    self.object_points.append(objp)

                    self.status_bar_update.emit('{} frames captured'.format(len(self.corners)))

            self.images_updated.emit()

    def redraw_images(self):
        qim = QImage(self.image.data, self.image.shape[1], self.image.shape[0], self.image.strides[0], QImage.Format_RGB888)
        pixmap = QPixmap()
        pixmap.convertFromImage(qim.rgbSwapped())
        self.image_label.setPixmap(pixmap)

    def recording_button_clicked(self):
        if self.recording:
            self.stop_recording()
        else:
            self.start_recording()

    def start_recording(self):

        self.chb_dimensions_spin.setEnabled(False)
        self.chb_size_x_spin.setEnabled(False)
        self.chb_size_y_spin.setEnabled(False)
        self.start_calib_btn.setText('Stop recording')

        self.corners = []
        self.status_bar_update.emit('{} frames captured'.format(len(self.corners)))

        self.recording = True

    def stop_recording(self):
        self.chb_dimensions_spin.setEnabled(True)
        self.chb_size_x_spin.setEnabled(True)
        self.chb_size_y_spin.setEnabled(True)

        self.start_calib_btn.setText('Start recording')

        self.recording = False

        min_frames = 1  # todo
        if len(self.corners) < min_frames:
            QMessageBox.critical(self, "Not enough images",
                                        """You have not recorded enough images of the checkerboard. Please record at least {} images to calibrate""".format(min_frames),
                                        QMessageBox.Ok)
            return

        self.status_bar_update.emit('Calibrating left camera intrinsics')

        rms, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(self.object_points, self.corners, (self.image.shape[1], self.image.shape[0]), flags=cv2.CALIB_ZERO_TANGENT_DIST)
        print('Reprojection error {}'.format(rms))
        if rms > 1:
            rospy.logwarn('The reprojection error is very large. Take more images from different view points. Make sure That the checkerboard covers all parts of the image in some frames. Take images from close up, far away and shallow angles of attack.')

        thresh = 1  # threshold for RMS warning
        if rms > thresh:
            reply = QMessageBox.warning(self, "High reprojection error",
                                        """The reprojection error is very large.\nRMS reprojection error: {:.2f}.\
                                        \nTake more images from different view points.\
                                        Make sure That the checkerboard covers all parts of the image in some frames.\
                                        \nTake images from close up, far away and shallow angles of attack.\
                                        \n\nWould you like to save this calibration anyway?""".format(rms),
                                        QMessageBox.Yes | QMessageBox.No,
                                        QMessageBox.No)

            if reply == QMessageBox.No:
                return

        calib = dict()
        folder = 'calib/VGA' #default VGA resolution
        if self.image.shape[1] < 321: # QVGA resolution
            folder = 'calib/QVGA'

        calib_path = os.path.join(rospkg.RosPack().get_path('snap_cam'), folder, 'cameraParameters.yaml')
        if not os.path.exists(os.path.dirname(calib_path)):
            rospy.loginfo('Directory does not exist yet. Creating directory.')
            os.makedirs(os.path.dirname(calib_path))

        calib['DistortionModel'] = 'plumb_bob'
        calib['FocalLength'] = [0]*2
        calib['FocalLength'][0] = float(mtx[0][0])
        calib['FocalLength'][1] = float(mtx[1][1])
        calib['PrincipalPoint'] = [0]*2
        calib['PrincipalPoint'][0] = float(mtx[0][2])
        calib['PrincipalPoint'][1] = float(mtx[1][2])
        calib['RadialDistortion'] = [0]*3
        calib['RadialDistortion'][0] = float(dist[0][0])
        calib['RadialDistortion'][1] = float(dist[0][1])
        calib['RadialDistortion'][2] = float(dist[0][4])
        calib['CameraMatrix'] = mtx.tolist()

        with open(calib_path, 'w') as outfile:
            outfile.write(yaml.dump(calib, default_flow_style=None))
            self.status_bar_update.emit('Wrote calibration to {}'.format(calib_path))
            rospy.loginfo('Wrote calibration to {}'.format(calib_path))


if __name__ == '__main__':
    rospy.init_node("CameraCalibrator")

    image_topic = rospy.get_param('~image_topic', None) #todo

    app = QApplication(sys.argv)
    ex = CameraCalibrator(img_topic=image_topic)
    ex.show()
    sys.exit(app.exec_())
