#!/usr/bin/env python
import rospy
import rviz
import math
import time
import numpy
import random
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3

from PyQt5 import QtGui, QtCore, QtWidgets
from PyQt5.QtWidgets import QMainWindow, QLabel, QVBoxLayout, QHBoxLayout, QWidget, QGridLayout, QComboBox, QGraphicsView, QGraphicsScene
from PyQt5.QtGui import QImage, QPixmap;
from PyQt5.QtCore import QSize, Qt, pyqtSlot, pyqtSignal, QTimer

from matplotlib.backends.backend_qt5agg import FigureCanvas
from matplotlib.figure import Figure, SubplotParams
import matplotlib.pyplot as plt

#GUI class
class myWidget( QWidget ):
    def __init__(self):
        QWidget.__init__(self)
        #jackal values 
        self.num_jackals = 3
        self.currentJackal = 0
        self.jackal_names = ["Disconnected"]
        self.setup = False
        #start locations of jackal in global coordinates
        self.jackal_spawn_x = [0,2,-2]
        self.jackal_spawn_y = [0,0,0]
        # number of non jackal objects (ex: 1 ground plane)
        self.otherObjects = 1
        #intial linear and angular speed
        self.lin = 1
        self.ang = 1
        #position values (size determined by number of jackals)

        #gazebo data
        #self.gazebo_loc = [[ [0 for col in range(6)] for col in range(self.num_jackals)] for row in range(1)]
        self.gazebo_loc = numpy.zeros((self.num_jackals,6,1))
        self.odom_loc = numpy.zeros((self.num_jackals,3,1))
        self.odom_buffer = numpy.zeros((self.num_jackals,3))
        #rospy.loginfo(self.gazebo_loc[0])
        #tmp = numpy.ones((self.num_jackals,6))
        #self.gazebo_loc = numpy.dstack((self.gazebo_loc,tmp))
        #rospy.loginfo(self.gazebo_loc)


        self.x_odom = numpy.zeros(self.num_jackals,dtype=float)
        self.y_odom = numpy.zeros(self.num_jackals,dtype=float)
        self.yaw_odom = numpy.zeros(self.num_jackals,dtype=float)
        self.subbed = False
        self.map_counter = 0
        self.map_x_odom = numpy.zeros(50,dtype=float)
        self.map_y_odom = numpy.zeros(50,dtype=float)
        #label formating variables
        self.old_vel = 1
        self.old_ang = 1

        self.controllerMain()


    def controllerMain(self):
        #ros subscriber
        self.gazebo_sub = rospy.Subscriber("/gazebo/model_states", ModelStates, self.findLoc)
        self.pub_vel = rospy.Publisher('jackal0/jackal_velocity_controller/cmd_vel',Twist, queue_size = 1)
        #while not rospy.is_shutdown():
        #create jackal controls
        self.createPushButtons()
        #creating speed controls
        self.createSpeedControls()
        #creating odometry labels
        self.createOdomLabels()
        #create jackal selector
        self.createSelector()
        #create dynamic map
        self.createMap()
        #connect all controls
        self.connectControls()
        #create pyqt layout
        self.createLayout()
            #rospy.spin()

    #creates Qt layout
    def createLayout(self):
        #turning controls layout
        control_box = QGridLayout()
        control_box.addWidget(self.pushButtonForwards,0,1)
        control_box.addWidget(self.pushButtonLeft,1,0)
        control_box.addWidget(self.pushButtonRight,1,2)
        control_box.addWidget(self.pushButtonBackwards,2,1)   
        #speed controls layout    
        speed_box = QGridLayout()
        speed_box.addWidget(self.qdialspeed,1,0)
        speed_box.addWidget(self.qdialAngspeed,1,1)
        speed_box.addWidget(self.labelVel,0,0)
        speed_box.addWidget(self.labelAngVel,0,1)
        #side controls
        side_layout = QVBoxLayout()
        side_layout.addLayout(control_box)
        side_layout.addLayout(speed_box)
        side_layout.addWidget(self.labelPosition)
        side_layout.addWidget(self.labelOdom)
        side_layout.addWidget(self.robotSelector)
        #main window layout
        main_layout = QHBoxLayout()
        main_layout.addWidget(self.dynamic_canvas)
        main_layout.addLayout(side_layout)
        self.setLayout( main_layout )

    #creates position map
    def createMap(self):
        self.dynamic_canvas = FigureCanvas(Figure(figsize=(5,5)))
        self._dynamic_ax = self.dynamic_canvas.figure.subplots()

    #updates map
    def updateMap(self):
        self._dynamic_ax.clear()
        self._dynamic_ax.set_xlabel("x [m]")
        self._dynamic_ax.set_xlim(-10, 10)
        self._dynamic_ax.set_ylabel("y [m]")
        self._dynamic_ax.set_ylim(-10, 10)
        self._dynamic_ax.set_title(self.jackal_names[self.currentJackal] + " Map")
        self._dynamic_ax.grid()
        self._dynamic_ax.plot(self.gazebo_loc[self.currentJackal,0,1:],self.gazebo_loc[self.currentJackal,1,1:],label = '1')
        self._dynamic_ax.plot(self.odom_loc[self.currentJackal,0,1:],self.odom_loc[self.currentJackal,1,1:],label = '3')
        self._dynamic_ax.scatter(self.gazebo_loc[self.currentJackal,0,-1],self.gazebo_loc[self.currentJackal,1,-1],label='2')
        self._dynamic_ax.scatter(self.odom_loc[self.currentJackal,0,-1],self.odom_loc[self.currentJackal,1,-1],label='4')
        self._dynamic_ax.figure.canvas.draw()

    #creates QPushButtons for controls
    def createPushButtons(self):
        #forwards
        self.pushButtonForwards = QtWidgets.QPushButton(self)
        self.pushButtonForwards.setObjectName("pushButtonForwards")
        self.pushButtonForwards.setText("Forwards")
        self.pushButtonForwards.setAutoRepeat(True)
        self.pushButtonForwards.setAutoRepeatDelay(1)
        #backwards
        self.pushButtonBackwards = QtWidgets.QPushButton(self)
        self.pushButtonBackwards.setObjectName("pushButtonBackwards")
        self.pushButtonBackwards.setText("Backwards")
        self.pushButtonBackwards.setAutoRepeat(True)
        self.pushButtonBackwards.setAutoRepeatDelay(1)
        #turn left
        self.pushButtonLeft = QtWidgets.QPushButton(self)
        self.pushButtonLeft.setObjectName("pushButtonLeft")
        self.pushButtonLeft.setText("Left")
        self.pushButtonLeft.setAutoRepeat(True)
        #turn right
        self.pushButtonRight = QtWidgets.QPushButton(self)
        self.pushButtonRight.setObjectName("pushButtonRight")
        self.pushButtonRight.setText("Right")
        self.pushButtonRight.setAutoRepeat(True)        

    #creates QDials for speed controls
    def createSpeedControls(self):
        #linear speed dial
        self.qdialspeed = QtWidgets.QDial(self)
        self.qdialspeed.setObjectName("QDialSpeed")
        self.qdialspeed.setMinimum(1)
        self.qdialspeed.setMaximum(5)
        self.qdialspeed.setValue(self.lin)
        self.qdialspeed.setNotchesVisible(True)        
        #angular speed dial
        self.qdialAngspeed = QtWidgets.QDial(self)
        self.qdialAngspeed.setObjectName("QDialAngSpeed")
        self.qdialAngspeed.setMinimum(1)
        self.qdialAngspeed.setMaximum(5)
        self.qdialAngspeed.setValue(self.ang)
        self.qdialAngspeed.setNotchesVisible(True)
        #label for linear
        self.labelVel = QtWidgets.QLabel(self)
        self.labelVel.setObjectName("labelVel")
        self.labelVel.setText("Linear Speed : {} m/s".format(self.lin))
        self.labelVel.adjustSize()
        #label for angular
        self.labelAngVel = QtWidgets.QLabel(self)
        self.labelAngVel.setObjectName("labelAngVel")
        self.labelAngVel.setText("Angular Speed : {} rad/s".format(self.ang))
        self.labelAngVel.adjustSize()

    #creates position labels
    def createOdomLabels(self):
        #create position label
        self.labelPosition = QtWidgets.QLabel(self)
        self.labelPosition.setObjectName('labelPosition')
        self.labelPosition.setText("Gazebo\nposition: ({0:.2f}, {1:.2f}) [m]\nAngle: {2:.2f} [rad]".format(self.gazebo_loc[self.currentJackal,0,-1],self.gazebo_loc[self.currentJackal,1,-1],self.gazebo_loc[self.currentJackal,5,-1]))
        self.labelPosition.adjustSize()
        #creating odom labels
        self.labelOdom = QtWidgets.QLabel(self)
        self.labelOdom.setObjectName('labelOdom')
        self.labelOdom.setText("Odometry\nposition: ({0:.2f}, {1:.2f}) [m]\nAngle: {2:.2f} [rad]".format(self.x_odom[self.currentJackal],self.y_odom[self.currentJackal],self.yaw_odom[self.currentJackal]))
        self.labelOdom.adjustSize()
        #timer for odom update
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.updateLabelPosition)
        self.timer.start(500) #repeat every 1 sec    
        self.timer2 = QtCore.QTimer()
        self.timer2.timeout.connect(self.updateLabelOdom)
        self.timer2.start(500) #repeat          

    #creates QComboBox to select which robot controled
    def createSelector(self):
        #create robot selctor menu using QComboBox
        self.robotSelector = QtWidgets.QComboBox(self)
        self.robotSelector.insertItems(0,self.jackal_names)

    #populates QComboBox after subscription to gazebo node
    def updateSelector(self):
        self.robotSelector.clear()
        self.robotSelector.insertItems(0,self.jackal_names)
        nodeStr = self.jackal_names[0] + '/jackal_velocity_controller/cmd_vel'
        self.pub_vel = rospy.Publisher(nodeStr,Twist, queue_size = 1)
        node_sub_str = self.jackal_names[self.currentJackal] + '/jackal_velocity_controller/odom'
        self.odom_sub = rospy.Subscriber(node_sub_str, Odometry, self.getOdom)

    #links signals for QT
    def connectControls(self):
        #behavior
        self.pushButtonForwards.pressed.connect(self.moveForward)
        self.pushButtonBackwards.pressed.connect(self.moveBackwards)
        self.pushButtonLeft.pressed.connect(self.moveLeft)
        self.pushButtonRight.pressed.connect(self.moveRight)
        self.qdialspeed.sliderReleased.connect(self.updateLin)
        self.qdialspeed.sliderReleased.connect(self.updateLabel)
        self.qdialAngspeed.sliderReleased.connect(self.updateAng)
        self.qdialAngspeed.sliderReleased.connect(self.updateLabelAng)       
        self.robotSelector.currentIndexChanged.connect(self.updateJackal)

    #update linear speed label
    def updateLabel(self):
        if self.qdialspeed.value() != self.old_vel:
            self.labelVel.setText("Linear Speed : {0:.0f} m/s".format(self.qdialspeed.value()))
            self.labelVel.adjustSize()
        self.old_vel = self.qdialspeed.value()

    #update angular speed label
    def updateLabelAng(self):
        if self.qdialAngspeed.value() != self.old_ang:
            self.labelAngVel.setText("Angular Speed : {0:.0f} rad/s".format(self.qdialAngspeed.value()))
            self.labelVel.adjustSize()
        self.old_ang = self.qdialAngspeed.value()

    def updateLabelPosition(self):
        self.labelPosition.setText("Gazebo\nposition: ({0:.2f}, {1:.2f}) [m]\nAngle: {2:.2f} [rad]".format(self.gazebo_loc[self.currentJackal,0,-1],self.gazebo_loc[self.currentJackal,1,-1],self.gazebo_loc[self.currentJackal,5,-1]))

    #update odometry position label
    def updateLabelOdom(self):
        self.labelOdom.setText("Odometry\nposition: ({0:.2f}, {1:.2f}) [m]\nAngle: {2:.2f} [rad]".format(self.x_odom[self.currentJackal],self.y_odom[self.currentJackal],self.yaw_odom[self.currentJackal]))

    #update current selected jackal from QComboBox
    def updateJackal(self):
        self.odom_sub.unregister()
        self.currentJackal = self.robotSelector.currentIndex()
        self.map_x_odom = numpy.zeros(50,dtype=float)
        self.map_y_odom = numpy.zeros(50,dtype=float)
        self.map_counter = 0
        nodeStr = self.jackal_names[self.currentJackal] + '/jackal_velocity_controller/cmd_vel'
        self.pub_vel = rospy.Publisher(nodeStr,Twist, queue_size = 1)
        #node_sub_str = self.jackal_names[self.currentJackal] + '/jackal_velocity_controller/odom'
        #self.odom_sub = rospy.Subscriber(node_sub_str, Odometry, self.getOdom)
        self.updateMap()


    #get odometry data from current jackal
    def getOdom(self,msg):
        #get odomoetry data for jackal
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x,orientation_q.y,orientation_q.z,orientation_q.w]
        (roll,pitch,yaw) = euler_from_quaternion(orientation_list)   
        row = [x,y,yaw]
        #buffer is full
        if numpy.count_nonzero(self.odom_buffer) == 9:
            self.odom_loc = numpy.dstack((self.odom_loc,self.odom_buffer))
            self.odom_buffer = numpy.zeros((self.num_jackals,3))
        self.odom_buffer[self.indexer-1][:] = row
        rospy.loginfo(numpy.size(self.odom_loc))

        self.x_odom[self.currentJackal] = msg.pose.pose.position.x
        self.y_odom[self.currentJackal] = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x,orientation_q.y,orientation_q.z,orientation_q.w]
        (roll_odom,pitch_odom,self.yaw_odom[self.currentJackal]) = euler_from_quaternion(orientation_list)   
        #rospy.loginfo(self.x_odom) 

    #publishing forward velocity command
    def moveForward(self):
        if self.pushButtonForwards.isDown() == True:
            self.updateMap()
        command = Twist()
        command.linear.x = self.lin
        self.pub_vel.publish(command)

    #publish backwards velocity command
    def moveBackwards(self):
        if self.pushButtonBackwards.isDown() == True:
            self.updateMap()         
        command = Twist()
        command.linear.x = self.lin*-1
        self.pub_vel.publish(command)

    #publish left angular velocity command
    def moveLeft(self):
        if self.pushButtonLeft.isDown() == True:
            self.updateMap()
        command = Twist()
        command.angular.z = self.ang*1
        self.pub_vel.publish(command)

    #publish right angular velocity command
    def moveRight(self):
        if self.pushButtonRight.isDown() == True:
            self.updateMap()
        command = Twist()
        command.angular.z = self.ang*-1
        self.pub_vel.publish(command)

    #update linear speed
    def updateLin(self):
        self.lin = self.qdialspeed.value()

    #update angular speed
    def updateAng(self):
        self.ang = self.qdialAngspeed.value()

    #ROS Gazebo subscriber function
    def findLoc(self, msg):
        object_names = msg.name
        self.jackal_names = object_names[self.otherObjects:]

        self.num_jackals = len(self.jackal_names)
        #set up combobox
        if not self.setup:
            self.updateSelector()
            self.x_odom = numpy.zeros(self.num_jackals,dtype=float)
            self.y_odom = numpy.zeros(self.num_jackals,dtype=float)
            self.yaw_odom = numpy.zeros(self.num_jackals,dtype=float)
            self.setup = True
        row_loc = numpy.zeros((self.num_jackals,6))
        #get x,y location and angles
        self.indexer = 0
        for w in range(0, self.num_jackals):
            #get gazebo locations
            x = (msg.pose[w+1]).position.x
            y = (msg.pose[w+1]).position.y
            z = (msg.pose[w+1]).position.z
            orientation_q = (msg.pose[w+1]).orientation
            orientation_list = [orientation_q.x,orientation_q.y,orientation_q.z,orientation_q.w]
            (roll,pitch,yaw) = euler_from_quaternion(orientation_list)     
            row_loc[w] = [x,y,z,roll,pitch,yaw]
            #get odometry location
            node_sub_str = self.jackal_names[self.currentJackal] + '/jackal_velocity_controller/odom'
            rospy.Subscriber(node_sub_str, Odometry, self.getOdom)
            self.indexer+=1
        self.gazebo_loc = numpy.dstack((self.gazebo_loc,row_loc))


if __name__ == '__main__':
    try:
        #ROS node
        rospy.init_node('Jackal_Control_Node', anonymous=True)
        #PyQt application
        app = QtWidgets.QApplication([])
        myviz = myWidget()
        myviz.resize( 1500, 750 )
        myviz.setWindowTitle("Jackal Controller")
        #display window
        myviz.show()
        app.exec_()
    except rospy.ROSInterruptException:
        pass