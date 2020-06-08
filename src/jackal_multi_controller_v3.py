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
        numpy.set_printoptions(threshold=numpy.inf)
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
        #position arrays
        self.gazebo_loc = numpy.zeros((self.num_jackals,6,1))
        self.odom_loc = numpy.zeros((self.num_jackals,3,1))
        self.odom_buffer = numpy.zeros((self.num_jackals,3))
        self.subbed = False
        #label formating variables
        self.old_vel = self.lin
        self.old_ang = self.ang
        #call main
        self.controllerMain()


    #main setup
    def controllerMain(self):
        #ros subscriber
        self.gazebo_sub = rospy.Subscriber("/gazebo/model_states", ModelStates, self.findLoc)
        self.pub_vel = rospy.Publisher('jackal0/jackal_velocity_controller/cmd_vel',Twist, queue_size = 1)
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
        #commit layout
        self.setLayout( main_layout )


    #creates position map
    def createMap(self):
        self.dynamic_canvas = FigureCanvas(Figure(figsize=(5,5)))
        self._dynamic_ax = self.dynamic_canvas.figure.subplots()


    #updates map
    def updateMap(self):
        self._dynamic_ax.clear()
        #formating
        self._dynamic_ax.set_xlabel("x [m]")
        self._dynamic_ax.set_xlim(-10, 10)
        self._dynamic_ax.set_ylabel("y [m]")
        self._dynamic_ax.set_ylim(-10, 10)
        self._dynamic_ax.set_title(self.jackal_names[self.currentJackal] + " Map")
        self._dynamic_ax.grid()
        #plot trace
        ##print(self.odom_loc)
        #print("==========")
        self._dynamic_ax.plot(self.gazebo_loc[self.currentJackal,0,:],self.gazebo_loc[self.currentJackal,1,:],label = '1')
        self._dynamic_ax.plot(self.odom_loc[self.currentJackal,0,:],self.odom_loc[self.currentJackal,1,:],label = '3')
        #plot current position
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
        self.labelOdom.setText("Odometry\nposition: ({0:.2f}, {1:.2f}) [m]\nAngle: {2:.2f} [rad]".format(self.odom_loc[self.currentJackal,0,-1],self.odom_loc[self.currentJackal,1,-1],self.odom_loc[self.currentJackal,2,-1]))
        self.labelOdom.adjustSize()
        #timer for odom update
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.updateLabelPosition)
        self.timer.start(500) #repeat every .5 sec    
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
        for i in range(0, self.num_jackals):
            node_sub_str = self.jackal_names[i] + '/jackal_velocity_controller/odom'
            self.odom_sub = rospy.Subscriber(node_sub_str, Odometry, self.getOdom)
        rospy.loginfo(self.jackal_names)


    #links signals for QT
    def connectControls(self):
        #behavior for QT
        self.pushButtonForwards.pressed.connect(self.moveForward)
        self.pushButtonBackwards.pressed.connect(self.moveBackwards)
        self.pushButtonLeft.pressed.connect(self.moveLeft)
        self.pushButtonRight.pressed.connect(self.moveRight)
        self.qdialspeed.sliderReleased.connect(self.updateLin)
        self.qdialspeed.sliderReleased.connect(self.updateLabelLin)
        self.qdialAngspeed.sliderReleased.connect(self.updateAng)
        self.qdialAngspeed.sliderReleased.connect(self.updateLabelAng)       
        self.robotSelector.currentIndexChanged.connect(self.updateJackal)


    #update linear speed label
    def updateLabelLin(self):
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


    #update position label
    def updateLabelPosition(self):
        self.labelPosition.setText("Gazebo\nposition: ({0:.2f}, {1:.2f}) [m]\nAngle: {2:.2f} [rad]".format(self.gazebo_loc[self.currentJackal,0,-1],self.gazebo_loc[self.currentJackal,1,-1],self.gazebo_loc[self.currentJackal,5,-1]))


    #update odometry position label
    def updateLabelOdom(self):
        self.labelOdom.setText("Odometry\nposition: ({0:.2f}, {1:.2f}) [m]\nAngle: {2:.2f} [rad]".format(self.odom_loc[self.currentJackal,0,-1],self.odom_loc[self.currentJackal,1,-1],self.odom_loc[self.currentJackal,2,-1]))


    #update current selected jackal from QComboBox
    def updateJackal(self):
        self.currentJackal = self.robotSelector.currentIndex()
        nodeStr = self.jackal_names[self.currentJackal] + '/jackal_velocity_controller/cmd_vel'
        self.pub_vel = rospy.Publisher(nodeStr,Twist, queue_size = 1)
        self.updateMap()


    #get odometry data from current jackal
    def getOdom(self,msg):
        #get odomoetry data for jackal
        name_id = msg.child_frame_id
        name = name_id.strip('/base_link')
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x,orientation_q.y,orientation_q.z,orientation_q.w]
        (roll,pitch,yaw) = euler_from_quaternion(orientation_list)   
        row = [x,y,yaw]
        #buffer is full
        if numpy.count_nonzero(self.odom_buffer)==9:
            self.odom_loc = numpy.dstack((self.odom_loc,self.odom_buffer))




            # Write the array to disk
            with open('test.txt', 'w') as outfile:
                # I'm writing a header here just for the sake of readability
                # Any line starting with "#" will be ignored by numpy.loadtxt
                outfile.write('# Array shape: {0}\n'.format(self.odom_loc.shape))

                # Iterating through a ndimensional array produces slices along
                # the last axis. This is equivalent to data[i,:,:] in this case
                for data_slice in self.odom_loc:

                    # The formatting string indicates that I'm writing out
                    # the values in left-justified columns 7 characters in width
                    # with 2 decimal places.  
                    numpy.savetxt(outfile, data_slice, fmt='%-7.2f')

                    # Writing out a break to indicate different slices...
                    outfile.write('# New slice\n')





            #numpy.savetxt(outfile, self.odom_loc)
            self.odom_buffer = numpy.zeros((self.num_jackals,3))
        self.odom_buffer[self.jackal_names.index(name)] = row


    #publishing forward velocity command
    def moveForward(self):
        #update map while pushed
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
        #get jackal info
        self.jackal_names = object_names[self.otherObjects:]
        self.num_jackals = len(self.jackal_names)
        #set up combobox
        row_loc = numpy.zeros((self.num_jackals,6))
        #get x,y location and angles
        for w in range(0, self.num_jackals):
            #get gazebo locations, subtracting of starting position to get in local coords
            x = (msg.pose[w+1]).position.x + self.jackal_spawn_x[w]
            y = (msg.pose[w+1]).position.y + self.jackal_spawn_y[w]
            z = (msg.pose[w+1]).position.z
            orientation_q = (msg.pose[w+1]).orientation
            orientation_list = [orientation_q.x,orientation_q.y,orientation_q.z,orientation_q.w]
            (roll,pitch,yaw) = euler_from_quaternion(orientation_list)     
            row_loc[w] = [x,y,z,roll,pitch,yaw]
        #check if first location
        all_zeros = not numpy.any(self.gazebo_loc)
        if all_zeros:
            self.gazebo_loc = row_loc
        else:
            self.gazebo_loc = numpy.dstack((self.gazebo_loc,row_loc))
        #set up combobox
        if not self.setup:
            self.updateSelector()
            self.setup = True



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