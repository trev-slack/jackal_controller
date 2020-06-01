#!/usr/bin/env python
import rospy
import rviz
import math
import numpy
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
        self.num_jackals = 1
        self.currentJackal = 0
        self.jackal_names = ["Disconnected"]
        self.setup = False
        # number of non jackal objects
        self.otherObjects = 1
        #intial linear and angular speed
        self.lin = 1
        self.ang = 1
        #position values (size determined by number of jackals)
        self.x = numpy.zeros(self.num_jackals,dtype=float)
        self.y = numpy.zeros(self.num_jackals,dtype=float)
        self.roll = numpy.zeros(self.num_jackals,dtype=float) 
        self.pitch = numpy.zeros(self.num_jackals,dtype=float)
        self.yaw = numpy.zeros(self.num_jackals,dtype=float)
        self.x_odom = numpy.zeros(self.num_jackals,dtype=float)
        self.y_odom = numpy.zeros(self.num_jackals,dtype=float)
        self.yaw_odom = numpy.zeros(self.num_jackals,dtype=float)
        self.map_x = [0]
        self.map_y = [0]
        #label formating variables
        self.old_vel = 1
        self.old_ang = 1
        #ROS subscriber
        rospy.Subscriber("/gazebo/model_states", ModelStates, self.findLoc)
        self.pub_vel = rospy.Publisher('jackal0/jackal_velocity_controller/cmd_vel',Twist, queue_size = 1)

        #create jackal controls
        self.createPushButtons()
        #creating speed controls
        self.createSpeedControls()
        #creating odometry labels
        self.createOdomLabels()
        #create jackal selector
        self.createSelector()
        #connect all controls
        self.connectControls()
'''
        self.graphView = QGraphicsView(self)
        self.graphScene = QGraphicsScene(self)
        self.graphPixmap = self.graphScene.addPixmap(QPixmap())
        self.graphView.setScene(self.graphScene); 
'''

        #create dynamic map
        self.createMap()
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
        main_layout.addWidget(dynamic_canvas)
        main_layout.addLayout(side_layout)
        self.setLayout( main_layout )

    #creates position map
    def createMap(self):
        self.dynamic_canvas = FigureCanvas(Figure(figsize=(5,3)))

        self._dynamic_ax = self.dynamic_canvas.figure.subplots()
        self._dynamic_timer = self.dynamic_canvas.new_timer(100, [(self.updateMap,(),{})])
        self._dynamic_timer.start()
'''
        sp = SubplotParams(left=0.,bottom=0.,right=1.,top=1.); 
        self.fig = Figure(subplotpars=sp); 
        self.canvas = FigureCanvas(self.fig); 
        self.ax = self.fig.add_subplot(111); 
        self.ax.scatter(self.x,self.y)
        self.ax.set_xlabel("x [m]")
        self.ax.set_ylabel("y [m]")
        self.ax.set_title("Jackal Map")
        self.ax.grid(True)
        self.ax.set_xlim([-20, 20])
        self.ax.set_ylim([-20, 20])
        self.canvas.draw(); 
        size = self.canvas.size(); 
        width,height = size.width(),size.height(); 
        self.im = QImage(self.canvas.buffer_rgba(),width,height,QtGui.QImage.Format_ARGB32); 
        self.pm = QPixmap(self.im); 
        self.graphPixmap.setPixmap(self.pm)
'''
    def updateMap(self):
        self._dynamic_ax.clear()
        t = np.linspace(0,10,501)
        self._dynamic_ax.plot(t, np.sin(t + time.time()))
        self._dynamic_ax.figure.canvas.draw()

    #creates QPushButtons for controls
    def createPushButtons(self):
        #forwards
        self.pushButtonForwards = QtWidgets.QPushButton(self)
        self.pushButtonForwards.setObjectName("pushButtonForwards")
        self.pushButtonForwards.setText("Forwards")
        self.pushButtonForwards.setAutoRepeat(True)
        #backwards
        self.pushButtonBackwards = QtWidgets.QPushButton(self)
        self.pushButtonBackwards.setObjectName("pushButtonBackwards")
        self.pushButtonBackwards.setText("Backwards")
        self.pushButtonBackwards.setAutoRepeat(True)
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
        self.labelPosition.setText("Gazebo\nPosition: ({0:.2f}, {1:.2f}) [m]\nAngle: {2:.2f} [rad]".format(self.x[self.currentJackal],self.y[self.currentJackal],self.yaw[self.currentJackal]))
        self.labelPosition.adjustSize()
        #creating odom labels
        self.labelOdom = QtWidgets.QLabel(self)
        self.labelOdom.setObjectName('labelOdom')
        self.labelOdom.setText("Odometry\nPosition: ({0:.2f}, {1:.2f}) [m]\nAngle: {2:.2f} [rad]".format(self.x_odom[self.currentJackal],self.y_odom[self.currentJackal],self.yaw_odom[self.currentJackal]))
        self.labelOdom.adjustSize()
        #timer for odom update
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.updateLabelPosition)
        self.timer.start(500) #repeat every 1 sec    
        self.timer2 = QtCore.QTimer()
        self.timer2.timeout.connect(self.updateLabelOdom)
        self.timer2.start(500) #repeat every 1 sec          

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
        rospy.Subscriber(node_sub_str, Odometry, self.getOdom)

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
        self.labelPosition.setText("Gazebo\nPosition: ({0:.2f}, {1:.2f}) [m]\nAngle: {2:.2f} [rad]".format(self.x[self.currentJackal],self.y[self.currentJackal],self.yaw[self.currentJackal]))

    #update odometry position label
    def updateLabelOdom(self):
        #rospy.loginfo(self.currentJackal)
        self.labelOdom.setText("Odometry\nPosition: ({0:.2f}, {1:.2f}) [m]\nAngle: {2:.2f} [rad]".format(self.x_odom[self.currentJackal],self.y_odom[self.currentJackal],self.yaw_odom[self.currentJackal]))

    #update current selected jackal from QComboBox
    def updateJackal(self):
        self.currentJackal = self.robotSelector.currentIndex()
        nodeStr = self.jackal_names[self.currentJackal] + '/jackal_velocity_controller/cmd_vel'
        self.pub_vel = rospy.Publisher(nodeStr,Twist, queue_size = 1)
        node_sub_str = self.jackal_names[self.currentJackal] + '/jackal_velocity_controller/odom'
        rospy.Subscriber(node_sub_str, Odometry, self.getOdom)
        rospy.loginfo("Subscribed to: {}".format(self.jackal_names[self.currentJackal]))


        #node_sub_str = self.jackal_names[self.currentJackal] + '/jackal_velocity_controller/odom'
        #rospy.Subscriber(node_sub_str, Odometry, self.getOdom)
        #rospy.loginfo("Index changed to: {}".format(self.currentJackal))

    #get odometry data from current jackal
    def getOdom(self,msg):
        #get odomoetry data for jackal
        rospy.loginfo(msg)
        rospy.loginfo(self.w-1)
        self.x_odom[self.w-1] = msg.pose.pose.position.x
        self.y_odom[self.w-1] = msg.pose.pose.position.y
        #rospy.loginfo(msg)
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x,orientation_q.y,orientation_q.z,orientation_q.w]
        (roll_odom,pitch_odom,self.yaw_odom[self.w-1]) = euler_from_quaternion(orientation_list)    

    #publishing forward velocity command
    def moveForward(self):
        #self.updateMap()
        command = Twist()
        command.linear.x = self.lin
        self.pub_vel.publish(command)

    #publish backwards velocity command
    def moveBackwards(self):
        command = Twist()
        command.linear.x = self.lin*-1
        self.pub_vel.publish(command)

    #publish left angular velocity command
    def moveLeft(self):
        command = Twist()
        command.angular.z = self.ang*1
        self.pub_vel.publish(command)

    #publish right angular velocity command
    def moveRight(self):
        command = Twist()
        command.angular.z = self.ang*-1
        self.pub_vel.publish(command)

    #update linear speed
    def updateLin(self):
        self.lin = self.qdialspeed.value()

    #update angular speed
    def updateAng(self):
        self.ang = self.qdialAngspeed.value()


    #manage RViz View
    def onTopButtonClick( self ):
        self.switchToView( "Top View" );
        
    #manage RViz side view
    def onSideButtonClick( self ):
        self.switchToView( "Side View" );
        

    #get views from config
    def switchToView( self, view_name ):
        view_man = self.manager.getViewManager()
        for i in range( view_man.getNumViews() ):
            if view_man.getViewAt( i ).getName() == view_name:
                view_man.setCurrentFrom( view_man.getViewAt( i ))
                return
        print( "Did not find view named %s." % view_name )


    #ROS Gazebo subscriber function
    def findLoc(self, msg):
        object_names = msg.name
        self.jackal_names = object_names[self.otherObjects:]
        self.num_jackals = len(self.jackal_names)
        #rospy.loginfo("Number jackals: {}".format(self.num_jackals))
        #set up combobox
        if not self.setup:
            self.updateSelector()
            self.setup = True
        #get x,y location and angles
        self.x = numpy.zeros(self.num_jackals,dtype=float)
        self.y = numpy.zeros(self.num_jackals,dtype=float)
        self.roll = numpy.zeros(self.num_jackals,dtype=float) 
        self.pitch = numpy.zeros(self.num_jackals,dtype=float)
        self.yaw = numpy.zeros(self.num_jackals,dtype=float)
        self.x_odom = numpy.zeros(self.num_jackals,dtype=float)
        self.y_odom = numpy.zeros(self.num_jackals,dtype=float)
        self.yaw_odom = numpy.zeros(self.num_jackals,dtype=float)
        for self.w in range(1, self.num_jackals+1):
            self.x[self.w-1] = (msg.pose[self.w]).position.x
            self.y[self.w-1] = (msg.pose[self.w]).position.y
            orientation_q = (msg.pose[self.w]).orientation
            orientation_list = [orientation_q.x,orientation_q.y,orientation_q.z,orientation_q.w]
            (self.roll[self.w-1],self.pitch[self.w-1],self.yaw[self.w-1]) = euler_from_quaternion(orientation_list)
            #rospy.loginfo(self.jackal_names[i-1])
            node_sub_str = self.jackal_names[self.w-1] + '/jackal_velocity_controller/odom'
            rospy.Subscriber(node_sub_str, Odometry, self.getOdom)
        #rospy.loginfo(self.x)
        #rospy.loginfo("Current Jackal: {}".format(self.currentJackal))


if __name__ == '__main__':
    try:
        #ROS node
        rospy.init_node('Jackal_Control_Node', anonymous=True)
        #PyQt application
        app = QtWidgets.QApplication([])
        myviz = myWidget()
        myviz.resize( 500, 500 )
        #display window
        myviz.show()
        app.exec_()
    except rospy.ROSInterruptException:
        pass