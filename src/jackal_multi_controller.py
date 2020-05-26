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

from PyQt5 import QtCore, QtWidgets
from PyQt5.QtWidgets import QMainWindow, QLabel, QVBoxLayout, QHBoxLayout, QWidget, QGridLayout
from PyQt5.QtCore import QSize, Qt, pyqtSlot, pyqtSignal, QTimer

#GUI class
class myWidget( QWidget ):
    pub_vel = rospy.Publisher('jackal1/jackal_velocity_controller/cmd_vel',Twist, queue_size = 1)
    def __init__(self):
        QWidget.__init__(self)
        # number of jackals to control
        self.num_jackals = 2
        #intial linear and angular speed
        self.lin = 1
        self.ang = 1
        self.x = numpy.zeros(self.num_jackals,dtype=float)
        self.y = numpy.zeros(self.num_jackals,dtype=float)
        self.roll = numpy.zeros(self.num_jackals,dtype=float) 
        self.pitch = numpy.zeros(self.num_jackals,dtype=float)
        self.yaw = numpy.zeros(self.num_jackals,dtype=float)
        #label formating variables
        self.old_vel = 1
        self.old_ang = 1
        #ROS subscriber
        rospy.Subscriber("/gazebo/model_states", ModelStates, self.findLoc)
        #create the RViz window
        self.createRViz()

        #layout for RViz
        rviz_layout = QVBoxLayout()
        rviz_layout.addWidget( self.frame )
        view_layout = QHBoxLayout()
        #view buttons
        toppushButton = QtWidgets.QPushButton( "Top View" )
        toppushButton.clicked.connect( self.onTopButtonClick )
        view_layout.addWidget( toppushButton )
        sidepushButton = QtWidgets.QPushButton( "Side View" )
        sidepushButton.clicked.connect( self.onSideButtonClick )
        view_layout.addWidget( sidepushButton )
        rviz_layout.addLayout( view_layout )
        

        #creating turning controls
        control_box = QGridLayout()
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
        #add buttons to layout
        control_box.addWidget(self.pushButtonForwards,0,1)
        control_box.addWidget(self.pushButtonLeft,1,0)
        control_box.addWidget(self.pushButtonRight,1,2)
        control_box.addWidget(self.pushButtonBackwards,2,1)       


        #creating speed controls
        self.createSpeedControls()

        #creating odometry labels
        self.createOdomLabels()

        #side controls
        side_layout = QVBoxLayout()
        side_layout.addLayout(control_box)
        side_layout.addLayout(self.speed_box)
        side_layout.addWidget(self.labelOdom)

        #main window layout
        main_layout = QHBoxLayout()
        main_layout.addLayout(rviz_layout)
        main_layout.addLayout(side_layout)
        self.setLayout( main_layout )

        self.connectControls()
        #behavior
        self.pushButtonForwards.pressed.connect(self.moveForward)
        self.pushButtonForwards.released.connect(self.decayForward)
        self.pushButtonBackwards.pressed.connect(self.moveBackwards)
        self.pushButtonLeft.pressed.connect(self.moveLeft)
        self.pushButtonRight.pressed.connect(self.moveRight)
        self.qdialspeed.sliderReleased.connect(self.updateLin)
        self.qdialspeed.sliderReleased.connect(self.updateLabel)
        self.qdialAngspeed.sliderReleased.connect(self.updateAng)
        self.qdialAngspeed.sliderReleased.connect(self.updateLabelAng)


    def createRViz(self):
        #main RViz window
        self.frame = rviz.VisualizationFrame()
        #remove loading splash
        self.frame.setSplashPath( "" )
        #initialize manager
        self.frame.initialize()
        #read/run config
        reader = rviz.YamlConfigReader()
        config = rviz.Config()
        reader.readFile( config, "jackal_config_reduced_multi.rviz" )
        self.frame.load( config )
        self.setWindowTitle( config.mapGetChild( "Title" ).getValue() )
        #diable menus
        self.frame.setMenuBar( None )
        self.frame.setStatusBar( None )
        self.frame.setHideButtonVisibility( False )
        #set up manager for RViz
        self.manager = self.frame.getManager()
        #reference to display
        self.grid_display = self.manager.getRootDisplayGroup().getDisplayAt( 0 )        

    def createSpeedControls(self):
        #linear speed dial
        self.qdialspeed = QtWidgets.QDial(self)
        self.qdialspeed.setObjectName("QDialSpeed")
        self.qdialspeed.setMinimum(1)
        self.qdialspeed.setMaximum(5)
        self.qdialspeed.setValue(1)
        self.qdialspeed.setNotchesVisible(True)        
        #angular speed dial
        self.qdialAngspeed = QtWidgets.QDial(self)
        self.qdialAngspeed.setObjectName("QDialAngSpeed")
        self.qdialAngspeed.setMinimum(1)
        self.qdialAngspeed.setMaximum(5)
        self.qdialAngspeed.setValue(1)
        self.qdialAngspeed.setNotchesVisible(True)
        #label for linear
        self.labelVel = QtWidgets.QLabel(self)
        self.labelVel.setObjectName("labelVel")
        self.labelVel.setText("Linear Speed : 1 m/s")
        self.labelVel.adjustSize()
        #label for angular
        self.labelAngVel = QtWidgets.QLabel(self)
        self.labelAngVel.setObjectName("labelAngVel")
        self.labelAngVel.setText("Angular Speed : 1 rad/s")
        self.labelAngVel.adjustSize()
        #layout
        self.speed_box = QGridLayout()
        self.speed_box.addWidget(self.qdialspeed,1,0)
        self.speed_box.addWidget(self.qdialAngspeed,1,1)
        self.speed_box.addWidget(self.labelVel,0,0)
        self.speed_box.addWidget(self.labelAngVel,0,1)

    def createOdomLabels(self):
        #creating odom labels
        self.labelOdom = QtWidgets.QLabel(self)
        self.labelOdom.setObjectName('labelPositon')
        self.labelOdom.setText("Postion: ({0:.2f}, {1:.2f}) [m]\nAngle: {2:.2f} [rad]".format(self.x[1],self.y[1],self.yaw[1]))
        self.labelOdom.adjustSize()
        #timer for odom update
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.updateLabelOdom)
        self.timer.start(500) #repeat every 1 sec        

    def connectControls(self):
         #behavior
        self.pushButtonForwards.pressed.connect(self.moveForward)
        self.pushButtonForwards.released.connect(self.decayForward)
        self.pushButtonBackwards.pressed.connect(self.moveBackwards)
        self.pushButtonLeft.pressed.connect(self.moveLeft)
        self.pushButtonRight.pressed.connect(self.moveRight)
        self.qdialspeed.sliderReleased.connect(self.updateLin)
        self.qdialspeed.sliderReleased.connect(self.updateLabel)
        self.qdialAngspeed.sliderReleased.connect(self.updateAng)
        self.qdialAngspeed.sliderReleased.connect(self.updateLabelAng)       

    #update labels
    def updateLabel(self):
        if self.qdialspeed.value() != self.old_vel:
            self.labelVel.setText("Linear Speed : {0:.0f} m/s".format(self.qdialspeed.value()))
            self.labelVel.adjustSize()
        self.old_vel = self.qdialspeed.value()
    def updateLabelAng(self):
        if self.qdialAngspeed.value() != self.old_ang:
            self.labelAngVel.setText("Angular Speed : {0:.0f} rad/s".format(self.qdialAngspeed.value()))
            self.labelVel.adjustSize()
        self.old_ang = self.qdialAngspeed.value()
    def updateLabelOdom(self):
        self.labelOdom.setText("Postion: ({0:.2f}, {1:.2f}) [m]\nAngle: {2:.2f} [rad]".format(self.x[1],self.y[1],self.yaw[1]))
        #self.labelAngle.setText("Angle: ({0:.2f}, {1:.2f}, {2:.2f})".format(self.roll,self.pitch,self.yaw))

    #publishing to ROS
    def moveForward(self):
        command = Twist()
        command.linear.x = self.lin
        #rospy.loginfo("Approximate Global (x,y)=({},{})\nAngle = {}".format(self.x,self.y,self.yaw))
        #command.angular.z = 0
        self.pub_vel.publish(command)

    def decayForward(self):
        rospy.loginfo("x = {},{},{}".format(self.x_test(0),self.x_test(1),self.x_test(2)))
        command = Twist()
        command.linear.z = 0

    def moveBackwards(self):
        command = Twist()
        command.linear.x = self.lin*-1
        #rospy.loginfo("Approximate Global (x,y)=({},{})\nAngle = {}".format(self.x,self.y,self.yaw))
        #command.angular.z = 0
        self.pub_vel.publish(command)

    def moveLeft(self):
        command = Twist()
        command.angular.z = self.ang*1
        #rospy.loginfo("Approximate Global (x,y)=({},{})\nAngle = {}".format(self.x,self.y,self.yaw))
        self.pub_vel.publish(command)

    def moveRight(self):
        command = Twist()
        command.angular.z = self.ang*-1
        #rospy.loginfo("Approximate Global (x,y)=({},{})\nAngle = {}".format(self.x,self.y,self.yaw))
        self.pub_vel.publish(command)

    def updateLin(self):
        self.lin = self.qdialspeed.value()

    def updateAng(self):
        self.ang = self.qdialAngspeed.value()

    #manage RViz View
    def onTopButtonClick( self ):
        self.switchToView( "Top View" );
        
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

    #ROS subscriber function
    def findLoc(self, msg):
        #get x,y location and angles
        for i in range(1, self.num_jackals):
            self.x[i] = (msg.pose[i]).position.x
            self.y[i] = (msg.pose[i]).position.y
            orientation_q = (msg.pose[i]).orientation
            orientation_list = [orientation_q.x,orientation_q.y,orientation_q.z,orientation_q.w]
            (self.roll[i],self.pitch[i],self.yaw[i]) = euler_from_quaternion(orientation_list)


if __name__ == '__main__':
    try:
        #ROS node
        rospy.init_node('Jackal_Control_Node', anonymous=True)
        #PyQt application
        app = QtWidgets.QApplication([])
        myviz = myWidget()
        myviz.resize( 500, 500 )
        myviz.show()
        app.exec_()
    except rospy.ROSInterruptException:
        pass