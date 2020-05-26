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
from PyQt5.QtWidgets import QMainWindow, QLabel, QVBoxLayout, QHBoxLayout, QWidget, QGridLayout, QComboBox
from PyQt5.QtCore import QSize, Qt, pyqtSlot, pyqtSignal, QTimer

#GUI class
class myWidget( QWidget ):
    def __init__(self):
        QWidget.__init__(self)
        # number of jackals to control (temporary value)
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
        #label formating variables
        self.old_vel = 1
        self.old_ang = 1
        #ROS subscriber
        rospy.Subscriber("/gazebo/model_states", ModelStates, self.findLoc)
        self.pub_vel = rospy.Publisher('jackal0/jackal_velocity_controller/cmd_vel',Twist, queue_size = 1)
        #create the RViz window
        self.createRViz()
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
        #create pyqt layout
        self.createLayout()


    def createLayout(self):
        #view buttons layout
        view_layout = QHBoxLayout()
        view_layout.addWidget( self.toppushButton )
        view_layout.addWidget( self.sidepushButton )
        #RViz layout
        rviz_layout = QVBoxLayout()
        rviz_layout.addWidget( self.frame )
        rviz_layout.addLayout( view_layout )
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
        side_layout.addWidget(self.labelOdom)
        side_layout.addWidget(self.robotSelector)
        #main window layout
        main_layout = QHBoxLayout()
        main_layout.addLayout(rviz_layout)
        main_layout.addLayout(side_layout)
        self.setLayout( main_layout )


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
        #view buttons
        self.toppushButton = QtWidgets.QPushButton( "Top View" )
        self.toppushButton.clicked.connect( self.onTopButtonClick )
        self.sidepushButton = QtWidgets.QPushButton( "Side View" )
        self.sidepushButton.clicked.connect( self.onSideButtonClick )


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


    def createOdomLabels(self):
        #creating odom labels
        self.labelOdom = QtWidgets.QLabel(self)
        self.labelOdom.setObjectName('labelPositon')
        self.labelOdom.setText("Postion: ({0:.2f}, {1:.2f}) [m]\nAngle: {2:.2f} [rad]".format(self.x[self.currentJackal],self.y[self.currentJackal],self.yaw[self.currentJackal]))
        self.labelOdom.adjustSize()
        #timer for odom update
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.updateLabelOdom)
        self.timer.start(500) #repeat every 1 sec        

    def createSelector(self):
        #create robot selctor menu using QComboBox
        self.robotSelector = QtWidgets.QComboBox(self)
        self.robotSelector.insertItems(0,self.jackal_names)

    def updateSelector(self):
        self.robotSelector.clear()
        self.robotSelector.insertItems(0,self.jackal_names)
        nodeStr = self.jackal_names[0] + '/jackal_velocity_controller/cmd_vel'
        self.pub_vel = rospy.Publisher(nodeStr,Twist, queue_size = 1)

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
        self.labelOdom.setText("Postion: ({0:.2f}, {1:.2f}) [m]\nAngle: {2:.2f} [rad]".format(self.x[self.currentJackal],self.y[self.currentJackal],self.yaw[self.currentJackal]))

    def updateJackal(self):
        self.currentJackal = self.robotSelector.currentIndex()
        nodeStr = self.jackal_names[self.currentJackal] + '/jackal_velocity_controller/cmd_vel'
        self.pub_vel = rospy.Publisher(nodeStr,Twist, queue_size = 1)
        #rospy.loginfo("Index changed to: {}".format(self.currentJackal))

    #publishing to ROS
    def moveForward(self):
        command = Twist()
        command.linear.x = self.lin
        self.pub_vel.publish(command)


    def moveBackwards(self):
        command = Twist()
        command.linear.x = self.lin*-1
        self.pub_vel.publish(command)


    def moveLeft(self):
        command = Twist()
        command.angular.z = self.ang*1
        self.pub_vel.publish(command)


    def moveRight(self):
        command = Twist()
        command.angular.z = self.ang*-1
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
        for i in range(1, self.num_jackals+1):
            self.x[i-1] = (msg.pose[i]).position.x
            self.y[i-1] = (msg.pose[i]).position.y
            orientation_q = (msg.pose[i]).orientation
            orientation_list = [orientation_q.x,orientation_q.y,orientation_q.z,orientation_q.w]
            (self.roll[i-1],self.pitch[i-1],self.yaw[i-1]) = euler_from_quaternion(orientation_list)
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
        myviz.show()
        app.exec_()
    except rospy.ROSInterruptException:
        pass