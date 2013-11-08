import os
import rospy
#from PySide import QtCore
#from PySide import QtGui, QtCore
from python_qt_binding import QtGui
from python_qt_binding import QtCore

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget
from george.msg import Appendage_state
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from robotDescription import *
from george.srv import *
robot_description = RobotDescription()
##########################################################################
##########################################################################
class Communicate(QtCore.QObject):
##########################################################################
##########################################################################
    pbar_val = QtCore.Signal(int)

##########################################################################
##########################################################################
class CmdrPanel(QtGui.QWidget):
##########################################################################
##########################################################################
    def __init__(self, name, index, caller_change_callback):
        self.c = Communicate()
        QtGui.QWidget.__init__(self)
        self.panelLayout = QtGui.QGridLayout()
        self.setLayout(self.panelLayout)
        self.lblName = QtGui.QLabel( "Panel" )
        self.panelLayout.addWidget(self.lblName,0,0)
        

##########################################################################
##########################################################################
class CmdrPlugin(Plugin):
##########################################################################
##########################################################################

    def __init__(self, context):
        self.init_ros_params()
        # robot_description.Print()

        super(SliderPlugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName(self.formname)


        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                      dest="quiet",
                      help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print 'arguments: ', args
            print 'unknowns: ', unknowns

        self.mainLayout = QtGui.QVBoxLayout( )

        # Create the panels
        self.main_panel = CmdrPanel( )
            self.mainLayout.addWidget( self.panels[ i ] )
            
        self.button_panel = buttonPanel(self)
        self.mainLayout.addWidget( self.button_panel )

        # central widget
        self.form1 = QWidget()
        self.form1.setLayout(self.mainLayout)
        #self.ui.setLayout(self.mainLayout)

        # set central widget
        
        # Give QObjects reasonable names
        self.form1.setObjectName(self.formname)
        self.form1.setWindowTitle( self.formname )
        # Show _widget.windowTitle on left-top of each plugin (when 
        # it's set in _widget). This is useful when you open multiple 
        # plugins at once. Also if you open multiple instances of your 
        # plugin at once, these lines add number to make it easy to 
        # tell from pane to pane.
        if context.serial_number() > 1:
            self.form1.setWindowTitle(self.form1.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        #self.a = QtGui.QPushButton("button")
        #self.ui.panel3.addWidget(self.a)

        #context.add_widget(self.ui)
        context.add_widget(self.form1)
        
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect( self.on_timer_update)
        self.timer.start(50)
        
        self.c = Communicate()
        
        #self.c.slider1.connect( self.panel1.slider.setValue )
        #self.panel1.slider.valueChanged.connect( self.on_slider1_changed )

    def init_ros_params(self):
        robot_description.ReadParameters()
        self.command_pub = rospy.Publisher("command" + str(self.appendage_no), Appendage_state)
        self.command_msg = Appendage_state()
        self.command_msg.joints = [90] * 6
        self.command_msg.speed = 1
        
        self.macro_cmd_sub = rospy.Subscriber("macro_cmd", String, self.macro_cmd_callback, None, 100)
        
        self.joint_state_sub = rospy.Subscriber("joint_states" + str(self.appendage_no), JointState, self.joint_states_callback, None, 1)
        self.joint_state_msg = Appendage_state()
        self.joint_state_msg.joints = [90] * 6
        self.joint_state_msg.speed = 1

    def slider_changed_callback(self, slider_no, value):
        rospy.logdebug( "Slider changed callback, slider %d value: %d" % (slider_no, value))
        self.command_msg.joints[slider_no] = value
        # self.command_publish()
        
    def command_publish(self, docheck=True):
        rospy.logdebug("publishing command message for appendage " + str(self.appendage_no) )
        if docheck == False or self.is_changed( self.command_msg ):
            self.command_pub.publish( self.command_msg )
        
    def is_changed(self, cmd_msg):
        retval = False

        if len( self.prev_cmd_joints ) == len( cmd_msg.joints ):
            for i in range( len( cmd_msg.joints ) ) :
                if self.prev_cmd_joints[i] != cmd_msg.joints[i]:
                    retval = True
                    self.prev_cmd_joints[i] = cmd_msg.joints[i]
        else:
            # prev_cmd has not been initialized
            self.prev_cmd_joints = []
            for i in range( len( cmd_msg.joints ) ) :
                self.prev_cmd_joints.append(cmd_msg.joints[i])
            retval = True
        return( retval )
        
    def macro_cmd_callback(self, msg):
        pass
    
    def joint_states_callback(self, msg):
        rospy.logdebug( "joint_states_callback msg: %s " % msg)
        for i in range( len( msg.position ) ):
            self.panels[i].set_progressbar(msg.position[i])
            
    def copy_states(self):
        rospy.logdebug( "-D- in copy_states")
        for i in range( self.npanels ):
            self.panels[i].slider.setValue( self.panels[i].prog.value() )
            
    def on_timer_update(self):
       #rospy.logdebug( "-D- tick")
        if self.button_panel.auto_button.isChecked():
            rospy.logdebug("publishing")
            self.command_publish( )
    
    def shutdown_plugin(self):
        # TODO unregister all publishers here
        pass

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog
        