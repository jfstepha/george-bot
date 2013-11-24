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
class buttonPanel(QtGui.QWidget):
##########################################################################
##########################################################################
    def __init__(self, parent):
        
        QtGui.QWidget.__init__(self)
        self.parent = parent
        btnw = 40
        btnh = 20

        self.srv_home = rospy.ServiceProxy('home', Home)
        self.srv_stop = rospy.ServiceProxy('stop', Stop)
        
        self.panelLayout = QtGui.QHBoxLayout()
        self.setLayout(self.panelLayout)
        
        self.speed_text = QtGui.QLineEdit()
        self.speed_text.setText("1")
        self.speed_text.maximumSize = [120,113]
        self.speed_text.setMinimumSize(btnw,btnh)
        self.speed_text.textChanged.connect( self.on_speed_text_changed )

        self.home_button = QtGui.QPushButton()
        self.home_button.setText('Home')
        self.home_button.clicked.connect( self.on_home_button_clicked )
        self.home_button.setMinimumSize(btnw,btnh)
        
        self.stop_button = QtGui.QPushButton()
        self.stop_button.setText("Stop")
        self.stop_button.clicked.connect( self.on_stop_button_clicked )
        self.stop_button.setMinimumSize(btnw,btnh)
        
        self.copy_button = QtGui.QPushButton()
        self.copy_button.setText("Copy Positions")
        self.copy_button.clicked.connect( self.on_copy_button_clicked)
        self.copy_button.setMinimumSize(btnw,btnh)
        
        self.send_button = QtGui.QPushButton()
        self.send_button.setText("Send")
        self.send_button.clicked.connect( self.on_send_button_clicked )
        self.send_button.setMinimumSize(btnw,btnh)
        
        self.auto_button = QtGui.QPushButton()
        self.auto_button.setText("Auto Send")
        self.auto_button.clicked.connect( self.on_auto_button_clicked )
        self.auto_button.setCheckable(True)
        self.auto_button.setMinimumSize(btnw,btnh)

        self.panelLayout.addWidget(self.speed_text)
        self.panelLayout.addWidget(self.home_button)
        self.panelLayout.addWidget(self.stop_button)
        self.panelLayout.addWidget(self.copy_button)
        self.panelLayout.addWidget(self.send_button)
        self.panelLayout.addWidget(self.auto_button)
        
    def on_home_button_clicked(self):
        rospy.logdebug("home_button_clicked")
        try:
            self.auto_button.setChecked(False)
            self.send_button.setEnabled(True)
            resp = self.srv_home()
        except rospy.ServiceException, e:
            rospy.logerr("Service call failed: %s"%e)

    def on_stop_button_clicked(self):
        rospy.logdebug("stop_button_clicked")
        try:
            self.auto_button.setChecked(False)
            self.send_button.setEnabled(True)
            resp = self.srv_stop()
        except rospy.ServiceException, e:
            rospy.logerr("Service call failed: %s"%e)
        
    def on_copy_button_clicked(self):
        rospy.logdebug("copy_button_clicked")
        self.parent.copy_states()

    def on_send_button_clicked(self):
        rospy.logdebug("send_button_clicked")
        self.parent.command_publish( docheck=False)

    def on_auto_button_clicked(self):
        rospy.logdebug("auto_button_clicked")
        
    def on_speed_text_changed(self, value):
        rospy.logdebug("speed text changed %d" % value)
        
    
##########################################################################
##########################################################################
class sliderPanel(QtGui.QWidget):
##########################################################################
##########################################################################
    def __init__(self, name, index, caller_change_callback):
        self.c = Communicate()
        self.caller_change_callback = caller_change_callback
        self.index = index
        self.name = name
        QtGui.QWidget.__init__(self)
        self.panelLayout = QtGui.QGridLayout()
        self.setLayout(self.panelLayout)
        self.slider = QtGui.QSlider( QtCore.Qt.Horizontal )
        self.slider.setMinimum(0)
        self.slider.setMaximum(180)
        self.slider.setValue(90)
        self.slider.setMinimumSize(100,10)
        self.slider.setMinimumSize(100,10)

        self.lblName = QtGui.QLabel( name )
        self.lblName.setMinimumSize(100,10)

        self.prog = QtGui.QProgressBar()
        self.prog.setMinimum(0)
        self.prog.setMaximum(180)
        self.prog.setMinimumSize(100,10)

        self.panelLayout.addWidget(self.lblName,0,0)
        self.panelLayout.addWidget(self.slider, 0, 1)
        self.panelLayout.addWidget(self.prog, 1, 1)
        
        self.slider.valueChanged.connect( self.on_slider_changed )
        self.c.pbar_val.connect( self.prog.setValue)
        self.max_update_rate = 20 # msec
        self.last_changed = rospy.Time.now()
        
    def on_slider_changed(self, value):
        dt_duration = rospy.Time.now() - self.last_changed
        rospy.logdebug("Slider %d of %s changed value: %d, time since last update: %0.3f" % (self.index, self.name, value, dt_duration.to_sec()))
        if dt_duration.to_sec() * 100 > self.max_update_rate / 100:
            rospy.logdebug("calling publish")
            self.caller_change_callback(self.index, value)
            self.last_changed = rospy.Time.now()
    def set_progressbar(self, value):
        v2 = (value  + 3.1416 / 2.0) / 3.1416 * 180.0
        rospy.loginfo("set_progressbar %s (%d) to %0.3f (original=%0.3f)" % (self.name, self.index, v2, value))
        self.c.pbar_val.emit(v2)

        

##########################################################################
##########################################################################
class SliderPlugin(Plugin):
    '''
    This is the parent class for an appendage slider control panel.  Each
    appendage has a separate child class.
    '''
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
        self.npanels = robot_description.appendages[ self.appendage_no ].nservos
        jointnames = robot_description.appendages[ self.appendage_no ].jointnames
        self.prev_cmd_joints = []
        self.panels = []
        for i in range( self.npanels ):
            self.panels.append( sliderPanel( jointnames[ i ], i, self.slider_changed_callback ) )
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
        self.command_msg.joints = [0] * 6
        self.command_msg.speed = 1
        
        self.macro_cmd_sub = rospy.Subscriber("macro_cmd", String, self.macro_cmd_callback, None, 100)
        
        self.joint_state_sub = rospy.Subscriber("joint_states" + str(self.appendage_no), JointState, self.joint_states_callback, None, 1)
        self.joint_state_msg = Appendage_state()
        self.joint_state_msg.joints = [0] * 6
        self.joint_state_msg.speed = 1

    def slider_changed_callback(self, slider_no, value):
        rospy.logdebug( "Slider changed callback, slider %d value: %d" % (slider_no, value))
        self.command_msg.joints[slider_no] = (value / 180.0) * 3.1416 - 3.1416 / 2 
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
        
##########################################################################
##########################################################################
class RArmSliderPlugin(SliderPlugin):
##########################################################################
##########################################################################
    def __init__(self, context):
        print "Right Arm Init"
        self.formname = "Right Arm"
        self.appendage_no = 3
        SliderPlugin.__init__(self, context)
        
        
    def on_slider1_changed(self, value):
        print "Slider1 changed! Value=%d" % value
        self.c.slider1 = value
        self.panel1.prog.setValue(value)
    
##########################################################################
##########################################################################
class LArmSliderPlugin(SliderPlugin):
##########################################################################
##########################################################################
    def __init__(self, context):
        print "Left Arm Init"
        self.formname = "Left Arm"
        self.appendage_no = 2
        SliderPlugin.__init__(self, context)
        
        
    def on_slider1_changed(self, value):
        print "Slider1 changed! Value=%d" % value
        self.panel1.prog.setValue(value)
    
##########################################################################
##########################################################################
class RLegSliderPlugin(SliderPlugin):
##########################################################################
##########################################################################
    def __init__(self, context):
        print "Right Leg Init"
        self.formname = "Right Leg"
        self.appendage_no = 0
        SliderPlugin.__init__(self, context)
        
        
    def on_slider1_changed(self, value):
        print "Slider1 changed! Value=%d" % value
        self.c.slider1 = value
        self.panel1.prog.setValue(value)
    
##########################################################################
##########################################################################
class LLegSliderPlugin(SliderPlugin):
##########################################################################
##########################################################################
    def __init__(self, context):
        print "Left Leg Init"
        self.formname = "Left Leg"
        self.appendage_no = 1
        SliderPlugin.__init__(self, context)
        
        
    def on_slider1_changed(self, value):
        print "Slider1 changed! Value=%d" % value
        self.c.slider1 = value
        self.panel1.prog.setValue(value)
    
##########################################################################
##########################################################################
class HeadSliderPlugin(SliderPlugin):
##########################################################################
##########################################################################
    def __init__(self, context):
        print "Head"
        self.formname = "Head"
        self.appendage_no = 4
        SliderPlugin.__init__(self, context)
        
        
    def on_slider1_changed(self, value):
        print "Slider1 changed! Value=%d" % value
        self.c.slider1 = value
        self.panel1.prog.setValue(value)