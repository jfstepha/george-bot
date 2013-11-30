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
class ButtonPanel(QtGui.QWidget):
##########################################################################
##########################################################################
    def __init__(self, parent):
        
        QtGui.QWidget.__init__(self)
        self.init_ros_params()
        self.parent = parent
        btnw = 40
        btnh = 20
        
        self.panelLayout = QtGui.QGridLayout()
        self.setLayout(self.panelLayout)
        
        self.home_button = QtGui.QPushButton()
        self.home_button.setText('Home')
        self.home_button.clicked.connect( self.on_home_button_clicked )
        self.home_button.setMinimumSize(btnw,btnh)

        self.send_button = QtGui.QPushButton()
        self.send_button.setText('Send')
        self.send_button.clicked.connect( self.on_send_button_clicked )
        self.send_button.setMinimumSize(btnw,btnh)

        self.copy_button = QtGui.QPushButton()
        self.copy_button.setText('Copy positions')
        self.copy_button.clicked.connect( self.on_copy_button_clicked )
        self.copy_button.setMinimumSize(btnw,btnh)

        self.auto_button = QtGui.QPushButton()
        self.auto_button.setText('Auto send')
        self.auto_button.clicked.connect( self.on_auto_button_clicked )
        self.auto_button.setMinimumSize(btnw,btnh)
        self.auto_button.setCheckable(True)

        self.settrim_button = QtGui.QPushButton()
        self.settrim_button.setText('Set Trim')
        self.settrim_button.clicked.connect( self.on_settrim_button_clicked )
        self.settrim_button.setMinimumSize(btnw,btnh)

        self.printtrim_button = QtGui.QPushButton()
        self.printtrim_button.setText('Print Trim')
        self.printtrim_button.clicked.connect( self.on_printtrim_button_clicked )
        self.printtrim_button.setMinimumSize(btnw,btnh)

        self.savetrim_button = QtGui.QPushButton()
        self.savetrim_button.setText('Save Trim')
        self.savetrim_button.clicked.connect( self.on_savetrim_button_clicked )
        self.savetrim_button.setMinimumSize(btnw,btnh)
  
        self.loadtrim_button = QtGui.QPushButton()
        self.loadtrim_button.setText('Load Trim')
        self.loadtrim_button.clicked.connect( self.on_loadtrim_button_clicked )
        self.loadtrim_button.setMinimumSize(btnw,btnh)
  
        self.panelLayout.addWidget(self.home_button,0,0)
        self.panelLayout.addWidget(self.copy_button,0,1)
        self.panelLayout.addWidget(self.send_button,0,2)
        self.panelLayout.addWidget(self.auto_button,0,3)
        self.panelLayout.addWidget(self.settrim_button,1,0)
        self.panelLayout.addWidget(self.printtrim_button,1,1)
        self.panelLayout.addWidget(self.savetrim_button,1,2)
        self.panelLayout.addWidget(self.loadtrim_button,1,3)
        
    def init_ros_params(self): 
        robot_description.ReadParameters()
        self.macro_cmd_pub = rospy.Publisher("macro_cmd", String)
         
    def on_home_button_clicked(self):
        rospy.logdebug("home_button_clicked")
        self.macro_cmd_pub.publish("home")

    def on_send_button_clicked(self):
        rospy.logdebug("send_button_clicked")
        self.macro_cmd_pub.publish("send")

    def on_copy_button_clicked(self):
        rospy.logdebug("copy_button_clicked")
        self.macro_cmd_pub.publish("copy")

    def on_auto_button_clicked(self):
        rospy.logdebug("auto_button_clicked")
        if self.auto_button.isChecked() == True:
            self.macro_cmd_pub.publish("auto_on")
        else:
            self.macro_cmd_pub.publish("auto_off")
            
    def on_printtrim_button_clicked(self):
        rospy.logdebug("print trim button clicked")
        self.macro_cmd_pub.publish("print_trim")

    def on_settrim_button_clicked(self):
        rospy.logdebug("set trim button clicked")
        self.macro_cmd_pub.publish("set_trim")

    def on_savetrim_button_clicked(self):
        rospy.logdebug("save trim button clicked")
        self.macro_cmd_pub.publish("save_trim")

    def on_loadtrim_button_clicked(self):
        rospy.logdebug("load trim button clicked")
        self.macro_cmd_pub.publish("load_trim")
        

##########################################################################
##########################################################################
class CmdrPlugin(Plugin):
##########################################################################
##########################################################################

    def __init__(self, context):
        # robot_description.Print()

        super(CmdrPlugin, self).__init__(context)
        # Give QObjects reasonable names
        #self.setObjectName(self.formname)


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
        self.button_panel = ButtonPanel(self)
        self.mainLayout.addWidget( self.button_panel )

        # central widget
        self.form1 = QWidget()
        self.form1.setLayout(self.mainLayout)
        #self.ui.setLayout(self.mainLayout)

        # set central widget
        
        # Give QObjects reasonable names
        self.form1.setObjectName("George Commander")
        self.form1.setWindowTitle( "George Commander" )
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

        
        
    def on_timer_update(self):
       rospy.logdebug( "-D- tick")
    
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
        