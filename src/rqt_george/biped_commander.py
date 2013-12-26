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
from moveit_commander import MoveGroupCommander 
from BipedCommander import *

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
        self.bipedCommander = BipedCommander()
        self.lfoot_group = MoveGroupCommander("legs")
        self.rfoot_group = MoveGroupCommander("legs")
        
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
        
        self.lfoot_button= QtGui.QPushButton()
        self.lfoot_button.setText("LFoot")
        self.lfoot_button.clicked.connect( self.on_lfoot_button_clicked )
        self.lfoot_button.setMinimumSize(btnw,btnh)
        
        self.lfoot_x = QtGui.QLineEdit("0.045")
        self.lfoot_y = QtGui.QLineEdit("0.000")
        self.lfoot_z = QtGui.QLineEdit("-0.19")

        self.rfoot_button= QtGui.QPushButton()
        self.rfoot_button.setText("RFoot")
        self.rfoot_button.clicked.connect( self.on_rfoot_button_clicked )
        self.rfoot_button.setMinimumSize(btnw,btnh)
        
        self.rfoot_x = QtGui.QLineEdit("-0.045")
        self.rfoot_y = QtGui.QLineEdit("0.000")
        self.rfoot_z = QtGui.QLineEdit("-0.19")

        self.larm_button= QtGui.QPushButton()
        self.larm_button.setText("LArm")
        self.larm_button.clicked.connect( self.on_larm_button_clicked )
        self.larm_button.setMinimumSize(btnw,btnh)
        
        self.larm_x = QtGui.QLineEdit("0.278")
        self.larm_y = QtGui.QLineEdit("0.059")
        self.larm_z = QtGui.QLineEdit("0.089")

        self.rarm_button= QtGui.QPushButton()
        self.rarm_button.setText("RArm")
        self.rarm_button.clicked.connect( self.on_rarm_button_clicked )
        self.rarm_button.setMinimumSize(btnw,btnh)
        
        self.rarm_x = QtGui.QLineEdit("-0.213")
        self.rarm_y = QtGui.QLineEdit("0.060")
        self.rarm_z = QtGui.QLineEdit("0.069")

        self.print_pose_button= QtGui.QPushButton()
        self.print_pose_button.setText("pose print")
        self.print_pose_button.clicked.connect( self.on_print_pose_button_clicked )
        self.print_pose_button.setMinimumSize(btnw,btnh)
  
        self.legs_button= QtGui.QPushButton()
        self.legs_button.setText("both feet")
        self.legs_button.clicked.connect( self.on_legs_button_clicked )
        self.legs_button.setMinimumSize(btnw,btnh)

        self.walk1_button= QtGui.QPushButton()
        self.walk1_button.setText("walk1")
        self.walk1_button.clicked.connect( self.on_walk1_button_clicked )
        self.walk1_button.setMinimumSize(btnw,btnh)

        self.walk2_button= QtGui.QPushButton()
        self.walk2_button.setText("walk lstep")
        self.walk2_button.clicked.connect( self.on_walk2_button_clicked )
        self.walk2_button.setMinimumSize(btnw,btnh)

        self.walk3_button= QtGui.QPushButton()
        self.walk3_button.setText("walk rstep")
        self.walk3_button.clicked.connect( self.on_walk3_button_clicked )
        self.walk3_button.setMinimumSize(btnw,btnh)

        self.walk4_button= QtGui.QPushButton()
        self.walk4_button.setText("rstep2home")
        self.walk4_button.clicked.connect( self.on_walk4_button_clicked )
        self.walk4_button.setMinimumSize(btnw,btnh)
  
        self.panelLayout.addWidget(self.home_button,0,0)
        self.panelLayout.addWidget(self.copy_button,0,1)
        self.panelLayout.addWidget(self.send_button,0,2)
        self.panelLayout.addWidget(self.auto_button,0,3)
        self.panelLayout.addWidget(self.settrim_button,1,0)
        self.panelLayout.addWidget(self.printtrim_button,1,1)
        self.panelLayout.addWidget(self.savetrim_button,1,2)
        self.panelLayout.addWidget(self.loadtrim_button,1,3)
        self.panelLayout.addWidget(self.lfoot_button,2,0)
        self.panelLayout.addWidget(self.lfoot_x,2,1)
        self.panelLayout.addWidget(self.lfoot_y,2,2)
        self.panelLayout.addWidget(self.lfoot_z,2,3)
        self.panelLayout.addWidget(self.rfoot_button,3,0)
        self.panelLayout.addWidget(self.rfoot_x,3,1)
        self.panelLayout.addWidget(self.rfoot_y,3,2)
        self.panelLayout.addWidget(self.rfoot_z,3,3)
        self.panelLayout.addWidget(self.larm_button,4,0)
        self.panelLayout.addWidget(self.larm_x,4,1)
        self.panelLayout.addWidget(self.larm_y,4,2)
        self.panelLayout.addWidget(self.larm_z,4,3)
        self.panelLayout.addWidget(self.rarm_button,5,0)
        self.panelLayout.addWidget(self.rarm_x,5,1)
        self.panelLayout.addWidget(self.rarm_y,5,2)
        self.panelLayout.addWidget(self.rarm_z,5,3)
        self.panelLayout.addWidget(self.print_pose_button,6,0)
        self.panelLayout.addWidget(self.legs_button,6,1)
        self.panelLayout.addWidget(self.walk1_button,7,0)
        self.panelLayout.addWidget(self.walk2_button,7,1)
        self.panelLayout.addWidget(self.walk3_button,7,2)
        self.panelLayout.addWidget(self.walk4_button,7,3)
        
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

    def on_lfoot_button_clicked(self):
        rospy.logdebug("lfoot button clicked")
        end_effector_link="lfoot"
        x = float(self.lfoot_x.text())
        y = float(self.lfoot_y.text())
        z = float(self.lfoot_z.text())
        self.bipedCommander.move_lfoot( x,y,z )

    def on_rfoot_button_clicked(self):
        rospy.logdebug("rfoot button clicked")
        end_effector_link="rfoot"
        x = float(self.rfoot_x.text())
        y = float(self.rfoot_y.text())
        z = float(self.rfoot_z.text())
        self.bipedCommander.move_rfoot( x,y,z )

    def on_larm_button_clicked(self):
        rospy.logdebug("larm button clicked")
        x = float(self.larm_x.text())
        y = float(self.larm_y.text())
        z = float(self.larm_z.text())
        self.bipedCommander.move_larm( x,y,z )

    def on_rarm_button_clicked(self):
        rospy.logdebug("rarm button clicked")
        x = float(self.rarm_x.text())
        y = float(self.rarm_y.text())
        z = float(self.rarm_z.text())
        self.bipedCommander.move_rarm( x,y,z )

    def on_legs_button_clicked(self):
        rospy.logdebug("legs button clicked")
        lx = float(self.lfoot_x.text())
        ly = float(self.lfoot_y.text())
        lz = float(self.lfoot_z.text())

        rx = float(self.rfoot_x.text())
        ry = float(self.rfoot_y.text())
        rz = float(self.rfoot_z.text())
        
        self.bipedCommander.move_legs( lx, ly, lz, rx, ry, rz)
        

    def on_print_pose_button_clicked(self):
        self.bipedCommander.pose_print()

    def on_walk1_button_clicked(self):
        self.bipedCommander.walk_first_step()

    def on_walk2_button_clicked(self):
        self.bipedCommander.walk_left_step()

    def on_walk3_button_clicked(self):
        self.bipedCommander.walk_right_step()

    def on_walk4_button_clicked(self):
        self.bipedCommander.walk_right2home()
        
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
       # rospy.logdebug( "-D- tick")
       pass
    
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
        