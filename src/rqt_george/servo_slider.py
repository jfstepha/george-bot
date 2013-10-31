import os
import rospy
#from PySide import QtCore
#from PySide import QtGui, QtCore
from python_qt_binding import QtGui
from python_qt_binding import QtCore

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget
##########################################################################
##########################################################################
class Communicate(QtCore.QObject):
##########################################################################
##########################################################################
    slider1 = QtCore.Signal(int)
    pbar1_val = QtCore.Signal(int)
    
##########################################################################
##########################################################################
class sliderPanel(QtGui.QWidget):
##########################################################################
##########################################################################
    def __init__(self, name):
        QtGui.QWidget.__init__(self)
        self.panelLayout = QtGui.QGridLayout()
        self.setLayout(self.panelLayout)
        self.slider = QtGui.QSlider( QtCore.Qt.Horizontal )
        self.lblName = QtGui.QLabel( name )
        self.prog = QtGui.QProgressBar()
        self.panelLayout.addWidget(self.lblName,0,0)
        self.panelLayout.addWidget(self.slider, 0, 1)
        self.panelLayout.addWidget(self.prog, 1, 1)
        

##########################################################################
##########################################################################
class SliderPlugin(Plugin):
##########################################################################
##########################################################################

    def __init__(self, context):
        super(SliderPlugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('SliderPlugin')

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

        # Create the panels
        self.panel1 = sliderPanel("R Shoulder")
        self.panel2 = sliderPanel("R Elbow")

        # main layout
        self.mainLayout = QtGui.QVBoxLayout( )

        # add all main to the main vLayout
        self.mainLayout.addWidget(self.panel1)
        self.mainLayout.addWidget(self.panel2)

        # central widget
        self.form1 = QWidget()
        self.form1.setLayout(self.mainLayout)
        #self.ui.setLayout(self.mainLayout)

        # set central widget
        
        # Give QObjects reasonable names
        self.form1.setObjectName('ServoSliderUI')
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
        
        self.c = Communicate()
        
        self.c.slider1.connect( self.panel1.slider.setValue )
        self.panel1.slider.valueChanged.connect( self.on_slider1_changed )
        self.post_init()
        

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
        self.formname = "Right Arm"
        SliderPlugin.__init__(self, context)
        
    def post_init(self):
        self.x = 1
        
        
    def on_slider1_changed(self, value):
        print "Slider1 changed! Value=%d" % value
        self.c.slider1 = value
        self.panel1.prog.setValue(value)
    