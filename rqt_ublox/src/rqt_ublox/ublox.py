import os
import rospy
import rospkg

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget

from ublox_msgs.msg import NavPVT

class UBloxStatus(Plugin):

    def __init__(self, context):
        super(UBloxStatus, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('UBloxStatus')

        # Create QWidget
        self._widget = QWidget()
        # Get path to UI file which should be in the "resource" folder of this package
        ui_file = os.path.join(rospkg.RosPack().get_path('rqt_ublox'), 'resource', 'UBloxStatus.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('UBloxStatusUi')
        # Show _widget.windowTitle on left-top of each plugin (when 
        # it's set in _widget). This is useful when you open multiple 
        # plugins at once. Also if you open multiple instances of your 
        # plugin at once, these lines add number to make it easy to 
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)

        self.navpvt_subscriber = rospy.Subscriber("/gnss/navpvt", NavPVT, self.pvt_callback)

    def shutdown_plugin(self):
        self.navpvt_subscriber.unregister()

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

    def pvt_callback(self, m):
        print "Callback! %d" % m.iTOW
        self._widget.lon.display(m.lon / 1e7)
        self._widget.lat.display(m.lat / 1e7)
        self._widget.alt.display(m.height / 1e3)
        self._widget.altMSL.display(m.hMSL / 1e3)
        self._widget.hAcc.display(m.hAcc / 1e4)
        self._widget.vAcc.display(m.vAcc / 1e4)
        self._widget.pDOP.display(m.pDOP / 1e2)
        self._widget.sat.display(m.numSV)
        self._widget.spd.display(m.gSpeed / 1e3)
        self._widget.spdAcc.display(m.sAcc / 1e3)
        self._widget.head.display(m.headMot / 1e5)
        self._widget.headAcc.display(m.headAcc / 1e5)

    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog
