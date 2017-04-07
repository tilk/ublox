import os
import rospy
import rospkg

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget

from ublox_msgs.msg import NavPVT, NavDGPS

class UBloxStatus(Plugin):
    fixTypes = {0: "no fix", 1: "DR", 2: "2D", 3: "3D", 4: "DR+GNSS", 5: "TIME"}

    def __init__(self, context):
        super(UBloxStatus, self).__init__(context)
        self.setObjectName('UBloxStatus')

        self._widget = QWidget()
        ui_file = os.path.join(rospkg.RosPack().get_path('rqt_ublox'), 'resource', 'UBloxStatus.ui')
        loadUi(ui_file, self._widget)
        self._widget.setObjectName('UBloxStatusUi')

        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))

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
        fixtext = self.fixTypes[m.fixType]
        if m.flags & NavPVT.FLAGS_DIFFSOLN: fixtext += "+DGNSS"
        if m.flags & NavPVT.FLAGS_CARRSOLN_MASK: fixtext += "+CP"
        self._widget.fixMode.setText(fixtext)
        self._widget.utcTime.setText("%0.4d-%0.2d-%0.2d %0.2d:%0.2d:%0.2d" % (m.year, m.month, m.day, m.hour, m.min, m.sec))
        self._widget.lon.setText("%.4f" % (m.lon / 1e7))
        self._widget.lat.setText("%.4f" % (m.lat / 1e7))
        self._widget.alt.setText("%.2f m" % (m.height / 1e3))
        self._widget.altMSL.setText("%.2f m" % (m.hMSL / 1e3))
        self._widget.hAcc.setText("%.2f m" % (m.hAcc / 1e4))
        self._widget.vAcc.setText("%.2f m" % (m.vAcc / 1e4))
        self._widget.pDOP.setText("%.2f" % (m.pDOP / 1e2))
        self._widget.sat.setText("%d" % m.numSV)
        self._widget.spd.setText("%.2f m/s" % (m.gSpeed / 1e3))
        self._widget.spdAcc.setText("%.2f m/s" % (m.sAcc / 1e3))
        self._widget.head.setText("%.2f" % (m.headMot / 1e5))
        self._widget.headAcc.setText("%.2f" % (m.headAcc / 1e5))
        self._widget.dateValid.setChecked(m.valid & NavPVT.VALID_DATE)
        self._widget.timeValid.setChecked(m.valid & NavPVT.VALID_TIME)
        self._widget.timeResolved.setChecked(m.valid & NavPVT.VALID_FULLYRESOLVED)

    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog
 
class UBloxSatellites(Plugin):
    def __init__(self, context):
        super(UBloxDGPS, self).__init__(context)
        self.setObjectName('UBloxSatellites')

        self._widget = QWidget()
        ui_file = os.path.join(rospkg.RosPack().get_path('rqt_ublox'), 'resource', 'UBloxSatellites.ui')
        loadUi(ui_file, self._widget)
        self._widget.setObjectName('UBloxSatellites')

        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))

        context.add_widget(self._widget)

        self.navdgps_subscriber = rospy.Subscriber("/gnss/navdgps", NavDGPS, self.dgps_callback)
    
    def shutdown_plugin(self):
        self.navdgps_subscriber.unregister()
    
    def dgps_callback(self, m):
        pass

