import os
import rospy
import rospkg

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtCore import pyqtSignal
from python_qt_binding.QtWidgets import QWidget

from ublox_msgs.msg import NavPVT, NavSAT, NavSAT_SV, NavDOP

class UBloxStatus(Plugin):
    fixTypes = {0: "no fix", 1: "DR", 2: "2D", 3: "3D", 4: "DR+GNSS", 5: "TIME"}
    pvt_signal = pyqtSignal(NavPVT)

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

        self.pvt_signal.connect(self.pvt_slot)

        self.navpvt_subscriber = rospy.Subscriber("/gnss/navpvt", NavPVT, self.pvt_signal.emit)

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

    def pvt_slot(self, m):
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
    gnssTypes = {0: "GPS", 1: "SBAS", 2: "Galileo", 3: "BeiDou", 4: "IMES", 5: "QZSS", 6: "GLONASS"}
    qualities = {
        NavSAT_SV.FLAGS_QUALITY_NO: "no signal",
        NavSAT_SV.FLAGS_QUALITY_SEARCHING: "searching",
        NavSAT_SV.FLAGS_QUALITY_ACQUIRED: "acquired",
        NavSAT_SV.FLAGS_QUALITY_UNUSABLE: "unusable",
        NavSAT_SV.FLAGS_QUALITY_CODE_LOCKED: "code lock",
        NavSAT_SV.FLAGS_QUALITY_CARRIER_LOCKED1: "carrier lock",
        NavSAT_SV.FLAGS_QUALITY_CARRIER_LOCKED2: "carrier lock",
        NavSAT_SV.FLAGS_QUALITY_CARRIER_LOCKED3: "carrier lock"
    }
    update_signal = pyqtSignal(NavSAT)

    def __init__(self, context):
        super(UBloxSatellites, self).__init__(context)
        self.setObjectName('UBloxSatellites')

        self._widget = QWidget()
        ui_file = os.path.join(rospkg.RosPack().get_path('rqt_ublox'), 'resource', 'UBloxSatellites.ui')
        loadUi(ui_file, self._widget)
        self._widget.setObjectName('UBloxSatellites')

        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))

        context.add_widget(self._widget)

        self.update_signal.connect(self.sat_slot)
        self.navdgps_subscriber = rospy.Subscriber("/gnss/navsat", NavSAT, self.update_signal.emit)

        self.ui_sat_file = os.path.join(rospkg.RosPack().get_path('rqt_ublox'), 'resource', 'UBloxSatelliteWidget.ui')

    def newSatWidget(self):
        satWidget = QWidget()
        loadUi(self.ui_sat_file, satWidget)
        return satWidget

    def updateSatWidget(self, wdgt, sv):
        wdgt.carrierToNoise.setValue(sv.cno)
        wdgt.satelliteId.setText("%d" % sv.svId)
        wdgt.gnssType.setText(self.gnssTypes[sv.gnssId])
        wdgt.quality.setText(self.qualities[sv.flags & NavSAT_SV.FLAGS_QUALITY_MASK])
        wdgt.elevazim.setText("El %d Az %d" % (sv.elev, sv.azim))
        wdgt.used.setChecked(sv.flags & NavSAT_SV.FLAGS_SVUSED)
        wdgt.eph.setChecked(sv.flags & NavSAT_SV.FLAGS_EPH_AVAIL)
        wdgt.alm.setChecked(sv.flags & NavSAT_SV.FLAGS_ALM_AVAIL)
        wdgt.sbas.setChecked(sv.flags & NavSAT_SV.FLAGS_SBAS_CORR_USED)
        wdgt.rtcm.setChecked(sv.flags & NavSAT_SV.FLAGS_RTCM_CORR_USED)
        wdgt.pr.setChecked(sv.flags & NavSAT_SV.FLAGS_PR_CORR_USED)
        wdgt.cr.setChecked(sv.flags & NavSAT_SV.FLAGS_CR_CORR_USED)
        wdgt.dop.setChecked(sv.flags & NavSAT_SV.FLAGS_DO_CORR_USED)

    def shutdown_plugin(self):
        self.navdgps_subscriber.unregister()
    
    def sat_slot(self, m):
        lay = self._widget.satContents.layout()
        while lay.count():
            child = lay.takeAt(0)
            if child.widget(): child.widget().deleteLater()
        for sv in m.sv:
            w = self.newSatWidget()
            self.updateSatWidget(w, sv)
            lay.addWidget(w)

class UBloxDOP(Plugin):
    dop_signal = pyqtSignal(NavDOP)

    def __init__(self, context):
        super(UBloxDOP, self).__init__(context)
        self.setObjectName('UBloxDOP')

        self._widget = QWidget()
        ui_file = os.path.join(rospkg.RosPack().get_path('rqt_ublox'), 'resource', 'UBloxDOP.ui')
        loadUi(ui_file, self._widget)
        self._widget.setObjectName('UBloxDOPUi')

        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))

        context.add_widget(self._widget)

        self.dop_signal.connect(self.dop_slot)

        self.navdop_subscriber = rospy.Subscriber("/gnss/navdop", NavDOP, self.dop_signal.emit)

    def shutdown_plugin(self):
        self.navdop_subscriber.unregister()

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

    def dop_slot(self, m):
        self._widget.gDOP.setText("%.2f" % (m.gDOP/1e2))
        self._widget.pDOP.setText("%.2f" % (m.pDOP/1e2))
        self._widget.tDOP.setText("%.2f" % (m.tDOP/1e2))
        self._widget.vDOP.setText("%.2f" % (m.vDOP/1e2))
        self._widget.hDOP.setText("%.2f" % (m.hDOP/1e2))
        self._widget.nDOP.setText("%.2f" % (m.nDOP/1e2))
        self._widget.eDOP.setText("%.2f" % (m.eDOP/1e2))

