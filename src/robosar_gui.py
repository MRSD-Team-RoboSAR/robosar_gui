import sys
import os
import math

import rospy
from cv_bridge import CvBridge
from PIL import Image as ImagePIL
from PIL import ImageQt
from PyQt5 import QtWidgets, QtGui
from PyQt5.QtCore import QTime, QTimer
from std_msgs.msg import String, Bool
from sensor_msgs.msg import Image
from robosar_messages.srv import *
from robosar_messages.msg import *
from output import Ui_Dialog


class AgentGroup():
    def __init__(self) -> None:
        self.group_box = None
        self.grid_layout = None
        self.status_label = None
        self.battery_label = None
        self.feedback_label = None
        self.ip_label = None

class Ui(QtWidgets.QDialog):
    def __init__(self):
        super(Ui, self).__init__()
        self.ui = Ui_Dialog()
        self.ui.setupUi(self)
        rospy.init_node('robosar_gui')
        self.dummy_pub = rospy.Publisher('dummy_topic', String, queue_size=1)
        self.task_image_label = self.ui.task_image
        rospy.Subscriber("/task_allocation_image", Image, self.display_task_allocation)
        rospy.Subscriber("/robosar_agent_bringup_node/status",
                         Bool, self.get_active_agents)
        self.agent_active_status = {}
        self.agent_status_dict = {}
        self.start_time = 0.0
        self.elasped_time = 0
        self.start = False
        self.ui.e_stop_button.clicked.connect(self.publish_button_pressed)
        self.ui.start_mission_button.clicked.connect(self.send_mission)
        self.ui.start_timer_button.clicked.connect(self.start_timer)
        self.ui.stop_timer_button.clicked.connect(self.pause_timer)
        self.ui.restart_timer_button.clicked.connect(self.restart_timer)
        self.agent_scroll_area = self.ui.scrollArea
        self.get_active_agents()

        self.mission_timer = QTimer(self)
        self.mission_timer.timeout.connect(self.show_time)

        self.show()

    def display_task_allocation(self, msg):
        print("image received")
        br = CvBridge()
        data = br.imgmsg_to_cv2(msg, "rgb8")
        img = ImagePIL.fromarray(data, mode='RGB')
        qt_img = ImageQt.ImageQt(img)
        pixmap = QtGui.QPixmap.fromImage(qt_img)
        self.task_image_label.setPixmap(pixmap)
        self.task_image_label.adjustSize()

    def show_time(self):
        if self.start:
            self.elasped_time += 1
            mins = str(math.floor(self.elasped_time/60)).zfill(2)
            secs = str(round(self.elasped_time%60)).zfill(2)
            self.ui.mission_timer_label.setText("{}:{}".format(mins, secs))

    def send_mission(self):
        # publish start mission msg

        # start timer
        self.start_timer()
        self.show_time()
        self.mission_timer.start(1000)

    def start_timer(self):
        self.start = True

    def pause_timer(self):
        self.start = False

    def restart_timer(self):
        self.start = False
        self.elasped_time = 0
        self.ui.mission_timer_label.setText("00:00")

    def get_active_agents(self, msg=None):
        print("")
        rospy.wait_for_service('/robosar_agent_bringup_node/agent_status')
        try:
            get_status = rospy.ServiceProxy(
                '/robosar_agent_bringup_node/agent_status', agent_status)
            resp1 = get_status()
            active_agents = resp1.agents_active
            for a in self.agent_active_status:
                self.agent_active_status[a] = False
            for a in active_agents:
                self.agent_active_status[a] = True
            print("{} agents active".format(len(active_agents)))
            assert len(self.agent_active_status) > 0
            self.display_active_agents()
        except rospy.ServiceException as e:
            print("Agent status service call failed: %s" % e)
            raise Exception("Agent status service call failed")

    def display_active_agents(self):
        for agent, status in self.agent_active_status.items():
            alive = "alive" if status else "dead"

            if agent in self.agent_status_dict:
                status_label = self.agent_status_dict[agent].status_label
                status_label.setText(alive)
            else:
                agent_group = AgentGroup()
                agent_group.group_box = QtWidgets.QGroupBox(self.ui.scrollAreaWidgetContents)
                agent_group.group_box.setTitle(agent)
                layout = QtWidgets.QGridLayout()
                agent_group.grid_layout = layout
                layout.setObjectName("{}_grid_layout".format(agent))
                layout.setColumnStretch(0, 4)
                layout.setColumnStretch(1, 8)
                layout.addWidget(QtWidgets.QLabel("Status: "), 0, 0)
                agent_group.status_label = QtWidgets.QLabel(alive)
                layout.addWidget(agent_group.status_label, 0, 1)
                layout.addWidget(QtWidgets.QLabel("Battery Level: "), 1, 0)
                agent_group.battery_label = QtWidgets.QLabel("0")
                layout.addWidget(agent_group.battery_label, 1, 1)
                layout.addWidget(QtWidgets.QLabel(
                    "Feedback Frequency: "), 2, 0)
                agent_group.feedback_label = QtWidgets.QLabel("0")
                layout.addWidget(agent_group.feedback_label, 2, 1)
                layout.addWidget(QtWidgets.QLabel("IP: "), 3, 0)
                agent_group.ip_label = QtWidgets.QLabel("192.168.11.1")
                layout.addWidget(agent_group.ip_label, 3, 1)

                self.agent_status_dict[agent] = agent_group
                agent_group.group_box.setLayout(layout)
                self.ui.verticalLayout.addWidget(agent_group.group_box)

    def publish_button_pressed(self):
        # This is executed when the button is pressed
        print('publish_button_pressed')
        stuff = String()
        stuff.data = "hello"
        self.dummy_pub.publish(stuff)


if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    window = Ui()
    app.exec_()
