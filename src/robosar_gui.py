# GUI using PyQt5 and ROS
#
# This is a GUI for the RoboSARe project. It is written in Python3 and uses PyQt5 for the GUI.
# It also uses ROS to communicate with the rest of the system.
#
# The GUI is split into two parts. The first part is the task allocation image. This is a
# representation of the task allocation. The second part is the status of the agents. This
# shows the status of each agent, including the battery level, the IP address, and the
# feedback from the agent.
# 
#  contributor: @githubco-pilot
import sys
import math

import rospy
from cv_bridge import CvBridge, CvBridgeError
from PyQt5 import QtWidgets, QtGui
from PyQt5.QtCore import QTimer, QSize
from std_msgs.msg import String, Bool, Int32
from sensor_msgs.msg import Image
from robosar_messages.srv import *
from robosar_messages.msg import *
from gui_designer import Ui_Dialog

ALIVE = ["ROBOT_STATUS_ACTIVE", "ROBOT_STATUS_INACTIVE"]
DEAD = ["ROBOT_STATUS_COMM_FAIL",
        "ROBOT_STATUS_UNREACHABLE", "ROBOT_STATUS_NO_HEARTBEAT"]


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

        # key: agent name, value: Bool
        self.agent_active_status = {}
        # key: agent name, value: AgentGroup
        self.agent_status_dict = {}

        # init ROS stuff
        rospy.init_node('robosar_gui')
        self.tc_pub = rospy.Publisher('/system_mission_command', mission_command, queue_size=5)
        rospy.Subscriber("/task_allocation_image", Image,
                         self.display_task_allocation)
        rospy.Subscriber("/robosar_agent_bringup_node/status",
                         Bool, self.get_active_agents)
        self.get_active_agents()
        rospy.Subscriber("/robosar_agent_bringup_node/all_agent_status",
                         agents_status, self.display_agents_status)
        rospy.Subscriber("/tasks_completed", Int32, self.display_task_count)

        self.start_time = 0.0
        self.elasped_time = 0
        self.start = False
        self.ui.start_mission_button.clicked.connect(self.send_mission)
        self.ui.e_stop_button.clicked.connect(self.send_estop)
        self.ui.homing_button.clicked.connect(self.send_homing)
        self.ui.start_timer_button.clicked.connect(self.start_timer)
        self.ui.stop_timer_button.clicked.connect(self.pause_timer)
        self.ui.restart_timer_button.clicked.connect(self.restart_timer)
        self.agent_scroll_area = self.ui.scrollArea

        self.mission_timer = QTimer(self)
        self.mission_timer.timeout.connect(self.show_time)

        self.show()

    def display_task_count(self, msg):
        self.ui.tasks_completed_label.setText(str(msg.data))

    def display_task_allocation(self, msg):
        if msg:
            br = CvBridge()
            try:
                data = br.imgmsg_to_cv2(msg, "rgb8")
                height, width, _ = data.shape
                bytesPerLine = 3 * width
                qImg = QtGui.QImage(data.data, width, height,
                                    bytesPerLine, QtGui.QImage.Format_RGB888)
                pixmap = QtGui.QPixmap.fromImage(qImg)
                self.ui.task_image.setPixmap(pixmap)
            except CvBridgeError as e:
                print(e)

    def show_time(self):
        if self.start:
            self.elasped_time += 1
            mins = str(math.floor(self.elasped_time/60)).zfill(2)
            secs = str(round(self.elasped_time % 60)).zfill(2)
            self.ui.mission_timer_label.setText("{}:{}".format(mins, secs))

    def send_mission(self):
        # publish start mission msg
        start_msg = mission_command()
        start_msg.data = mission_command.START
        self.tc_pub.publish(start_msg)

        # start timer
        self.start_timer()
        self.show_time()
        self.mission_timer.start(1000)

    def send_estop(self):
        print("sending e-stop command")
        start_msg = mission_command()
        start_msg.data = mission_command.STOP
        self.tc_pub.publish(start_msg)

    def send_homing(self):
        print("sending homing command")
        start_msg = mission_command()
        start_msg.data = mission_command.HOME
        self.tc_pub.publish(start_msg)

    def start_timer(self):
        self.start = True

    def pause_timer(self):
        self.start = False

    def restart_timer(self):
        self.start = False
        self.elasped_time = 0
        self.ui.mission_timer_label.setText("00:00")

    def get_active_agents(self, msg=None):
        print("Waiting for agent_status service ...")
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
            self.display_active_agents()
        except rospy.ServiceException as e:
            print("Agent status service call failed: %s" % e)
            raise Exception("Agent status service call failed")

    def display_active_agents(self):
        num_active = 0
        for agent, status in self.agent_active_status.items():
            num_active += 1 if status else 0
            alive = "ALIVE" if status else "DEAD"

            if agent in self.agent_status_dict:
                status_label = self.agent_status_dict[agent].status_label
                status_label.setText(alive)
            else:
                agent_group = AgentGroup()
                agent_group.group_box = QtWidgets.QGroupBox(
                    self.ui.scrollAreaWidgetContents)
                agent_group.group_box.setTitle(agent)
                agent_group.group_box.setMinimumSize(QSize(300, 100))
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
        self.ui.active_agents_label.setText(str(num_active))

    def display_agents_status(self, msg):
        num_active = 0
        for i in range(len(msg.robot_id)):
            agent = msg.robot_id[i]
            status = msg.status[i]
            splitted = status.split("_")
            status_short = '_'.join(splitted[2:])
            print("{}, {}".format(agent, status))

            if status in ALIVE:
                num_active += 1
                if agent in self.agent_status_dict:
                    agent_group = self.agent_status_dict[agent]
                    agent_group.group_box.setStyleSheet(
                        "QGroupBox {background-color: white;}")
                    agent_group.status_label.setText(status_short)
                    agent_group.battery_label.setText(str(msg.battery_lvl[i]))
                    agent_group.feedback_label.setText(
                        str(msg.feedback_freq[i]))
            else:
                if agent in self.agent_status_dict:
                    agent_group = self.agent_status_dict[agent]
                    agent_group.group_box.setStyleSheet(
                        "QGroupBox {background-color: grey;}")
                    agent_group.status_label.setText(status_short)
                    agent_group.battery_label.setText("")
                    agent_group.feedback_label.setText("")
            self.ui.active_agents_label.setText(str(num_active))


if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    window = Ui()
    app.exec_()
