import sys
import os

import rospy
from PyQt5 import QtWidgets, uic
from std_msgs.msg import String, Bool
from robosar_messages.srv import *
from robosar_messages.msg import *


class Ui(QtWidgets.QDialog):
    def __init__(self):
        super(Ui, self).__init__()
        rospy.init_node('robosar_gui')
        self.dummy_pub = rospy.Publisher('dummy_topic', String, queue_size=1)
        rospy.Subscriber("/robosar_agent_bringup_node/status",
                         Bool, self.get_active_agents)
        uic.loadUi(
            os.path.join(os.path.dirname(os.path.abspath(__file__)), 'robosar_gui.ui'), self)
        self.agent_active_status = {}
        self.button = self.findChild(
            QtWidgets.QPushButton, 'pushButton')  # Find the button
        self.agent_status_dict = {}
        self.button.clicked.connect(self.publish_button_pressed)
        self.agent_vertical_layout = self.findChild(QtWidgets.QVBoxLayout, 'verticalLayout')
        self.get_active_agents()

        self.show()

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
            
        except rospy.ServiceException as e:
            print("Agent status service call failed: %s" % e)
            raise Exception("Agent status service call failed")

    def display_active_agents(self):
        for agent, status in self.agent_active_status.items():
            alive = "alive" if status else "dead"
            
            self.agent_status_dict[agent] = QtWidgets.QGroupBox(agent)
            layout = QtWidgets.QGridLayout()
            layout.setColumnStretch(0, 4)
            layout.setColumnStretch(1, 8)
            layout.addWidget(QtWidgets.QLabel("Status: "), 0, 0)
            layout.addWidget(QtWidgets.QLabel(alive), 0, 1)
            layout.addWidget(QtWidgets.QLabel("Battery Level: "), 1, 0)
            layout.addWidget(QtWidgets.QLabel("Feedback Frequency: "), 2, 0)
            layout.addWidget(QtWidgets.QLabel("IP: "), 3, 0)
            self.agent_status_dict[agent].setLayout(layout)
            self.agent_vertical_layout.addWidget(self.agent_status_dict[agent])

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
