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
        self.agent_status_text = self.findChild(QtWidgets.QLabel, 'label')
        self.button.clicked.connect(self.publish_button_pressed)
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
            self.display_active_agents()
        except rospy.ServiceException as e:
            print("Agent status service call failed: %s" % e)
            raise Exception("Agent status service call failed")

    def display_active_agents(self):
        text = ""
        for agent, status in self.agent_active_status.items():
            alive = "alive" if status else "dead"
            single_agent_txt = agent + ": " + alive + "\n"
            text += single_agent_txt
        self.agent_status_text.setText(text)

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
