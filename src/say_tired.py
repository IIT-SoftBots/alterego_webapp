#!/usr/bin/env python3

import rospy
from alterego_msgs.msg import BatteryStatus
import time
from alterego_text2speech.src.text2speech import EmbodiedTextToSpeech

class BatteryMonitor:
    def __init__(self):
        rospy.init_node('battery_monitor', anonymous=True)
        
        # Subscribe to battery status
        self.sub = rospy.Subscriber('/robot_adriano/battery/status', BatteryStatus, self.battery_callback)
        
        # Create TTS instance
        self.tts = EmbodiedTextToSpeech()
        
        self.is_charging = False
        self.last_announcement = 0
        self.announcement_interval = 60  # 60 seconds = 1 minute

    def battery_callback(self, msg):
        self.is_charging = msg.is_charging
        self.power_alert = msg.power_alert
        
        # If charging, terminate the node
        if self.is_charging:
            rospy.signal_shutdown("Robot is now charging")
            return

        # If not charging and no power alert and it's been more than 1 minute since last announcement
        current_time = time.time()
        if not self.power_alert and not self.is_charging and (current_time - self.last_announcement) >= self.announcement_interval:
            self.announce_low_battery()
            self.last_announcement = current_time

    def announce_low_battery(self):
        # Use say_edgeTTS_streaming instead of publishing
        message = "Ho le batterie quasi scariche. Avrei bisogno di una mano per essere messo in carica."
        self.tts.say_edgeTTS_streaming(message, language="italian")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        monitor = BatteryMonitor()
        monitor.run()
    except rospy.ROSInterruptException:
        pass
