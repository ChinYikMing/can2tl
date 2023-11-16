import math
import time
import rclpy
from rclpy.node import Node
import lanelet2

from std_msgs.msg import Header
from builtin_interfaces.msg import Time
from autoware_perception_msgs.msg import TrafficSignalArray
from autoware_perception_msgs.msg import TrafficSignalElement
from can_msgs.msg import Frame
from autoware_perception_msgs.msg import TrafficSignal
from autoware_auto_planning_msgs.msg import PathWithLaneId

from enum import Enum

class Color(Enum):
    RED = 1
    AMBER = 2
    GREEN = 3

class Shape(Enum):
    CIRCLE = 1
    LEFT_ARROW = 2
    RIGHT_ARROW = 3
    UP_ARROW = 4

class Status(Enum):
    OFF = 1
    ON = 2

confidence = 1.0

class MinimalPublisher(Node):
    lane_ids = None
    osmMap = None

    def __init__(self):
        super().__init__('minimal_publisher')
        self.osmMap = lanelet2.io.load("../../../ARTC-data/lanelet2/ARTC.osm", lanelet2.io.Origin(0,0))
        self.publisher_ = self.create_publisher(
            TrafficSignalArray,
            'perception/traffic_light_recognition/traffic_signals',
            1)
        self.canSub = self.create_subscription(
            Frame,
            'from_can_bus',
            self.canSub_listener_callback,
            0)
        self.laneSub = self.create_subscription(
            PathWithLaneId,
            'planning/scenario_planning/lane_driving/behavior_planning/path_with_lane_id',
            self.laneSub_listener_callback,
            0)
        #timer_period = 0.1  # 0.1 seconds = 10Hz
        #self.timer = self.create_timer(timer_period, self.listener_callback)
        self.canSub  # prevent unused variable warning
        self.laneSub  # prevent unused variable warning

    def laneSub_listener_callback(self, msg):
        self.lane_ids = msg.points[0].lane_ids
        #print(self.lane_ids)

    def canSub_listener_callback(self, msg):
        state = self.getState(msg.data)
        intersectionId = self.getIntersectionId(msg.data)
        trafficLightState = self.getTrafficLightState(msg.data)
        trafficLightRemainSeconds = self.getTrafficLightRemainSeconds(msg.data)
        counter = self.getCounter(msg.data)

        trafficLightId = None
        if self.osmMap != None and self.lane_ids != None:
            for lane_id in self.lane_ids:
                trafficLightId = self.getTrafficLightId(lane_id)
                if trafficLightId != None: # assume each lane only has one traffic light
                    break

        # we only need to check bit 2 - bit 7 for now
        color = (Color.GREEN.value, Shape.CIRCLE.value, Status.ON.value, confidence)
        trafficLightStateArr = [int(i) for i in "{0:08b}".format(trafficLightState)]
        print(trafficLightStateArr)

        if trafficLightStateArr[0]:   # general red
            color = (Color.RED.value, Shape.CIRCLE.value, Status.ON.value, confidence)
        elif trafficLightStateArr[1]: # general yellow
            color = (Color.AMBER.value, Shape.CIRCLE.value, Status.ON.value, confidence)
        elif trafficLightStateArr[2]: # general green
            color = (Color.GREEN.value, Shape.CIRCLE.value, Status.ON.value, confidence)
        elif trafficLightStateArr[3]: # left green
            color = (Color.GREEN.value, Shape.LEFT.value, Status.ON.value, confidence)
        elif trafficLightStateArr[4]: # straight green
            color = (Color.GREEN.value, Shape.UP.value, Status.ON.value, confidence)
        elif trafficLightStateArr[5]: # right green
            color = (Color.GREEN.value, Shape.RIGHT.value, Status.ON.value, confidence)
        elif trafficLightStateArr[6]: # crosswalk green(reserved)
            pass
        elif trafficLightStateArr[7]: # crosswalk red(reserved)
            pass

        if trafficLightId != None:
            # print("trafficId", trafficLightId)
            trafficSignals = self.trafficSignalsGen(trafficLightId, color)
            self.publisher_.publish(trafficSignals)
            # print("pub")

        # trafficSignals = self.trafficSignalsGen(400004)
        # self.publisher_.publish(trafficSignals)

        #self.get_logger().info(f'state: {state}')
        #self.get_logger().info(f'intersectionId: {intersectionId}')
        #self.get_logger().info(f'trafficLightState: {trafficLightState}')
        #self.get_logger().info(f'trafficLightRemainSeconds: {trafficLightRemainSeconds}')
        #self.get_logger().info(f'counter: {counter}')

    def getTrafficLightId(self, laneId):
        tlId = None
        for elem in self.osmMap.laneletLayer:
            #print("elem_id:",elem.id, "laneId:", laneId)
            if elem.regulatoryElements != [] and elem.id == laneId:
                tl = elem.regulatoryElements.pop()
                tlId = tl.id
                break
        return tlId

    def stampGen(self, frame_id = ''):
        stamp = Time()
        t = self.get_clock().now()
        stamp = t.to_msg()
        return stamp

    def trafficSigEleGen(self, state):
        tse = TrafficSignalElement()
        tse.color = state[0]
        tse.shape = state[1]
        tse.status = state[2]
        tse.confidence = state[3]
        return tse

    def trafficSignalGen(self, tl_id, state):
        ts = TrafficSignal()
        ts.traffic_signal_id = tl_id
        ts.elements.append(self.trafficSigEleGen(state))
        return ts

    def trafficSignalsGen(self, tl_id, state):
        stamp = self.stampGen()
        ts = self.trafficSignalGen(tl_id, state)
        trafficSignals = TrafficSignalArray()
        trafficSignals.stamp = stamp
        trafficSignals.signals.append(ts)
        return trafficSignals

    def getState(self, data):
        return data[0]

    def getIntersectionId(self, data):
        return data[3]

    def getTrafficLightState(self, data):
        return data[4]

    def getTrafficLightRemainSeconds(self, data):
        return (data[6] * 256 + data[5]) * 0.1

    def getCounter(self, data):
        return data[7]

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
