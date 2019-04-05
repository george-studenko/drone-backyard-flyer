import argparse
import time
from enum import Enum

import numpy as np

from udacidrone import Drone
from udacidrone.connection import MavlinkConnection, WebSocketConnection  # noqa: F401
from udacidrone.messaging import MsgID


class States(Enum):
    MANUAL = 0
    ARMING = 1
    TAKEOFF = 2
    WAYPOINT = 3
    LANDING = 4
    DISARMING = 5


class BackyardFlyer(Drone):
    def __init__(self, connection):
        super().__init__(connection)
        self.target_position = np.array([0.0, 0.0, 0.0])
        self.all_waypoints = []
        self.in_mission = True
        self.check_state = {}
        self.current_waypoint = 0
        self.starting_point = [0,0,0]
        self.navigating = False

        # initial state
        self.flight_state = States.MANUAL

        # Register all your callbacks
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

    def local_position_callback(self):
        """
        This triggers when `MsgID.LOCAL_POSITION` is received and self.local_position contains new data
        """

        if self.flight_state == States.TAKEOFF:
            # coordinate conversion
            altitude = -1.0 * self.local_position[2]
            self.starting_point = [self.local_position[0], self.local_position[1], (self.local_position[2] * -1)]
            # check if altitude is within 95% of target
            self.all_waypoints = self.get_box()
            print(self.local_position)
            self.flight_state = States.WAYPOINT
        elif self.local_position[2] * -1.0 > 2.98 and self.flight_state == States.WAYPOINT and self.navigating == False:  # * self.target_position[2]:
            #  self.landing_transition()
            self.navigating = True
            self.waypoint_transition()
        elif np.isclose(self.target_position[0], self.local_position[0], rtol=1e-1, atol=0.01) and np.isclose(self.target_position[1], self.local_position[1], rtol=1e-1, atol=0.01):
            self.navigating = False

        elif self.current_waypoint == 4 and np.isclose(self.all_waypoints[3][0], self.local_position[0], rtol=1e-1, atol=0.1) and np.isclose(self.all_waypoints[3][1], self.local_position[1], rtol=1e-1, atol=0.1):
            self.flight_state = States.LANDING




    def velocity_callback(self):
        """
        This triggers when `MsgID.LOCAL_VELOCITY` is received and self.local_velocity contains new data
        """
        #if self.flight_state == States.LANDING:
        #    if ((self.global_position[2] - self.global_home[2] < 0.1) and
        #                abs(self.local_position[2]) < 0.01):
        #        self.disarming_transition()
        pass

    def state_callback(self):
        """
        This triggers when `MsgID.STATE` is received and self.armed and self.guided contain new data
        """
        if not self.in_mission:
            return
        if self.flight_state == States.MANUAL:
            # now just passively waiting for the pilot to change these attributes
            # once the pilot changes, need to update our internal state
            if self.guided:
                self.flight_state = States.ARMING
        elif self.flight_state == States.ARMING:
            if self.armed:
                self.takeoff_transition()
        elif self.flight_state == States.LANDING:
            self.landing_transition()
        elif self.flight_state == States.DISARMING:
            if not self.armed and not self.guided:
                self.manual_transition()

    def arming_transition(self):
        """
        1. Take control of the drone
        2. Pass an arming command
        3. Set the home location to current position
        4. Transition to the ARMING state
        """
        print("arming transition")
        self.take_control()
        self.arm()
        self.set_home_position(self.global_position[0],
                               self.global_position[1],
                               self.global_position[2])
        self.flight_state = States.ARMING

    def get_box(self):
        """
         1. Return waypoints to fly a box
        """
        print('getting box waypoints')
        waypoints = [
            [self.starting_point[0] + 5, self.starting_point[1] + 0, self.starting_point[2] + 3, 0],
            [self.starting_point[0] + 5, self.starting_point[1] + 5, self.starting_point[2] + 3, 0],
            [self.starting_point[0] + 0, self.starting_point[1] + 5, self.starting_point[2] + 3, 0],
            [self.starting_point[0] + 0, self.starting_point[1] + 0, self.starting_point[2] + 3, 0]
            ]
        return waypoints


    def takeoff_transition(self):
        """
        1. Set target_position altitude to 3.0m
        2. Command a takeoff to 3.0m
        3. Transition to the TAKEOFF state
        """
        print("takeoff transition")
        self.target_position = np.array([0.0, 0.0, 3.0])
        self.takeoff(3)
        self.flight_state = States.TAKEOFF

    def waypoint_transition(self):
        """
        1. Command the next waypoint position
        2. Transition to WAYPOINT state
        """

        print("waypoint transition")

        if self.current_waypoint < len(self.all_waypoints):
            next_waypoint = self.all_waypoints[self.current_waypoint]
            self.current_waypoint = self.current_waypoint + 1
            self.cmd_position(*next_waypoint)
            self.target_position = next_waypoint



    def landing_transition(self):
        """
        1. Command the drone to land
        2. Transition to the LANDING state
        """
        print("landing transition")
        self.land()
        self.flight_state = States.LANDING

    def disarming_transition(self):
        """
        1. Command the drone to disarm
        2. Transition to the DISARMING state
        """
        print("disarm transition")
        self.disarm()
        self.flight_state = States.DISARMING

    def manual_transition(self):
        """This method is provided

        1. Release control of the drone
        2. Stop the connection (and telemetry log)
        3. End the mission
        4. Transition to the MANUAL state
        """
        print("manual transition")

        self.release_control()
        self.stop()
        self.in_mission = False
        self.flight_state = States.MANUAL

    def start(self):
        """This method is provided

        1. Open a log file
        2. Start the drone connection
        3. Close the log file
        """
        print("Creating log file")
        self.start_log("Logs", "NavLog.txt")
        print("starting connection")
        self.connection.start()
        print("Closing log file")
        self.stop_log()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=14550, help='Port number')
    parser.add_argument('--host', type=str, default='192.168.1.182', help="host address, i.e. '127.0.0.1'")
    args = parser.parse_args()

    conn = MavlinkConnection('udp:{0}:{1}'.format(args.host, args.port), threaded=False, PX4=True)
    # conn = WebSocketConnection('ws://{0}:{1}'.format(args.host, args.port))
    drone = BackyardFlyer(conn)
    time.sleep(2)
    drone.start()
