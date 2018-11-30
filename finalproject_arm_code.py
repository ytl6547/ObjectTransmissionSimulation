from pyCreate2 import create2
import math
import odometry
import pid_controller
import lab9_map
import particle_filter
import pyCreate2


class Run:
    def __init__(self, factory):
        """Constructor.

        Args:
            factory (factory.FactoryCreate)
        """
        self.create = factory.create_create()
        self.time = factory.create_time_helper()
        self.servo = factory.create_servo()
        self.sonar = factory.create_sonar()
        self.virtual_create = factory.create_virtual_create()
        self.odometry = odometry.Odometry()
        self.pidTheta = pid_controller.PIDController(200, 0, 100, [-10, 10], [-50, 50], is_angle=True)
        self.map = lab9_map.Map("finalproject_map2.json")
        self.pf = particle_filter.ParticleFilter(self.map, 1000, 0.01, 0.05, 0.1)
        self.arm = factory.create_kuka_lbr4p()

    def sleep(self, time_in_sec):
        """Sleeps for the specified amount of time while keeping odometry up-to-date
        Args:
            time_in_sec (float): time to sleep in seconds
        """
        start = self.time.time()
        while True:
            state = self.create.update()
            if state is not None:
                self.odometry.update(state.leftEncoderCounts, state.rightEncoderCounts)
                print("[{},{},{}]".format(self.odometry.x, self.odometry.y, math.degrees(self.odometry.theta)))
            t = self.time.time()
            if start + time_in_sec <= t:
                break

    def go_to_angle(self, goal_theta):
        old_x = self.odometry.x
        old_y = self.odometry.y
        old_theta = self.odometry.theta
        while math.fabs(math.atan2(
            math.sin(goal_theta - self.odometry.theta),
            math.cos(goal_theta - self.odometry.theta))) > 0.05:
            # print("Go TO: " + str(goal_theta) + " " + str(self.odometry.theta))
            output_theta = self.pidTheta.update(self.odometry.theta, goal_theta, self.time.time())
            self.create.drive_direct(int(+output_theta), int(-output_theta))
            self.sleep(0.01)
        self.create.drive_direct(0, 0)
        self.pf.move_by(self.odometry.x - old_x, self.odometry.y - old_y, self.odometry.theta - old_theta)

    def forward(self):
        old_x = self.odometry.x
        old_y = self.odometry.y
        old_theta = self.odometry.theta
        base_speed = 100
        distance = 0.5
        goal_x = self.odometry.x + math.cos(self.odometry.theta) * distance
        goal_y = self.odometry.y + math.sin(self.odometry.theta) * distance
        print(goal_x, goal_y)
        while True:
            goal_theta = math.atan2(goal_y - self.odometry.y, goal_x - self.odometry.x)
            output_theta = self.pidTheta.update(self.odometry.theta, goal_theta, self.time.time())
            self.create.drive_direct(int(base_speed+output_theta), int(base_speed-output_theta))

            # stop if close enough to goal
            distance = math.sqrt(math.pow(goal_x - self.odometry.x, 2) + math.pow(goal_y - self.odometry.y, 2))
            if distance < 0.05:
                self.create.drive_direct(0, 0)
                break
            self.sleep(0.01)
        self.pf.move_by(self.odometry.x - old_x, self.odometry.y - old_y, self.odometry.theta - old_theta)

    def take_measurements(self):
        angle = -90
        while angle <= 90:
            self.servo.go_to(angle)
            self.time.sleep(2.0)
            distance = self.sonar.get_distance()
            print(distance)
            self.pf.measure(distance, math.radians(angle))
            # self.map.draw(self.pf, "test{}.png".format(self._pos))
            x, y, theta = self.pf.get_estimate()
            self.virtual_create.set_pose((x, y, 0.1), theta)

            data = []
            for particle in self.pf._particles:
                data.extend([particle.x, particle.y, 0.1, particle.theta])

            self.virtual_create.set_point_cloud(data)

            angle += 45
            self._pos += 1

    def visualize(self):
        x, y, theta = self.pf.get_estimate()
        self.virtual_create.set_pose((x, y, 0.1), theta)
        data = []
        for particle in self.pf._particles:
            data.extend([particle.x, particle.y, 0.1, particle.theta])
        self.virtual_create.set_point_cloud(data)

    def lookAround(self):
        # Measure
        distance = self.sonar.get_distance()
        print(distance)
        self.pf.measure(distance, 0)
        if self.visualize():
            return True
        self.time.sleep(0.01)

        # Turn Right
        self.go_to_angle(self.odometry.theta - math.pi / 2)
        self.visualize()
        self.time.sleep(0.01)

        # Measure
        distance = self.sonar.get_distance()
        self.pf.measure(distance, 0)
        if self.visualize():
            return True
        self.time.sleep(0.01)

        # Turn Right
        self.go_to_angle(self.odometry.theta - math.pi / 2)
        self.visualize()
        self.time.sleep(0.01)

        # Measure
        distance = self.sonar.get_distance()
        self.pf.measure(distance, 0)
        if self.visualize():
            return True
        self.time.sleep(0.01)

        # Turn Right
        self.go_to_angle(self.odometry.theta - math.pi / 2)
        self.visualize()
        self.time.sleep(0.01)

        # Measure
        distance = self.sonar.get_distance()
        self.pf.measure(distance, 0)
        if self.visualize():
            return True
        self.time.sleep(0.01)

        # Turn Right
        self.go_to_angle(self.odometry.theta - math.pi / 2)
        self.visualize()
        self.time.sleep(0.01)

        # # Measure
        # distance = self.sonar.get_distance()
        # self.pf.measure(distance, 0)
        # if self.visualize():
        #     return True
        # self.time.sleep(0.01)

        return False

    def run(self):
        # self.create.start()
        # self.create.safe()
        # self.servo.go_to(0)
        # self.create.drive_direct(0, 0)

# go to pick up glass
        self.arm.open_gripper()
        self.time.sleep(1)
        self.arm.go_to(1, 1.6)
        self.time.sleep(3)
        self.arm.go_to(3, .15)
        self.time.sleep(3)
        self.arm.go_to(5, 0)
        self.time.sleep(3)
        self.arm.close_gripper()
        self.time.sleep(5)

# move over wall toward shelf
        for x in range(0, 20):
            self.arm.go_to(1, ((160-x)/100))
            self.time.sleep(1)
            self.arm.go_to(3, (20-x)/133)
            self.time.sleep(0)
        for y in range(0, 30):
            self.arm.go_to(0, (-3.1415 * (y / 30)) / 2)
            self.time.sleep(1)

# go to shelf 0
            for x in range(140, 160):
                self.arm.go_to(1, (x/100))
                self.time.sleep(1)
            for y in range(130, 135):
                self.arm.go_to(3, ((130-y) / 100))
                self.time.sleep(1)
            for c in range(157, 210):
                self.arm.go_to(0, -c/100)
                self.time.sleep(1)
            self.arm.open_gripper()
            self.time.sleep(5)
            self.arm.go_to(1, 1.6)
            self.time.sleep(1)
            self.arm.go_to(0, -3.1415/2)
            self.time.sleep(1)
            self.arm.go_to(1, 0)
            self.arm.go_to(3, 0)
            self.time.sleep(1)

# go to shelf 1
        for x in range(0, 35):
            self.arm.go_to(1, ((140-x)/100))
            self.time.sleep(1)
            self.arm.go_to(3, x/100)
            self.time.sleep(1)
        for c in range(157, 230):
            self.arm.go_to(0, -c/100)
            self.time.sleep(1)
        self.arm.open_gripper()
        self.time.sleep(5)
        self.arm.go_to(1, .8)
        self.time.sleep(1)
        self.arm.go_to(0, -3.1415/2)
        self.time.sleep(1)
        self.arm.go_to(1, 0)
        self.arm.go_to(3, 0)
        self.time.sleep(1)

# go to shelf 2
        for x in range(0, 60):
            self.arm.go_to(1, ((140-x)/100))
            self.time.sleep(1)
        for y in range(0, 30):
            self.arm.go_to(3, y/100)
            self.time.sleep(1)
        for c in range(157, 230):
            self.arm.go_to(0, -c/100)
            self.time.sleep(1)
        self.arm.open_gripper()
        self.time.sleep(5)
        self.arm.go_to(1, .6)
        self.time.sleep(1)
        self.arm.go_to(0, -3.1415/2)
        self.time.sleep(1)
        self.arm.go_to(1, 0)
        self.arm.go_to(3, 0)
        self.time.sleep(1)

        # # request sensors
        # self.create.start_stream([
        #     pyCreate2.Sensor.LeftEncoderCounts,
        #     pyCreate2.Sensor.RightEncoderCounts,
        # ])
        # self.sleep(0.5)
        # self.virtual_create.enable_buttons()
        #
        # self.visualize()
        #
        # # This is an example on how to visualize the pose of our estimated position
        # # where our estimate is that the robot is at (x,y,z)=(0.5,0.5,0.1) with heading pi
        # self.virtual_create.set_pose((0.5, 0.5, 0.1), 0)
        #
        # # This is an example on how to show particles
        # # the format is x,y,z,theta,x,y,z,theta,...
        # data = [0.5, 0.5, 0.1, math.pi / 2, 1.5, 1, 0.1, 0]
        # self.virtual_create.set_point_cloud(data)
        #
        # # This is an example on how to estimate the distance to a wall for the given
        # # map, assuming the robot is at (0.5, 0.5) and has heading math.pi
        # print(self.map.closest_distance((0.5, 0.5), math.pi))
        #
        # # This is an example on how to detect that a button was pressed in V-REP
        # # while True:
        # #     b = self.virtual_create.get_last_button()
        # #     if b == self.virtual_create.Button.MoveForward:
        # #         print("Forward pressed!")
        # #         self.forward()
        # #         self.visualize()
        # #     elif b == self.virtual_create.Button.TurnLeft:
        # #         print("Turn Left pressed!")
        # #         self.go_to_angle(self.odometry.theta + math.pi / 2)
        # #         self.visualize()
        # #     elif b == self.virtual_create.Button.TurnRight:
        # #         print("Turn Right pressed!")
        # #         self.go_to_angle(self.odometry.theta - math.pi / 2)
        # #         self.visualize()
        # #     elif b == self.virtual_create.Button.Sense:
        # #         print("Sense pressed!")
        # #         # TODO Update your particle filter here, e.g.,
        # #         distance = self.sonar.get_distance()
        # #         print("distance:", distance)
        # #         self.pf.measure(distance, 0)
        # #         self.visualize()
        # #
        # #     self.time.sleep(0.01)
        # # time1 = time.time()
        # while True:
        #     if self.lookAround():
        #         break
        #     self.forward()
        #     self.visualize()
        # # print("Time-to-finish:", time.time()-time1)
        #
        # self.sleep(100)
