import pyCreate2 as create2
import math
import odometry
import pid_controller
import lab9_map
import particle_filter
import time
import numpy as np
import lab11_map
import rrt

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
        self.arm = factory.create_kuka_lbr4p()
        self.virtual_create = factory.create_virtual_create()
        # self.virtual_create = factory.create_virtual_create("192.168.1.XXX")
        self.odometry = odometry.Odometry()
        self.map = lab9_map.Map("finalproject_map2.json")

        # TODO identify good PID controller gains
        self.pidTheta = pid_controller.PIDController(200, 0, 100, [-10, 10], [-50, 50], is_angle=True)
        # TODO identify good particle filter parameters
        # self.pf = particle_filter.ParticleFilter(self.map, 1000, 0.06, 0.15, 0.2)
        self.vc_x = 0
        self.vc_y = 0
        self.vc_theta = 0
        self.pf = particle_filter.ParticleFilter(self.map, 1500, 0.06, 0.15, 0.2)
        self.step_dist = 0.5

        # RRT
        self.map2 = lab11_map.Map("finalproject_map2_config.png")
        self.rrt = rrt.RRT(self.map2)

        self.joint_angles = np.zeros(7)

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
                # print("[{},{},{}]".format(self.odometry.x, self.odometry.y, math.degrees(self.odometry.theta)))
            t = self.time.time()
            if start + time_in_sec <= t:
                break

    def go_to_angle(self, goal_theta):
        old_x = self.odometry.x
        old_y = self.odometry.y
        old_theta = self.odometry.theta
        while math.fabs(math.atan2(
            math.sin(goal_theta - self.odometry.theta),
            math.cos(goal_theta - self.odometry.theta))) > 0.01:
            output_theta = self.pidTheta.update(self.odometry.theta, goal_theta, self.time.time())
            self.create.drive_direct(int(+output_theta), int(-output_theta))
            self.sleep(0.01)
        # self.create.drive_direct(-8, 8)
        # self.sleep(1)
        self.create.drive_direct(0, 0)
        self.pf.move_by(self.odometry.x - old_x, self.odometry.y - old_y, self.odometry.theta - old_theta)

    def forward(self):
        old_x = self.odometry.x
        old_y = self.odometry.y
        old_theta = self.odometry.theta
        base_speed = 100
        distance = self.step_dist
        goal_x = self.odometry.x + math.cos(self.odometry.theta) * distance
        goal_y = self.odometry.y + math.sin(self.odometry.theta) * distance
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

    def visualize(self):
        flag = True
        x, y, theta = self.pf.get_estimate()
        print("Estimated", x, y)
        self.virtual_create.set_pose((x, y, 0.1), theta)
        data = []
        for particle in self.pf._particles:
            data.extend([particle.x, particle.y, 0.1, particle.theta])
            if math.hypot(x - particle.x, y - particle.y) > 0.6:
                flag = False
        self.virtual_create.set_point_cloud(data)

        if flag is True:
            self.vc_x = x
            self.vc_y = y
            self.vc_theta = theta

        return flag

    def lookAround(self):
        converged = False
        angle = -90
        while angle <= 90:
            self.servo.go_to(angle)
            self.time.sleep(2.0)
            distance = self.sonar.get_distance()
            # print(distance)
            self.pf.measure(distance, math.radians(angle))
            angle += 90
            converged |= self.visualize()
        self.servo.go_to(0)
        self.time.sleep(2)
        if converged:
            return True
        return False

    def run(self):
        self.create.start()
        self.create.safe()

        self.create.drive_direct(0, 0)

        # self.arm.open_gripper()

        self.time.sleep(4)

        # self.arm.close_gripper()

        # request sensors
        self.create.start_stream([
            create2.Sensor.LeftEncoderCounts,
            create2.Sensor.RightEncoderCounts,
        ])
        self.sleep(0.5)
        self.visualize()
        # self.virtual_create.enable_buttons()

        self.lookAround()

        while True:
            distance = self.sonar.get_distance()
            # print(distance)
            self.pf.measure(distance, 0)
            if self.visualize():
                break
            if distance > self.step_dist + 0.2:
                self.forward()
                if self.lookAround():
                    break
            else:
                self.go_to_angle(self.odometry.theta + math.pi / 2)
                if self.visualize():
                    break
        if self.sonar.get_distance() < 0.4:
            while True:
                self.go_to_angle(self.odometry.theta + math.pi / 2)
                if self.visualize() and self.sonar.get_distance() >= 0.4:
                    break
                # state = self.create.update()
                # self.odometry.update(state.leftEncoderCounts, state.rightEncoderCounts)
                print("Turn left!")

        print("Finished locolization!")

        print(self.odometry.x, self.odometry.y, self.odometry.theta)
        print(self.vc_x, self.vc_y, self.vc_theta)

        self.odometry.x = self.vc_x
        self.odometry.y = self.vc_y
        self.odometry.theta = self.vc_theta
        base_speed = 100

        self.vc_x *= 100
        self.vc_y = 300 - self.vc_y * 100
        print(self.vc_x, self.vc_y)
        self.rrt.build((self.vc_x, self.vc_y), 1500, 30)

        x_goal = self.rrt.nearest_neighbor((74.095, 160))


        path = self.rrt.shortest_path(x_goal)

        print(len(self.rrt.T))
        for v in self.rrt.T:
            # print(v.state)
            for u in v.neighbors:
                self.map2.draw_line((v.state[0], v.state[1]), (u.state[0], u.state[1]), (0,0,0))
        for idx in range(0, len(path)-1):
            self.map2.draw_line((path[idx].state[0], path[idx].state[1]), (path[idx+1].state[0], path[idx+1].state[1]), (0,255,0))

        self.map2.save("finalproject_img.png")

        # execute the path (essentially waypoint following from lab 6)

        # count = 0
        self.pf.set_param(0.02, 0.15, 0.1)
        while math.sqrt(math.pow(self.odometry.x - 0.74095, 2) + math.pow(self.odometry.y - 1.6, 2)) > 0.05:
            # print("Outer loop", math.sqrt(math.pow(self.odometry.x - 0.74095, 2) + math.pow(self.odometry.y - 1.6, 2)))
            for i in range(1, len(path)):
                # print("Inner loop", math.sqrt(math.pow(self.odometry.x - 0.74095, 2) + math.pow(self.odometry.y - 1.6, 2)))
                old_x = self.odometry.x
                old_y = self.odometry.y
                old_theta = self.odometry.theta
                if math.sqrt(math.pow(self.odometry.x - 0.74095, 2) + math.pow(self.odometry.y - 1.6, 2)) <= 0.05:
                    break
                elif math.sqrt(math.pow(self.odometry.x - 0.74095, 2) + math.pow(self.odometry.y - 1.6, 2)) <= 0.3:

                    goal_x = 0.74095
                    goal_y = 1.6
                    goal_theta = math.atan2(goal_y - self.odometry.y, goal_x - self.odometry.x)
                    self.go_to_angle(goal_theta)
                    while True:
                        state = self.create.update()
                        if state is not None:
                            self.odometry.update(state.leftEncoderCounts, state.rightEncoderCounts)
                            goal_theta = math.atan2(goal_y - self.odometry.y, goal_x - self.odometry.x)
                            output_theta = self.pidTheta.update(self.odometry.theta, goal_theta, self.time.time())
                            self.create.drive_direct(int(base_speed + output_theta), int(base_speed - output_theta))
                            distance = math.sqrt(
                                math.pow(goal_x - self.odometry.x, 2) + math.pow(goal_y - self.odometry.y, 2))
                            if distance <= 0.05:
                                break
                    self.pf.move_by(self.odometry.x - old_x, self.odometry.y - old_y, self.odometry.theta - old_theta)
                    self.visualize()
                    break
                # if i == len(path)-2:
                #     i += 1

                p = path[i]
                # print(p.state)
                goal_x = p.state[0] / 100.0
                goal_y = 3 - p.state[1] / 100.0
                # goal_theta = math.atan2(goal_y - self.odometry.y, goal_x - self.odometry.x)
                # self.go_to_angle(goal_theta)
                print("Goal", goal_x, goal_y)
                while True:
                    state = self.create.update()
                    if state is not None:
                        self.odometry.update(state.leftEncoderCounts, state.rightEncoderCounts)
                        goal_theta = math.atan2(goal_y - self.odometry.y, goal_x - self.odometry.x)
                        # theta = math.atan2(math.sin(self.odometry.theta), math.cos(self.odometry.theta))
                        output_theta = self.pidTheta.update(self.odometry.theta, goal_theta, self.time.time())
                        print("Odometry:", self.odometry.x, self.odometry.y, self.odometry.theta)
                        print("Goal:", goal_x, goal_y, goal_theta)
                        self.create.drive_direct(int(base_speed+output_theta), int(base_speed-output_theta))
                        # print("[{},{},{}]".format(self.odometry.x, self.odometry.y, math.degrees(self.odometry.theta)))

                        distance = math.sqrt(math.pow(goal_x - self.odometry.x, 2) + math.pow(goal_y - self.odometry.y, 2))
                        if i is not len(path)-1:
                            if distance <= 0.2:
                                break
                        else:
                            if distance <= 0.01:
                                break
                self.pf.move_by(self.odometry.x - old_x, self.odometry.y - old_y, self.odometry.theta - old_theta)
                self.visualize()



                # check rebuild the tree
                if i % 10 == 0:
                    self.create.drive_direct(0, 0)
                    if not self.lookAround():
                        while True:
                            distance = self.sonar.get_distance()
                            print(distance)
                            self.pf.measure(distance, 0)
                            if self.visualize():
                                break
                            if distance > self.step_dist + 0.2:
                                self.forward()
                                if self.lookAround():
                                    break
                            else:
                                self.go_to_angle(self.odometry.theta + math.pi / 2)
                                if self.visualize():
                                    break
                    if math.sqrt(math.pow(self.odometry.x - self.vc_x, 2) + math.pow(self.odometry.y - self.vc_y, 2)) > 0.1:
                        print("Drifted too much!")
                        self.odometry.x = self.vc_x
                        self.odometry.y = self.vc_y
                        self.odometry.theta = self.vc_theta
                        self.vc_x *= 100
                        self.vc_y = 300 - self.vc_y * 100
                        print(self.vc_x, self.vc_y)
                        self.map2 = lab11_map.Map("finalproject_map2_config.png")
                        self.rrt = rrt.RRT(self.map2)
                        self.rrt.build((self.vc_x, self.vc_y), 1500, 20)

                        print("New map built")

                        x_goal = self.rrt.nearest_neighbor((74.095, 160))

                        path = self.rrt.shortest_path(x_goal)
                        for v in self.rrt.T:
                            # print(v.state)
                            for u in v.neighbors:
                                self.map2.draw_line((v.state[0], v.state[1]), (u.state[0], u.state[1]), (0, 0, 0))
                        for idx in range(0, len(path) - 1):
                            self.map2.draw_line((path[idx].state[0], path[idx].state[1]),
                                                (path[idx + 1].state[0], path[idx + 1].state[1]), (0, 255, 0))

                        self.map2.save("finalproject_img.png")
                        break


        self.create.drive_direct(0, 0)
        groundTruth = self.create.sim_get_position()
        print("{},{},{},{},{}".format(self.odometry.x, self.odometry.y, self.odometry.theta * 180 / math.pi,
                                      groundTruth[0], groundTruth[1]))
        print("Error", self.odometry.x-0.74095, self.odometry.y-1.6, math.sqrt(math.pow(self.odometry.x - 0.74095, 2) + math.pow(self.odometry.y - 1.6, 2)))

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
            self.arm.go_to(1, ((160 - x) / 100))
            self.time.sleep(1)
            self.arm.go_to(3, (20 - x) / 133)
            self.time.sleep(0)
        for y in range(0, 30):
            self.arm.go_to(0, (-3.1415 * (y / 30)) / 2)
            self.time.sleep(1)

            # go to shelf 0
            for x in range(140, 160):
                self.arm.go_to(1, (x / 100))
                self.time.sleep(1)
            for y in range(130, 135):
                self.arm.go_to(3, ((130 - y) / 100))
                self.time.sleep(1)
            for c in range(157, 210):
                self.arm.go_to(0, -c / 100)
                self.time.sleep(1)
            self.arm.open_gripper()
            self.time.sleep(5)
            self.arm.go_to(1, 1.6)
            self.time.sleep(1)
            self.arm.go_to(0, -3.1415 / 2)
            self.time.sleep(1)
            self.arm.go_to(1, 0)
            self.arm.go_to(3, 0)
            self.time.sleep(1)

        # go to shelf 1
        for x in range(0, 35):
            self.arm.go_to(1, ((140 - x) / 100))
            self.time.sleep(1)
            self.arm.go_to(3, x / 100)
            self.time.sleep(1)
        for c in range(157, 230):
            self.arm.go_to(0, -c / 100)
            self.time.sleep(1)
        self.arm.open_gripper()
        self.time.sleep(5)
        self.arm.go_to(1, .8)
        self.time.sleep(1)
        self.arm.go_to(0, -3.1415 / 2)
        self.time.sleep(1)
        self.arm.go_to(1, 0)
        self.arm.go_to(3, 0)
        self.time.sleep(1)

        # go to shelf 2
        for x in range(0, 60):
            self.arm.go_to(1, ((140 - x) / 100))
            self.time.sleep(1)
        for y in range(0, 30):
            self.arm.go_to(3, y / 100)
            self.time.sleep(1)
        for c in range(157, 230):
            self.arm.go_to(0, -c / 100)
            self.time.sleep(1)
        self.arm.open_gripper()
        self.time.sleep(5)
        self.arm.go_to(1, .6)
        self.time.sleep(1)
        self.arm.go_to(0, -3.1415 / 2)
        self.time.sleep(1)
        self.arm.go_to(1, 0)
        self.arm.go_to(3, 0)
        self.time.sleep(1)

        print("finished!")
        time.sleep(10)
