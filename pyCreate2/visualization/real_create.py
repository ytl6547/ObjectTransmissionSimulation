"""
Module to control a virtual create
"""

from vrep import vrep as vrep
from enum import Enum


class RealCreate:
    """
    Class to control a virtual create in V-REP.
    """

    def __init__(self, hostname):
        """Constructor.

        Args:
            hostname (string): IP address for host running the simulation.
        """
        vrep.simxFinish(-1)

        self._clientID = vrep.simxStart(hostname, 19997, True, False, 5000, 5)  # Connect to V-REP
        print("clientId:", self._clientID)

        # query objects
        rc, self._obj = vrep.simxGetObjectHandle(self._clientID, "create_estimate", vrep.simx_opmode_oneshot_wait)
        print("Return code 1:", rc)
        # Use custom GUI
        rc, self._uiHandle = vrep.simxGetUIHandle(self._clientID, "UI", vrep.simx_opmode_oneshot_wait)
        print("Return code 2:", rc)

        vrep.simxGetUIEventButton(self._clientID, self._uiHandle, vrep.simx_opmode_streaming)
        print("RealCreate uiHandle:", self._uiHandle)

    def set_pose(self, position, yaw):
        vrep.simxSetObjectPosition(self._clientID, self._obj, -1, position,
                                   vrep.simx_opmode_oneshot_wait)
        vrep.simxSetObjectOrientation(self._clientID, self._obj, -1, (0, 0, yaw),
                                      vrep.simx_opmode_oneshot_wait)

    def set_point_cloud(self, data):
        signal = vrep.simxPackFloats(data)
        vrep.simxWriteStringStream(self._clientID, "pointCloud", signal, vrep.simx_opmode_oneshot)

    class Button(Enum):
        MoveForward = 3
        TurnLeft = 4
        TurnRight = 5
        Sense = 6

    def get_last_button(self):
        self.enable_buttons()
        err, button_id, aux = vrep.simxGetUIEventButton(self._clientID, self._uiHandle, vrep.simx_opmode_buffer)
        if err == vrep.simx_return_ok and button_id != -1:
            self.disable_buttons()
            vrep.simxGetUIEventButton(self._clientID, self._uiHandle, vrep.simx_opmode_streaming)
            return self.Button(button_id)
        return None

    def disable_buttons(self):
        for i in range(3, 7):
            _, prop = vrep.simxGetUIButtonProperty(self._clientID, self._uiHandle, i, vrep.simx_opmode_oneshot)
            prop &= ~vrep.sim_buttonproperty_enabled
            vrep.simxSetUIButtonProperty(self._clientID, self._uiHandle, i, prop, vrep.simx_opmode_oneshot)

    def enable_buttons(self):
        for i in range(3, 7):
            _, prop = vrep.simxGetUIButtonProperty(self._clientID, self._uiHandle, i, vrep.simx_opmode_oneshot)
            # print(prop)
            prop |= vrep.sim_buttonproperty_enabled
            vrep.simxSetUIButtonProperty(self._clientID, self._uiHandle, i, prop, vrep.simx_opmode_oneshot)
