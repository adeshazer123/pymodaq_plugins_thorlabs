# -*- coding: utf-8 -*-
"""
Created the 15/06/2023

@author: Sebastien Weber
"""

from pymodaq.control_modules.move_utility_classes import DAQ_Move_base, comon_parameters_fun, main, DataActuatorType, DataActuator  # common set of parameters for all actuators
from pymodaq.utils.daq_utils import ThreadCommand 
from pymodaq.utils.parameter import Parameter
from typing import Union, List, Dict
from pymodaq_plugins_thorlabs.hardware.elliptec import Elliptec

# from elliptec import Controller, Rotator
# from elliptec.scan import find_ports, scan_for_devices

# com_ports = find_ports()


class DAQ_Move_Elliptec(DAQ_Move_base):
    """Plugin for the Template Instrument

    This object inherits all functionality to communicate with PyMoDAQ Module through inheritance via DAQ_Move_base
    It then implements the particular communication with the instrument

    Attributes:
    -----------
    controller: object
        The particular object that allow the communication with the hardware, in general a python wrapper around the
         hardware library

    """
    is_multiaxes = True
    _axis_names: Union[List[str], Dict[str, int]] = {'1': 1, '2': 2, '3': 3, '4': 4, '5': 5, '6': 6, '7': 7, '8': 8, '9': 9, '10': 10, '11': 11, '12': 12, '13': 13, '14': 14, '15': 15, '16': 16, '17': 17}
    _controller_units: Union[str, List[str]] = ' '
    _epsilon: Union[float, List[float]] = 0.1

    params = [ {'title': 'COM port', 'name': 'com_port', 'type': 'str', 'value': 'COM12'},
              {'title': 'Device Type', 'name': 'device_type', 'type': 'str','readonly':True},
            # {'title': 'Units', 'name': 'units', 'type': 'str', 'readonly': True},
            #    {'title': 'Motor Type', 'name': 'motor', 'type': 'str'},
            #    {'title': 'Range', 'name': 'range', 'type': 'str'},
               ] + comon_parameters_fun(is_multiaxes, axis_names = _axis_names, epsilon=_epsilon)

    def ini_attributes(self):
        self.controller: Elliptec = None

    def get_actuator_value(self):
        """Get the current value from the hardware with scaling conversion.

        Returns
        -------
        float: The position obtained after scaling conversion.
        """
        pos = DataActuator(data = self.controller.get_position(self.axis_value), 
        units = self.controller.get_units(self.axis_value)) #units = self.controller.get_units(self.axis_value) suggestion
        pos = self.get_position_with_scaling(pos)   
        return pos

    def close(self):
        """Terminate the communication protocol"""
        self.controller.close()

    def commit_settings(self, param: Parameter):
        """Apply the consequences of a change of value in the detector settings

        Parameters
        ----------
        param: Parameter
            A given parameter (within detector_settings) whose value has been changed by the user
        """
        # if param.name() == 'units': 
        #     self.axis_unit = self.controller.get_units(self.axis_value)
        if param.name() == 'axis':
            units = self.controller.get_units(self.axis_value)
            self.axis_unit = units            
        # if param.name() == 'range': 
        #     self.controller.set_range(param.value())
        # else:
        pass

    def ini_stage(self, controller=None):
        """Actuator communication initialization

        Parameters
        ----------
        controller: (object)
            custom object of a PyMoDAQ plugin (Slave case). None if only one actuator by controller (Master case)

        Returns
        -------
        info: str
        initialized: bool
            False if initialization failed otherwise True
        """
        self.ini_stage_init(slave_controller=controller)

        if self.is_master: 
            self.controller = Elliptec()
            self.controller.connect(self.settings['com_port'])
            units = self.controller.get_units(self.axis_value)
            self.axis_unit = units

        """
        info = {'Address': addr,
                'Motor Type': int(msg[3:5], 16),
                'Serial No.': msg[5:13],
                'Year': msg[13:17],
                'Firmware': msg[17:19],
                'Thread': is_metric(msg[19]),
                'Hardware': msg[20],
                'Range': (int(msg[21:25], 16)),
                'Pulse/Rev': (int(msg[25:], 16))}
        """
        info = "Elliptec actuator initialized"
        initialized = True
        return info, initialized

    def move_abs(self, value: DataActuator):
        """ Move the actuator to the absolute target defined by value

        Parameters
        ----------
        value: (float) value of the absolute target positioning
        """

        value = self.check_bound(value)  
        self.target_value = value
        value = self.set_position_with_scaling(value)

        self.controller.move_abs(self.axis_value, value)

    def move_rel(self, value: DataActuator):
        """ Move the actuator to the relative target actuator value defined by value

        Parameters
        ----------
        value: (float) value of the relative target positioning
        """
        value = self.check_bound(self.current_position + value) - self.current_position
        self.target_value = value + self.current_position
        value = self.set_position_relative_with_scaling(value)

        self.controller.move_rel(self.axis_value, value) 

    def move_home(self):
        """Call the reference method of the controller"""
        self.controller.home(self.axis_value)  
        self.emit_status(ThreadCommand('Update_Status', ['Some info you want to log']))

    def stop_motion(self):
        """Stop the actuator and emits move_done signal"""
        pass


if __name__ == '__main__':
    main(__file__, init=False)


