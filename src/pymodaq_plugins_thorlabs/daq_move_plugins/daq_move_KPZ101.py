# Must rename daq_move_kpz101 to daq_move_KPZ101. 
# Purpose: Control the KPZ101 piezo stage from Thorlabs with PyMoDAQ plugin

from pymodaq.control_modules.move_utility_classes import DAQ_Move_base, main, comon_parameters_fun
from pymodaq.utils.logger import set_logger, get_module_name

from pymodaq_plugins_thorlabs.hardware.kinesis import serialnumbers_piezo, Piezo


logger = set_logger(get_module_name(__file__))


class DAQ_Move_KPZ101(DAQ_Move_base):
    """
    Wrapper object to access Piezo functionalities, similar to Kinesis instruments 
    """
    _controller_units = 'V'
    _epsilon = 0.0

    is_multiaxes = False

    stage_names = []

    params = [{'title': 'Controller ID:', 'name': 'controller_id', 'type': 'str', 'value': '', 'readonly': True},
              {'title': 'Serial number:', 'name': 'serial_number', 'type': 'list',
               'limits': serialnumbers_piezo},
              {'title': 'Home Position:', 'name': 'home_position', 'type': 'float', 'value': 0.0, },
              {'title': 'Get Voltage', 'name': 'get_voltage', 'type': 'float', 'value': 0.0, 'readonly': True},
              ] + comon_parameters_fun(is_multiaxes, epsilon=_epsilon)

    def ini_attributes(self):
        self.controller: Piezo = None
        self.settings.child('bounds', 'is_bounds').setValue(True)
        self.settings.child('bounds', 'max_bound').setValue(360)
        self.settings.child('bounds', 'min_bound').setValue(0)

    def commit_settings(self, param):
       pass 

    def ini_stage(self, controller=None):
        """
        Connect to Kinesis Piezo Stage by communicating with kinesis.py
        """
        self.controller = self.ini_stage_init(controller, Piezo())

        if self.settings['multiaxes', 'multi_status'] == "Master":
            self.controller.connect(self.settings['serial_number'])

        info = self.controller.name
        self.settings.child('controller_id').setValue(info)

        #self.controller.backlash = self.settings['backlash']

        initialized = True
        return info, initialized

    def close(self):
        """
            close the current instance of Kinesis instrument.
        """
        if self.controller is not None:
            self.Disconnect()

    def stop_motion(self):
        """
            See Also
            --------
            DAQ_Move_base.move_done
        """
        if self.controller is not None:
            self.controller.stop()

    def get_actuator_value(self):
        """
            Get the current hardware position with scaling conversion of the Kinsesis instrument provided by get_position_with_scaling

            See Also
            --------
            DAQ_Move_base.get_position_with_scaling, daq_utils.ThreadCommand
        """
        pos = self.settings['get_voltage']
        return pos
        # pos = self.controller.get_voltage()
        # pos = self.get_position_with_scaling(pos) #TODO: Check if this converts voltage to position
        # return pos

    def move_abs(self, position):
        """
        Set the current position with voltage conversion of the Kinesis instrument 

        """
        
        position = self.check_bound(position)
        self.target_position = position
        position = self.set_position_with_scaling(position)

        self.controller.set_voltage(position) #TODO: Check if self.controller communicates with Piezo(Kinesis)

    def move_rel(self, position):
        """

        """
        position = self.check_bound(self.current_position + position) - self.current_position
        self.target_position = position + self.current_position
        position = self.set_position_relative_with_scaling(position)

        self.controller.move_rel(position)

    def move_home(self):
        """
        Move the Kinesis Piezo Stage to home position
        """
        home = self.settings['home_position']
        self.target_position = home
        self.controller.SetZero(home)


if __name__ == '__main__':
    main(__file__, init=False)