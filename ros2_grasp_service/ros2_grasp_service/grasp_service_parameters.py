# auto-generated DO NOT EDIT

from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.msg import SetParametersResult
from rcl_interfaces.msg import FloatingPointRange, IntegerRange
from rclpy.clock import Clock
from rclpy.exceptions import InvalidParameterValueException
from rclpy.time import Time
import copy
import rclpy
from generate_parameter_library_py.python_validators import ParameterValidators


class grasp_service_parameters:
    class Params:
        # for detecting if the parameter struct has been updated
        stamp_ = Time()

        joints_names = None
        positions_to_target_list = None
        positions_to_home_list = None
        time_to_wait_on_target = 0
        joints_controller_name = None
        gripper_controller_name = None
        gripper_close = None
        gripper_open = None

        class __PositionsToTarget:
            class __MapPositionsToTargetList:
                positions = None
                time_to_target = None

            __map_type = __MapPositionsToTargetList

            def add_entry(self, name):
                if not hasattr(self, name):
                    setattr(self, name, self.__map_type())

            def get_entry(self, name):
                return getattr(self, name)

        positions_to_target = __PositionsToTarget()

        class __PositionsToHome:
            class __MapPositionsToHomeList:
                positions = None
                time_to_target = None

            __map_type = __MapPositionsToHomeList

            def add_entry(self, name):
                if not hasattr(self, name):
                    setattr(self, name, self.__map_type())

            def get_entry(self, name):
                return getattr(self, name)

        positions_to_home = __PositionsToHome()

    class ParamListener:
        def __init__(self, node, prefix=""):
            self.prefix_ = prefix
            self.params_ = grasp_service_parameters.Params()
            self.node_ = node
            self.logger_ = rclpy.logging.get_logger("grasp_service_parameters." + prefix)

            self.declare_params()

            self.node_.add_on_set_parameters_callback(self.update)
            self.clock_ = Clock()

        def get_params(self):
            tmp = self.params_.stamp_
            self.params_.stamp_ = None
            paramCopy = copy.deepcopy(self.params_)
            paramCopy.stamp_ = tmp
            self.params_.stamp_ = tmp
            return paramCopy

        def is_old(self, other_param):
            return self.params_.stamp_ != other_param.stamp_

        def refresh_dynamic_parameters(self):
            updated_params = self.get_params()
            # TODO remove any destroyed dynamic parameters

            # declare any new dynamic parameters
            for value in updated_params.positions_to_target_list:
                updated_params.positions_to_target.add_entry(value)
                entry = updated_params.positions_to_target.get_entry(value)
                param_name = f"{self.prefix_}positions_to_target.{value}.positions"
                if not self.node_.has_parameter(self.prefix_ + param_name):
                    descriptor = ParameterDescriptor(
                        description="List of joints value for given position.", read_only=False
                    )
                    parameter = rclpy.Parameter.Type.DOUBLE_ARRAY
                    self.node_.declare_parameter(param_name, parameter, descriptor)
                param = self.node_.get_parameter(param_name)
                self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
                validation_result = ParameterValidators.size_gt(param, 0)
                if validation_result:
                    raise InvalidParameterValueException(
                        "positions_to_target.__map_positions_to_target_list.positions",
                        param.value,
                        "Invalid value set during initialization for parameter"
                        " positions_to_target.__map_positions_to_target_list.positions: "
                        + validation_result,
                    )
                entry.positions = param.value
            for value in updated_params.positions_to_target_list:
                updated_params.positions_to_target.add_entry(value)
                entry = updated_params.positions_to_target.get_entry(value)
                param_name = f"{self.prefix_}positions_to_target.{value}.time_to_target"
                if not self.node_.has_parameter(self.prefix_ + param_name):
                    descriptor = ParameterDescriptor(description="Time for given position", read_only=False)
                    descriptor.integer_range.append(IntegerRange())
                    descriptor.integer_range[-1].from_value = 0
                    descriptor.integer_range[-1].to_value = 2**31 - 1
                    parameter = rclpy.Parameter.Type.INTEGER
                    self.node_.declare_parameter(param_name, parameter, descriptor)
                param = self.node_.get_parameter(param_name)
                self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
                validation_result = ParameterValidators.gt(param, 0)
                if validation_result:
                    raise InvalidParameterValueException(
                        "positions_to_target.__map_positions_to_target_list.time_to_target",
                        param.value,
                        "Invalid value set during initialization for parameter"
                        " positions_to_target.__map_positions_to_target_list.time_to_target: "
                        + validation_result,
                    )
                entry.time_to_target = param.value
            for value in updated_params.positions_to_home_list:
                updated_params.positions_to_home.add_entry(value)
                entry = updated_params.positions_to_home.get_entry(value)
                param_name = f"{self.prefix_}positions_to_home.{value}.positions"
                if not self.node_.has_parameter(self.prefix_ + param_name):
                    descriptor = ParameterDescriptor(
                        description="List of joints value for given position.", read_only=False
                    )
                    parameter = rclpy.Parameter.Type.DOUBLE_ARRAY
                    self.node_.declare_parameter(param_name, parameter, descriptor)
                param = self.node_.get_parameter(param_name)
                self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
                validation_result = ParameterValidators.size_gt(param, 0)
                if validation_result:
                    raise InvalidParameterValueException(
                        "positions_to_home.__map_positions_to_home_list.positions",
                        param.value,
                        "Invalid value set during initialization for parameter"
                        " positions_to_home.__map_positions_to_home_list.positions: "
                        + validation_result,
                    )
                entry.positions = param.value
            for value in updated_params.positions_to_home_list:
                updated_params.positions_to_home.add_entry(value)
                entry = updated_params.positions_to_home.get_entry(value)
                param_name = f"{self.prefix_}positions_to_home.{value}.time_to_target"
                if not self.node_.has_parameter(self.prefix_ + param_name):
                    descriptor = ParameterDescriptor(description="Time for given position", read_only=False)
                    descriptor.integer_range.append(IntegerRange())
                    descriptor.integer_range[-1].from_value = 0
                    descriptor.integer_range[-1].to_value = 2**31 - 1
                    parameter = rclpy.Parameter.Type.INTEGER
                    self.node_.declare_parameter(param_name, parameter, descriptor)
                param = self.node_.get_parameter(param_name)
                self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
                validation_result = ParameterValidators.gt(param, 0)
                if validation_result:
                    raise InvalidParameterValueException(
                        "positions_to_home.__map_positions_to_home_list.time_to_target",
                        param.value,
                        "Invalid value set during initialization for parameter"
                        " positions_to_home.__map_positions_to_home_list.time_to_target: "
                        + validation_result,
                    )
                entry.time_to_target = param.value

        def update(self, parameters):
            updated_params = self.get_params()

            for param in parameters:
                if param.name == self.prefix_ + "joints_names":
                    validation_result = ParameterValidators.size_gt(param, 0)
                    if validation_result:
                        return SetParametersResult(successful=False, reason=validation_result)
                    validation_result = ParameterValidators.unique(param)
                    if validation_result:
                        return SetParametersResult(successful=False, reason=validation_result)
                    updated_params.joints_names = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "positions_to_target_list":
                    validation_result = ParameterValidators.size_gt(param, 0)
                    if validation_result:
                        return SetParametersResult(successful=False, reason=validation_result)
                    validation_result = ParameterValidators.unique(param)
                    if validation_result:
                        return SetParametersResult(successful=False, reason=validation_result)
                    updated_params.positions_to_target_list = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "positions_to_home_list":
                    validation_result = ParameterValidators.unique(param)
                    if validation_result:
                        return SetParametersResult(successful=False, reason=validation_result)
                    validation_result = ParameterValidators.size_gt(param, 0)
                    if validation_result:
                        return SetParametersResult(successful=False, reason=validation_result)
                    updated_params.positions_to_home_list = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "time_to_wait_on_target":
                    validation_result = ParameterValidators.gt(param, -1)
                    if validation_result:
                        return SetParametersResult(successful=False, reason=validation_result)
                    updated_params.time_to_wait_on_target = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "joints_controller_name":
                    validation_result = ParameterValidators.not_empty(param)
                    if validation_result:
                        return SetParametersResult(successful=False, reason=validation_result)
                    updated_params.joints_controller_name = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "gripper_controller_name":
                    validation_result = ParameterValidators.not_empty(param)
                    if validation_result:
                        return SetParametersResult(successful=False, reason=validation_result)
                    updated_params.gripper_controller_name = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "gripper_close":
                    updated_params.gripper_close = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "gripper_open":
                    updated_params.gripper_open = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

            # update dynamic parameters
            for param in parameters:
                for value in updated_params.positions_to_target_list:
                    param_name = f"{self.prefix_}positions_to_target.{value}.positions"
                    if param.name == param_name:
                        validation_result = ParameterValidators.size_gt(param, 0)
                        if validation_result:
                            return SetParametersResult(successful=False, reason=validation_result)
                        updated_params.positions_to_target.positions_to_target_list_map[value].positions = param.value
                        self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                for value in updated_params.positions_to_target_list:
                    param_name = f"{self.prefix_}positions_to_target.{value}.time_to_target"
                    if param.name == param_name:
                        validation_result = ParameterValidators.gt(param, 0)
                        if validation_result:
                            return SetParametersResult(successful=False, reason=validation_result)
                        updated_params.positions_to_target.positions_to_target_list_map[value].time_to_target = (
                            param.value
                        )
                        self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                for value in updated_params.positions_to_home_list:
                    param_name = f"{self.prefix_}positions_to_home.{value}.positions"
                    if param.name == param_name:
                        validation_result = ParameterValidators.size_gt(param, 0)
                        if validation_result:
                            return SetParametersResult(successful=False, reason=validation_result)
                        updated_params.positions_to_home.positions_to_home_list_map[value].positions = param.value
                        self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                for value in updated_params.positions_to_home_list:
                    param_name = f"{self.prefix_}positions_to_home.{value}.time_to_target"
                    if param.name == param_name:
                        validation_result = ParameterValidators.gt(param, 0)
                        if validation_result:
                            return SetParametersResult(successful=False, reason=validation_result)
                        updated_params.positions_to_home.positions_to_home_list_map[value].time_to_target = param.value
                        self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

            updated_params.stamp_ = self.clock_.now()
            self.update_internal_params(updated_params)
            return SetParametersResult(successful=True)

        def update_internal_params(self, updated_params):
            self.params_ = updated_params

        def declare_params(self):
            updated_params = self.get_params()
            # declare all parameters and give default values to non-required ones
            if not self.node_.has_parameter(self.prefix_ + "joints_names"):
                descriptor = ParameterDescriptor(
                    description="Names of joints controlled by joint trajectory controller.", read_only=False
                )
                parameter = rclpy.Parameter.Type.STRING_ARRAY
                self.node_.declare_parameter(self.prefix_ + "joints_names", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "positions_to_target_list"):
                descriptor = ParameterDescriptor(description="Names of positions to target.", read_only=False)
                parameter = rclpy.Parameter.Type.STRING_ARRAY
                self.node_.declare_parameter(self.prefix_ + "positions_to_target_list", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "positions_to_home_list"):
                descriptor = ParameterDescriptor(description="Names of positions to target.", read_only=False)
                parameter = rclpy.Parameter.Type.STRING_ARRAY
                self.node_.declare_parameter(self.prefix_ + "positions_to_home_list", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "time_to_wait_on_target"):
                descriptor = ParameterDescriptor(description="Time to wait on target position.", read_only=True)
                descriptor.integer_range.append(IntegerRange())
                descriptor.integer_range[-1].from_value = -1
                descriptor.integer_range[-1].to_value = 2**31 - 1
                parameter = updated_params.time_to_wait_on_target
                self.node_.declare_parameter(self.prefix_ + "time_to_wait_on_target", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "joints_controller_name"):
                descriptor = ParameterDescriptor(description="Name of joints trajectory controller", read_only=False)
                parameter = rclpy.Parameter.Type.STRING
                self.node_.declare_parameter(self.prefix_ + "joints_controller_name", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "gripper_controller_name"):
                descriptor = ParameterDescriptor(description="Name of gripper controller", read_only=False)
                parameter = rclpy.Parameter.Type.STRING
                self.node_.declare_parameter(self.prefix_ + "gripper_controller_name", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "gripper_close"):
                descriptor = ParameterDescriptor(description="Position of closed gripper", read_only=False)
                parameter = rclpy.Parameter.Type.DOUBLE
                self.node_.declare_parameter(self.prefix_ + "gripper_close", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "gripper_open"):
                descriptor = ParameterDescriptor(description="Position of open gripper", read_only=False)
                parameter = rclpy.Parameter.Type.DOUBLE
                self.node_.declare_parameter(self.prefix_ + "gripper_open", parameter, descriptor)

            # TODO: need validation
            # get parameters and fill struct fields
            param = self.node_.get_parameter(self.prefix_ + "joints_names")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            validation_result = ParameterValidators.size_gt(param, 0)
            if validation_result:
                raise InvalidParameterValueException(
                    "joints_names",
                    param.value,
                    "Invalid value set during initialization for parameter joints_names: " + validation_result,
                )
            validation_result = ParameterValidators.unique(param)
            if validation_result:
                raise InvalidParameterValueException(
                    "joints_names",
                    param.value,
                    "Invalid value set during initialization for parameter joints_names: " + validation_result,
                )
            updated_params.joints_names = param.value
            param = self.node_.get_parameter(self.prefix_ + "positions_to_target_list")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            validation_result = ParameterValidators.size_gt(param, 0)
            if validation_result:
                raise InvalidParameterValueException(
                    "positions_to_target_list",
                    param.value,
                    "Invalid value set during initialization for parameter positions_to_target_list: "
                    + validation_result,
                )
            validation_result = ParameterValidators.unique(param)
            if validation_result:
                raise InvalidParameterValueException(
                    "positions_to_target_list",
                    param.value,
                    "Invalid value set during initialization for parameter positions_to_target_list: "
                    + validation_result,
                )
            updated_params.positions_to_target_list = param.value
            param = self.node_.get_parameter(self.prefix_ + "positions_to_home_list")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            validation_result = ParameterValidators.unique(param)
            if validation_result:
                raise InvalidParameterValueException(
                    "positions_to_home_list",
                    param.value,
                    "Invalid value set during initialization for parameter positions_to_home_list: "
                    + validation_result,
                )
            validation_result = ParameterValidators.size_gt(param, 0)
            if validation_result:
                raise InvalidParameterValueException(
                    "positions_to_home_list",
                    param.value,
                    "Invalid value set during initialization for parameter positions_to_home_list: "
                    + validation_result,
                )
            updated_params.positions_to_home_list = param.value
            param = self.node_.get_parameter(self.prefix_ + "time_to_wait_on_target")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            validation_result = ParameterValidators.gt(param, -1)
            if validation_result:
                raise InvalidParameterValueException(
                    "time_to_wait_on_target",
                    param.value,
                    "Invalid value set during initialization for parameter time_to_wait_on_target: "
                    + validation_result,
                )
            updated_params.time_to_wait_on_target = param.value
            param = self.node_.get_parameter(self.prefix_ + "joints_controller_name")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            validation_result = ParameterValidators.not_empty(param)
            if validation_result:
                raise InvalidParameterValueException(
                    "joints_controller_name",
                    param.value,
                    "Invalid value set during initialization for parameter joints_controller_name: "
                    + validation_result,
                )
            updated_params.joints_controller_name = param.value
            param = self.node_.get_parameter(self.prefix_ + "gripper_controller_name")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            validation_result = ParameterValidators.not_empty(param)
            if validation_result:
                raise InvalidParameterValueException(
                    "gripper_controller_name",
                    param.value,
                    "Invalid value set during initialization for parameter gripper_controller_name: "
                    + validation_result,
                )
            updated_params.gripper_controller_name = param.value
            param = self.node_.get_parameter(self.prefix_ + "gripper_close")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.gripper_close = param.value
            param = self.node_.get_parameter(self.prefix_ + "gripper_open")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.gripper_open = param.value

            # declare and set all dynamic parameters
            for value in updated_params.positions_to_target_list:
                updated_params.positions_to_target.add_entry(value)
                entry = updated_params.positions_to_target.get_entry(value)
                param_name = f"{self.prefix_}positions_to_target.{value}.positions"
                if not self.node_.has_parameter(self.prefix_ + param_name):
                    descriptor = ParameterDescriptor(
                        description="List of joints value for given position.", read_only=False
                    )
                    parameter = rclpy.Parameter.Type.DOUBLE_ARRAY
                    self.node_.declare_parameter(param_name, parameter, descriptor)
                param = self.node_.get_parameter(param_name)
                self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
                validation_result = ParameterValidators.size_gt(param, 0)
                if validation_result:
                    raise InvalidParameterValueException(
                        "positions_to_target.__map_positions_to_target_list.positions",
                        param.value,
                        "Invalid value set during initialization for parameter"
                        " positions_to_target.__map_positions_to_target_list.positions: "
                        + validation_result,
                    )
                entry.positions = param.value
            for value in updated_params.positions_to_target_list:
                updated_params.positions_to_target.add_entry(value)
                entry = updated_params.positions_to_target.get_entry(value)
                param_name = f"{self.prefix_}positions_to_target.{value}.time_to_target"
                if not self.node_.has_parameter(self.prefix_ + param_name):
                    descriptor = ParameterDescriptor(description="Time for given position", read_only=False)
                    descriptor.integer_range.append(IntegerRange())
                    descriptor.integer_range[-1].from_value = 0
                    descriptor.integer_range[-1].to_value = 2**31 - 1
                    parameter = rclpy.Parameter.Type.INTEGER
                    self.node_.declare_parameter(param_name, parameter, descriptor)
                param = self.node_.get_parameter(param_name)
                self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
                validation_result = ParameterValidators.gt(param, 0)
                if validation_result:
                    raise InvalidParameterValueException(
                        "positions_to_target.__map_positions_to_target_list.time_to_target",
                        param.value,
                        "Invalid value set during initialization for parameter"
                        " positions_to_target.__map_positions_to_target_list.time_to_target: "
                        + validation_result,
                    )
                entry.time_to_target = param.value
            for value in updated_params.positions_to_home_list:
                updated_params.positions_to_home.add_entry(value)
                entry = updated_params.positions_to_home.get_entry(value)
                param_name = f"{self.prefix_}positions_to_home.{value}.positions"
                if not self.node_.has_parameter(self.prefix_ + param_name):
                    descriptor = ParameterDescriptor(
                        description="List of joints value for given position.", read_only=False
                    )
                    parameter = rclpy.Parameter.Type.DOUBLE_ARRAY
                    self.node_.declare_parameter(param_name, parameter, descriptor)
                param = self.node_.get_parameter(param_name)
                self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
                validation_result = ParameterValidators.size_gt(param, 0)
                if validation_result:
                    raise InvalidParameterValueException(
                        "positions_to_home.__map_positions_to_home_list.positions",
                        param.value,
                        "Invalid value set during initialization for parameter"
                        " positions_to_home.__map_positions_to_home_list.positions: "
                        + validation_result,
                    )
                entry.positions = param.value
            for value in updated_params.positions_to_home_list:
                updated_params.positions_to_home.add_entry(value)
                entry = updated_params.positions_to_home.get_entry(value)
                param_name = f"{self.prefix_}positions_to_home.{value}.time_to_target"
                if not self.node_.has_parameter(self.prefix_ + param_name):
                    descriptor = ParameterDescriptor(description="Time for given position", read_only=False)
                    descriptor.integer_range.append(IntegerRange())
                    descriptor.integer_range[-1].from_value = 0
                    descriptor.integer_range[-1].to_value = 2**31 - 1
                    parameter = rclpy.Parameter.Type.INTEGER
                    self.node_.declare_parameter(param_name, parameter, descriptor)
                param = self.node_.get_parameter(param_name)
                self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
                validation_result = ParameterValidators.gt(param, 0)
                if validation_result:
                    raise InvalidParameterValueException(
                        "positions_to_home.__map_positions_to_home_list.time_to_target",
                        param.value,
                        "Invalid value set during initialization for parameter"
                        " positions_to_home.__map_positions_to_home_list.time_to_target: "
                        + validation_result,
                    )
                entry.time_to_target = param.value

            self.update_internal_params(updated_params)
