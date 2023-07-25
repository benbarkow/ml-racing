# generated from rosidl_generator_py/resource/_idl.py.em
# with input from interfaces:srv/TrackingData.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_TrackingData_Request(type):
    """Metaclass of message 'TrackingData_Request'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'interfaces.srv.TrackingData_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__tracking_data__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__tracking_data__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__tracking_data__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__tracking_data__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__tracking_data__request

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class TrackingData_Request(metaclass=Metaclass_TrackingData_Request):
    """Message class 'TrackingData_Request'."""

    __slots__ = [
        '_x',
    ]

    _fields_and_field_types = {
        'x': 'string',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.x = kwargs.get('x', str())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.x != other.x:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def x(self):
        """Message field 'x'."""
        return self._x

    @x.setter
    def x(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'x' field must be of type 'str'"
        self._x = value


# Import statements for member types

# Member 'positioncar'
import array  # noqa: E402, I100

# already imported above
# import builtins

import math  # noqa: E402, I100

# already imported above
# import rosidl_parser.definition


class Metaclass_TrackingData_Response(type):
    """Metaclass of message 'TrackingData_Response'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'interfaces.srv.TrackingData_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__tracking_data__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__tracking_data__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__tracking_data__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__tracking_data__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__tracking_data__response

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class TrackingData_Response(metaclass=Metaclass_TrackingData_Response):
    """Message class 'TrackingData_Response'."""

    __slots__ = [
        '_speedx',
        '_speedy',
        '_positioncar',
    ]

    _fields_and_field_types = {
        'speedx': 'float',
        'speedy': 'float',
        'positioncar': 'sequence<float>',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('float')),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.speedx = kwargs.get('speedx', float())
        self.speedy = kwargs.get('speedy', float())
        self.positioncar = array.array('f', kwargs.get('positioncar', []))

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.speedx != other.speedx:
            return False
        if self.speedy != other.speedy:
            return False
        if self.positioncar != other.positioncar:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def speedx(self):
        """Message field 'speedx'."""
        return self._speedx

    @speedx.setter
    def speedx(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'speedx' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'speedx' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._speedx = value

    @builtins.property
    def speedy(self):
        """Message field 'speedy'."""
        return self._speedy

    @speedy.setter
    def speedy(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'speedy' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'speedy' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._speedy = value

    @builtins.property
    def positioncar(self):
        """Message field 'positioncar'."""
        return self._positioncar

    @positioncar.setter
    def positioncar(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'f', \
                "The 'positioncar' array.array() must have the type code of 'f'"
            self._positioncar = value
            return
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -3.402823466e+38 or val > 3.402823466e+38) or math.isinf(val) for val in value)), \
                "The 'positioncar' field must be a set or sequence and each value of type 'float' and each float in [-340282346600000016151267322115014000640.000000, 340282346600000016151267322115014000640.000000]"
        self._positioncar = array.array('f', value)


class Metaclass_TrackingData(type):
    """Metaclass of service 'TrackingData'."""

    _TYPE_SUPPORT = None

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'interfaces.srv.TrackingData')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__tracking_data

            from interfaces.srv import _tracking_data
            if _tracking_data.Metaclass_TrackingData_Request._TYPE_SUPPORT is None:
                _tracking_data.Metaclass_TrackingData_Request.__import_type_support__()
            if _tracking_data.Metaclass_TrackingData_Response._TYPE_SUPPORT is None:
                _tracking_data.Metaclass_TrackingData_Response.__import_type_support__()


class TrackingData(metaclass=Metaclass_TrackingData):
    from interfaces.srv._tracking_data import TrackingData_Request as Request
    from interfaces.srv._tracking_data import TrackingData_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
