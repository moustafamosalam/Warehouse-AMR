# generated from rosidl_generator_py/resource/_idl.py.em
# with input from vizanti_msgs:srv/ListLifecycles.idl
# generated code does not contain a copyright notice


# Import statements for member types

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_ListLifecycles_Request(type):
    """Metaclass of message 'ListLifecycles_Request'."""

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
            module = import_type_support('vizanti_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'vizanti_msgs.srv.ListLifecycles_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__list_lifecycles__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__list_lifecycles__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__list_lifecycles__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__list_lifecycles__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__list_lifecycles__request

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class ListLifecycles_Request(metaclass=Metaclass_ListLifecycles_Request):
    """Message class 'ListLifecycles_Request'."""

    __slots__ = [
    ]

    _fields_and_field_types = {
    }

    SLOT_TYPES = (
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))

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
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)


# Import statements for member types

# Member 'states'
import array  # noqa: E402, I100

import builtins  # noqa: E402, I100

# already imported above
# import rosidl_parser.definition


class Metaclass_ListLifecycles_Response(type):
    """Metaclass of message 'ListLifecycles_Response'."""

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
            module = import_type_support('vizanti_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'vizanti_msgs.srv.ListLifecycles_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__list_lifecycles__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__list_lifecycles__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__list_lifecycles__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__list_lifecycles__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__list_lifecycles__response

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class ListLifecycles_Response(metaclass=Metaclass_ListLifecycles_Response):
    """Message class 'ListLifecycles_Response'."""

    __slots__ = [
        '_nodes',
        '_states',
    ]

    _fields_and_field_types = {
        'nodes': 'sequence<string>',
        'states': 'sequence<int8>',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.UnboundedString()),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('int8')),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.nodes = kwargs.get('nodes', [])
        self.states = array.array('b', kwargs.get('states', []))

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
        if self.nodes != other.nodes:
            return False
        if self.states != other.states:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def nodes(self):
        """Message field 'nodes'."""
        return self._nodes

    @nodes.setter
    def nodes(self, value):
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
                 all(isinstance(v, str) for v in value) and
                 True), \
                "The 'nodes' field must be a set or sequence and each value of type 'str'"
        self._nodes = value

    @builtins.property
    def states(self):
        """Message field 'states'."""
        return self._states

    @states.setter
    def states(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'b', \
                "The 'states' array.array() must have the type code of 'b'"
            self._states = value
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
                 all(isinstance(v, int) for v in value) and
                 all(val >= -128 and val < 128 for val in value)), \
                "The 'states' field must be a set or sequence and each value of type 'int' and each integer in [-128, 127]"
        self._states = array.array('b', value)


class Metaclass_ListLifecycles(type):
    """Metaclass of service 'ListLifecycles'."""

    _TYPE_SUPPORT = None

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('vizanti_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'vizanti_msgs.srv.ListLifecycles')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__list_lifecycles

            from vizanti_msgs.srv import _list_lifecycles
            if _list_lifecycles.Metaclass_ListLifecycles_Request._TYPE_SUPPORT is None:
                _list_lifecycles.Metaclass_ListLifecycles_Request.__import_type_support__()
            if _list_lifecycles.Metaclass_ListLifecycles_Response._TYPE_SUPPORT is None:
                _list_lifecycles.Metaclass_ListLifecycles_Response.__import_type_support__()


class ListLifecycles(metaclass=Metaclass_ListLifecycles):
    from vizanti_msgs.srv._list_lifecycles import ListLifecycles_Request as Request
    from vizanti_msgs.srv._list_lifecycles import ListLifecycles_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
