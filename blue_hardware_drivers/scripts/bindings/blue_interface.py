# This file was automatically generated by SWIG (http://www.swig.org).
# Version 3.0.8
#
# Do not make changes to this file unless you know what you are doing--modify
# the SWIG interface file instead.





from sys import version_info
if version_info >= (2, 6, 0):
    def swig_import_helper():
        from os.path import dirname
        import imp
        fp = None
        try:
            fp, pathname, description = imp.find_module('_blue_interface', [dirname(__file__)])
        except ImportError:
            import _blue_interface
            return _blue_interface
        if fp is not None:
            try:
                _mod = imp.load_module('_blue_interface', fp, pathname, description)
            finally:
                fp.close()
            return _mod
    _blue_interface = swig_import_helper()
    del swig_import_helper
else:
    import _blue_interface
del version_info
try:
    _swig_property = property
except NameError:
    pass  # Python < 2.2 doesn't have 'property'.


def _swig_setattr_nondynamic(self, class_type, name, value, static=1):
    if (name == "thisown"):
        return self.this.own(value)
    if (name == "this"):
        if type(value).__name__ == 'SwigPyObject':
            self.__dict__[name] = value
            return
    method = class_type.__swig_setmethods__.get(name, None)
    if method:
        return method(self, value)
    if (not static):
        if _newclass:
            object.__setattr__(self, name, value)
        else:
            self.__dict__[name] = value
    else:
        raise AttributeError("You cannot add attributes to %s" % self)


def _swig_setattr(self, class_type, name, value):
    return _swig_setattr_nondynamic(self, class_type, name, value, 0)


def _swig_getattr_nondynamic(self, class_type, name, static=1):
    if (name == "thisown"):
        return self.this.own()
    method = class_type.__swig_getmethods__.get(name, None)
    if method:
        return method(self)
    if (not static):
        return object.__getattr__(self, name)
    else:
        raise AttributeError(name)

def _swig_getattr(self, class_type, name):
    return _swig_getattr_nondynamic(self, class_type, name, 0)


def _swig_repr(self):
    try:
        strthis = "proxy of " + self.this.__repr__()
    except Exception:
        strthis = ""
    return "<%s.%s; %s >" % (self.__class__.__module__, self.__class__.__name__, strthis,)

try:
    _object = object
    _newclass = 1
except AttributeError:
    class _object:
        pass
    _newclass = 0


class BLDCControllerClient(_object):
    __swig_setmethods__ = {}
    __setattr__ = lambda self, name, value: _swig_setattr(self, BLDCControllerClient, name, value)
    __swig_getmethods__ = {}
    __getattr__ = lambda self, name: _swig_getattr(self, BLDCControllerClient, name)
    __repr__ = _swig_repr

    def __init__(self, *args):
        this = _blue_interface.new_BLDCControllerClient(*args)
        try:
            self.this.append(this)
        except Exception:
            self.this = this

    def init(self, port, boards):
        return _blue_interface.BLDCControllerClient_init(self, port, boards)

    def queuePacket(self, server_id, packet):
        return _blue_interface.BLDCControllerClient_queuePacket(self, server_id, packet)

    def queueLeaveBootloader(self, server_id, jump_addr):
        return _blue_interface.BLDCControllerClient_queueLeaveBootloader(self, server_id, jump_addr)

    def queueSetTimeout(self, server_id, value):
        return _blue_interface.BLDCControllerClient_queueSetTimeout(self, server_id, value)

    def queueSetControlMode(self, server_id, control_mode):
        return _blue_interface.BLDCControllerClient_queueSetControlMode(self, server_id, control_mode)

    def queueSetZeroAngle(self, server_id, value):
        return _blue_interface.BLDCControllerClient_queueSetZeroAngle(self, server_id, value)

    def queueSetERevsPerMRev(self, server_id, value):
        return _blue_interface.BLDCControllerClient_queueSetERevsPerMRev(self, server_id, value)

    def queueSetInvertPhases(self, server_id, value):
        return _blue_interface.BLDCControllerClient_queueSetInvertPhases(self, server_id, value)

    def queueSetTorqueConstant(self, server_id, value):
        return _blue_interface.BLDCControllerClient_queueSetTorqueConstant(self, server_id, value)

    def queueSetPositionOffset(self, server_id, value):
        return _blue_interface.BLDCControllerClient_queueSetPositionOffset(self, server_id, value)

    def queueSetEACScale(self, server_id, value):
        return _blue_interface.BLDCControllerClient_queueSetEACScale(self, server_id, value)

    def queueSetEACOffset(self, server_id, value):
        return _blue_interface.BLDCControllerClient_queueSetEACOffset(self, server_id, value)

    def queueSetEACTable(self, server_id, start_index, values, count):
        return _blue_interface.BLDCControllerClient_queueSetEACTable(self, server_id, start_index, values, count)

    def queueSetDirectCurrentControllerKp(self, server_id, value):
        return _blue_interface.BLDCControllerClient_queueSetDirectCurrentControllerKp(self, server_id, value)

    def queueSetDirectCurrentControllerKi(self, server_id, value):
        return _blue_interface.BLDCControllerClient_queueSetDirectCurrentControllerKi(self, server_id, value)

    def queueSetQuadratureCurrentControllerKp(self, server_id, value):
        return _blue_interface.BLDCControllerClient_queueSetQuadratureCurrentControllerKp(self, server_id, value)

    def queueSetQuadratureCurrentControllerKi(self, server_id, value):
        return _blue_interface.BLDCControllerClient_queueSetQuadratureCurrentControllerKi(self, server_id, value)

    def queueSetVelocityControllerKp(self, server_id, value):
        return _blue_interface.BLDCControllerClient_queueSetVelocityControllerKp(self, server_id, value)

    def queueSetVelocityControllerKi(self, server_id, value):
        return _blue_interface.BLDCControllerClient_queueSetVelocityControllerKi(self, server_id, value)

    def queueSetPositionControllerKp(self, server_id, value):
        return _blue_interface.BLDCControllerClient_queueSetPositionControllerKp(self, server_id, value)

    def queueSetPositionControllerKi(self, server_id, value):
        return _blue_interface.BLDCControllerClient_queueSetPositionControllerKi(self, server_id, value)

    def queueSetCommand(self, server_id, value):
        return _blue_interface.BLDCControllerClient_queueSetCommand(self, server_id, value)

    def queueGetRotorPosition(self, server_id):
        return _blue_interface.BLDCControllerClient_queueGetRotorPosition(self, server_id)

    def queueSetCommandAndGetRotorPosition(self, server_id, value):
        return _blue_interface.BLDCControllerClient_queueSetCommandAndGetRotorPosition(self, server_id, value)

    def queueSetPositionAndGetRotorPosition(self, server_id, value):
        return _blue_interface.BLDCControllerClient_queueSetPositionAndGetRotorPosition(self, server_id, value)

    def queueGetState(self, server_id):
        return _blue_interface.BLDCControllerClient_queueGetState(self, server_id)

    def queueSetCommandAndGetState(self, server_id, value):
        return _blue_interface.BLDCControllerClient_queueSetCommandAndGetState(self, server_id, value)

    def exchange(self):
        return _blue_interface.BLDCControllerClient_exchange(self)

    def clearQueue(self):
        return _blue_interface.BLDCControllerClient_clearQueue(self)

    def resultGetRotorPosition(self, server_id, result):
        return _blue_interface.BLDCControllerClient_resultGetRotorPosition(self, server_id, result)

    def resultGetState(self, server_id, position, velocity, di, qi, voltage, temp, acc_x, acc_y, acc_z):
        return _blue_interface.BLDCControllerClient_resultGetState(self, server_id, position, velocity, di, qi, voltage, temp, acc_x, acc_y, acc_z)

    def initMotor(self, server_id):
        return _blue_interface.BLDCControllerClient_initMotor(self, server_id)
    __swig_destroy__ = _blue_interface.delete_BLDCControllerClient
    __del__ = lambda self: None
BLDCControllerClient_swigregister = _blue_interface.BLDCControllerClient_swigregister
BLDCControllerClient_swigregister(BLDCControllerClient)

class comms_error(_object):
    __swig_setmethods__ = {}
    __setattr__ = lambda self, name, value: _swig_setattr(self, comms_error, name, value)
    __swig_getmethods__ = {}
    __getattr__ = lambda self, name: _swig_getattr(self, comms_error, name)
    __repr__ = _swig_repr

    def __init__(self, msg):
        this = _blue_interface.new_comms_error(msg)
        try:
            self.this.append(this)
        except Exception:
            self.this = this

    def what(self):
        return _blue_interface.comms_error_what(self)
    __swig_destroy__ = _blue_interface.delete_comms_error
    __del__ = lambda self: None
comms_error_swigregister = _blue_interface.comms_error_swigregister
comms_error_swigregister(comms_error)

# This file is compatible with both classic and new-style classes.

