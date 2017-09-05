
GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, *args, **kwargs):
        # TODO: Implement
        # PID controller(s)
        _kp_steer = args[0]
        _ki_steer = args[1]
        _kd_steer = args[2]

        pid_steer = PID(_kp_steer, _ki_steer, _kd_steer)
        pass

    def control(self, *args, **kwargs):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        return 1., 0., 0.
