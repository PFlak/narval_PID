class PID:
    def __init__(self, type_: type = float):
        self.__Kp               = type_()   # Proportional gain constant
        self.__Ki               = type_()   # Integral gain constant
        self.__Kd               = type_()   # Derivative gain constant
        self.__Kaw              = type_()   # Anti-windup gain constant
        self.T_C                = .01       # Time constant for derivative filtering
        self.T                  = .0        # Time step
        self.__max              = type_()   # Max command
        self.__min              = type_()   # Min command
        self.__max_rate         = 0         # Max rate of change of the command
        self.__integral         = type_()   # Integral term
        self.__err_prev         = type_()   # Previous error
        self.__deriv_prev       = type_()   # Previous derivative
        self.__command_sat_prev = type_()   # Previous saturated command
        self.__command_prev     = type_()   # Previous command
        self.__setpoint         = type_()   



    @property
    def Kp(self):
        return self.__Kp    
    
    @Kp.setter
    def Kp(self, kp):
        self.__Kp = kp

    @property
    def Ki(self):
        return self.__Ki
    
    @Ki.setter
    def Ki(self, ki):
        self.__Ki = ki

    @property
    def Kd(self):
        return self.__Kd

    @Kd.setter
    def Kd(self, kd):
        self.__Kd = kd

    @property
    def Kaw(self):
        return self.__Kaw

    @Kaw.setter
    def Kaw(self, kaw):
        self.__Kaw = kaw

    @property
    def max(self):
        return self.__max

    @max.setter
    def max(self, max_val):
        self.__max = max_val

    @property
    def min(self):
        return self.__min

    @min.setter
    def min(self, min_val):
        self.__min = min_val

    @property
    def max_rate(self):
        return self.__max_rate

    @max_rate.setter
    def max_rate(self, max_rate_val):
        self.__max_rate = max_rate_val

    @property
    def integral(self):
        return self.__integral

    @integral.setter
    def integral(self, integral_val):
        self.__integral = integral_val

    @property
    def err_prev(self):
        return self.__err_prev

    @err_prev.setter
    def err_prev(self, err_prev_val):
        self.__err_prev = err_prev_val

    @property
    def deriv_prev(self):
        return self.__deriv_prev

    @deriv_prev.setter
    def deriv_prev(self, deriv_prev_val):
        self.__deriv_prev = deriv_prev_val

    @property
    def command_sat_prev(self):
        return self.__command_sat_prev

    @command_sat_prev.setter
    def command_sat_prev(self, command_sat_prev_val):
        self.__command_sat_prev = command_sat_prev_val

    @property
    def command_prev(self):
        return self.__command_prev

    @command_prev.setter
    def command_prev(self, command_prev_val):
        self.__command_prev = command_prev_val

    @property
    def setpoint(self):
        return self.__setpoint

    @setpoint.setter
    def setpoint(self, setpoint_val):
        self.__setpoint = setpoint_val

    def step(self, measurement):

        # Error calculation
        err = self.setpoint - measurement
        
        # Integral term calculation - including anti-windup
        self.integral += self.Ki * err * self.T + self.Kaw * (self.command_sat_prev - self.command_prev) * self.T

        # Derivative term calculation using filtered derivative method
        deriv_filt = (err - self.err_prev + self.T_C * self.deriv_prev) / (self.T + self.T_C)

        self.err_prev = err
        self.deriv_prev = deriv_filt

        # Summing the 3 terms
        command = self.Kp * err + self.integral + self.Kd*deriv_filt

        if command > self.max:
            command_sat = self.max
        elif command < self.min:
            command_sat = self.min
        else:
            command_sat = command
        
        # Apply rate limiter
        if command_sat > (self.command_sat_prev + self.max_rate * self.T):
            command_sat = self.command_sat_prev + self.max_rate*self.T
        elif command_sat < (self.command_sat_prev - self.max_rate * self.T):
            command_sat = self.command_sat_prev - self.max_rate * self.T

        self.command_sat_prev = command_sat

        return command_sat, err
