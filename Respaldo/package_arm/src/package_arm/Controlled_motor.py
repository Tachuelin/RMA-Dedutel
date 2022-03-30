class MotorDriver:
    def __init__(self, max_speed=10): #Init communication, set default settings
        self.max_speed = max_speed
        self.current_speed = 0
        self.voltaje = 12
        self.temperature = 47

    def set_speed(self, speed): #Give a speed that the motor will try to reach
        if speed <=self.max_speed:
            self.current_speed = speed
        else:
            self.current_speed = self.max_speed

    def stop(self): #set speed to 0 and thus stop the motor
        self.current_speed = 0

    def get_speed(self): #Return current speed
        return self.current_speed

    def get_status(self): #Get hardware information from the motor
        return{'temperature':self.temperature, 'voltage': self.voltage}



        