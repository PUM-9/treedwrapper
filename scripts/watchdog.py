from threading import Timer

hardwareErrorOccur = False

"""
class HardwareError(Exception):
    def __init__(self, arg):
        self.msg = arg

    def __str__(self):
        return repr(self.msg)
"""

        
class Watchdog:
    def __init__(self, timeout, userHandler=None):
        hardwareErrorOccur = False
        self.timeout = timeout
        self.handler = userHandler if userHandler is not None else self.defaultHandler
        self.timer = Timer(self.timeout, self.handler)
        self.timer.start()

    def reset(self):
        self.timer.cancel()
        self.timer = Timer(self.timeout, self.handler)
        self.timer.start()

    def stop(self):
        self.timer.cancel()

    def defaultHandler(self):
        hardwareErrorOccur = True


