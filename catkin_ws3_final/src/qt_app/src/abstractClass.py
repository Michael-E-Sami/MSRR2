from abc import ABCMeta,abstractmethod
class abstractClass(metaclass=ABCMeta):

    @abstractmethod
    def rotateLeft(self):
        pass

    @abstractmethod
    def stopRot(self):
        pass

    @abstractmethod
    def rotateRight(self):
        pass

    @abstractmethod
    def moveForward(self):
        pass

    @abstractmethod
    def stopFwdBwd(self):
        pass

    @abstractmethod
    def moveBackward(self):
        pass

    @abstractmethod
    def pitchUp(self):
        pass

    @abstractmethod
    def stopPitch(self):
        pass

    @abstractmethod
    def pitchDown(self):
        pass

    @abstractmethod
    def rollLeft(self):
        pass

    @abstractmethod
    def stopRoll(self):
        pass

    @abstractmethod
    def rollRight(self):
        pass


    @abstractmethod
    def getPointX(self):
        pass

    @abstractmethod
    def getPointY(self):
        pass

    @abstractmethod
    def getRotationYaw(self):
        pass

    