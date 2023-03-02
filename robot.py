import wpilib
import commands2
from robotContainer import RobotContainer
'''
TODO:
- test teleoperated control with PID's, add compatibility with macros
- Implement auto game piece placement macros
- Implement auto game piece pickup macros

What do you need to know to be able to understand the code:
- OOP
- Classes
- Class Extensions
- Polymorphism (you may ask what that is, don't even)


'''
class MyRobot(commands2.TimedCommandRobot):
    
    def __init__(self, period = 0.02) -> None:
        super().__init__(period)
        
    def robotInit(self):
        self.robotContainer = RobotContainer()
        self.autoCommand = commands2.Command()
        
    def disabledInit(self) -> None:
        ''''''
        return super().disabledInit()
            
    def testInit(self) -> None:
        return super().testInit()
    
    def testPeriodic(self) -> None:
        return super().testPeriodic()
    
    def autonomousInit(self):
        self.autoCommand = self.robotContainer.getAutonomousCommand()
        if (self.autoCommand != commands2.Command()):
            self.autoCommand.schedule()
        
    def autonomousPeriodic(self):
        pass
        
    def teleopInit(self):
        if (self.autoCommand != commands2.Command()):
            self.autoCommand.cancel()

if __name__ == "__main__":
    wpilib.run(MyRobot)