import rev
from wpimath import controller, kinematics, geometry
from wpilib import AnalogEncoder, SmartDashboard
import math

class SwerveModule:
    steeringRatio = 1
    drivingRatio = 1
    turnTolerance = 0.5 # degrees
    maxWheelVelocity = 4 # meters per second
    maxVoltage = 12.0
    
    def __init__(self, config: dict, name: str) -> None:
        self.config = config
        self.moduleName = name
        
        self.driveMotor = rev.CANSparkMax(self.config["driveMotorID"], rev.CANSparkMax.MotorType.kBrushless)
        self.driveMotor.enableVoltageCompensation(12.0)
        self.driveMotor.setIdleMode(rev.CANSparkMax.IdleMode.kCoast)
        self.driveEncoder = self.driveMotor.getEncoder(countsPerRev = 1)
        
        self.turnMotor = rev.CANSparkMax(self.config["turnMotorID"], rev.CANSparkMax.MotorType.kBrushless)
        self.turnMotor.enableVoltageCompensation(12.0)
        self.turnMotor.setIdleMode(rev.CANSparkMax.IdleMode.kCoast)
        self.turnEncoder = self.turnMotor.getEncoder(countsPerRev = 360)
        
        self.absoluteEncoder = AnalogEncoder(self.config["encoderID"])
        self.absoluteOffset = self.config["encoderOffset"]
        
        turnkP = self.maxVoltage / 90 # angle will at most be off by 90 degrees so we will set 12V there
        turnkI = 0
        turnkD = 0
        turnPeriod = 0.02
        
        self.turnPID = controller.PIDController(turnkP, turnkI, turnkD, turnPeriod)
        self.turnPID.enableContinuousInput(-180, 180)
        self.turnPID.setTolerance(self.turnTolerance)
        
        drivekP = self.maxWheelVelocity / self.maxVoltage
        drivekI = 0
        drivekD = 0
        drivePeriod = 0.02
        
        self.drivePID = controller.PIDController(drivekP, drivekI, drivekD, drivePeriod)
    
    def getAbsolutePositionZeroThreeSixty(self):
        return (self.absoluteEncoder.getAbsolutePosition() * 360) - self.absoluteOffset
    
    def getWheelAngleRadians(self):
        adjustedRelValDegrees = (self.turnEncoder.getPosition() / self.steeringRatio) % 360
        adjustedRelValDegrees -= 180
        return math.radians(adjustedRelValDegrees)
    
    def getWheelAngleDegrees(self):
        adjustedRelValDegrees = (self.turnEncoder.getPosition() / self.steeringRatio) % 360
        adjustedRelValDegrees -= 180
        return adjustedRelValDegrees
    
    def getTurnWheelState(self):
        return geometry.Rotation2d(self.getWheelAngleRadians())
    
    def setState(self, state: kinematics.SwerveModuleState):
        state = kinematics.SwerveModuleState.optimize(state, self.getTurnWheelState())
        '''
        May be needed:
        desiredStateAngle = state.angle.degrees()
        if desiredStateAngle > 0:
            desiredStateAngle -= 180
        else:
            desiredStateAngle += 180
        '''
        voltageTargetFF = self.maxVoltage * state.speed / self.maxWheelVelocity
        drivePIDOutputVolts = self.drivePID.calculate(self.driveEncoder.getVelocity() / (60 * self.drivingRatio), state.speed)
        self.driveMotor.setVoltage(voltageTargetFF + drivePIDOutputVolts)
        
        turnOutputVolts = self.turnPID.calculate(self.getWheelAngleDegrees(), (state.angle.degrees() - 180))
        self.turnMotor.setVoltage(turnOutputVolts)
    
    def coast(self):
        self.driveMotor.stopMotor()
        self.turnMotor.stopMotor()
    
    def reportPosition(self):
        SmartDashboard.putNumber(f"{self.moduleName} Drive Position", self.driveEncoder.getPosition() / self.drivingRatio)
        SmartDashboard.putNumber(f"{self.moduleName} Drive Velocity", self.driveEncoder.getVelocity() / (60 * self.drivingRatio))
        SmartDashboard.putNumber(f"{self.moduleName} ABS Turn Position", self.getAbsolutePositionZeroThreeSixty() - 180)
        SmartDashboard.putNumber(f"{self.moduleName} INT Turn Position", self.getWheelAngleDegrees())