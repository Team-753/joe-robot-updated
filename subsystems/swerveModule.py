import rev
from wpimath import controller, kinematics, geometry
from wpilib import AnalogEncoder, SmartDashboard
import math

class SwerveModule:
    steeringRatio = 8.5722
    drivingRatio = 5.04
    turnTolerance = 0.5 # degrees
    maxWheelVelocity = 3.4 # meters per second
    maxVoltage = 12.0
    wheelDiameterMeters = 0.1016
    
    def __init__(self, config: dict, name: str) -> None:
        self.config = config
        self.moduleName = name
        
        self.driveMotor = rev.CANSparkMax(self.config["driveMotorID"], rev.CANSparkMax.MotorType.kBrushless)
        self.driveMotor.enableVoltageCompensation(12.0)
        self.driveMotor.setIdleMode(rev.CANSparkMax.IdleMode.kCoast)
        self.driveMotor.setInverted(False)
        self.driveEncoder = self.driveMotor.getEncoder()
        
        self.turnMotor = rev.CANSparkMax(self.config["turnMotorID"], rev.CANSparkMax.MotorType.kBrushless)
        self.turnMotor.enableVoltageCompensation(12.0)
        self.turnMotor.setIdleMode(rev.CANSparkMax.IdleMode.kCoast)
        self.turnMotor.setInverted(False)
        self.turnEncoder = self.turnMotor.getEncoder()
        
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
        
        self.turnEncoder.setPosition(self.absoluteToMotorRotations())
        self.driveEncoder.setPosition(0)
    
    def getAbsolutePositionZeroThreeSixty(self):
        val = ((1 - self.absoluteEncoder.getAbsolutePosition()) * 360) - self.absoluteOffset
        if val < 0:
            val += 360
        elif val > 360:
            val -= 360
        return val
    
    def absoluteToMotorRotations(self):
        absolute = 1 - self.getAbsolutePositionZeroThreeSixty()
        absolute *= self.steeringRatio # first converting it from wheel to motor relative
        absolute /= 360 # converting from degrees to rotations
        absolute *= 42 # converting from rotations into ticks
        return absolute
    
    def getWheelAngleRadians(self):
        return math.radians(((self.turnEncoder.getPosition() * 360) / (self.steeringRatio * 42)) % 360)
    
    def getWheelAngleDegrees(self):
        return ((self.turnEncoder.getPosition() * 360) / (self.steeringRatio * 42)) % 360
    
    def getTurnWheelState(self):
        ''' For odometry purposes '''
        return geometry.Rotation2d(self.getWheelAngleRadians())
    
    def setState(self, state: kinematics.SwerveModuleState):
        state = kinematics.SwerveModuleState.optimize(state, self.getTurnWheelState())
        
        desiredStateAngle = state.angle.degrees()
        
        SmartDashboard.putNumber(f"{self.moduleName} Turn Target", desiredStateAngle)
        
        voltageTargetFF = self.maxVoltage * state.speed / self.maxWheelVelocity
        drivePIDOutputVolts = self.drivePID.calculate((self.driveEncoder.getVelocity() * math.pi * self.wheelDiameterMeters * self.drivingRatio) / (60 * 42), state.speed)
        self.driveMotor.setVoltage(voltageTargetFF + drivePIDOutputVolts)
        if self.moduleName == "rearRight":
            desiredStateAngle -= 180
            if desiredStateAngle < -180:
                desiredStateAngle += 360
        turnOutputVolts = self.turnPID.calculate(self.getAbsolutePositionZeroThreeSixty() - 180, desiredStateAngle)
        self.turnMotor.setVoltage(turnOutputVolts)
        
    def coast(self):
        self.driveMotor.stopMotor()
        self.turnMotor.stopMotor()
    
    def reportPosition(self):
        SmartDashboard.putNumber(f"{self.moduleName} Drive Position", (self.driveEncoder.getPosition() * math.pi * self.wheelDiameterMeters * self.drivingRatio) / 42)
        SmartDashboard.putNumber(f"{self.moduleName} Drive Velocity", (self.driveEncoder.getVelocity() * math.pi * self.wheelDiameterMeters * self.drivingRatio) / (60 * 42))
        SmartDashboard.putNumber(f"{self.moduleName} ABS Turn Position", self.getAbsolutePositionZeroThreeSixty())
        SmartDashboard.putNumber(f"{self.moduleName} INT Turn Position", self.getWheelAngleDegrees())