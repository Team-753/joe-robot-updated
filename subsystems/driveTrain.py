import commands2
from wpimath import kinematics, geometry
import navx
from subsystems.swerveModule import SwerveModule
from math import hypot
from wpilib import SmartDashboard

class DriveTrain(commands2.SubsystemBase):
    
    def __init__(self, config: dict) -> None:
        super().__init__()

        self.config = config
        
        self.navx = navx.AHRS.create_spi()
        
        self.kMaxSpeed = config["RobotDefaultSettings"]["wheelVelocityLimit"]
        self.wheelBase = self.config["RobotDimensions"]["wheelBase"]
        self.trackWidth = self.config["RobotDimensions"]["trackWidth"]
        
        self.KINEMATICS = kinematics.SwerveDrive4Kinematics(
            geometry.Translation2d(self.trackWidth / 2, self.wheelBase / 2),
            geometry.Translation2d(self.trackWidth / 2, -self.wheelBase / 2),
            geometry.Translation2d(-self.trackWidth / 2, self.wheelBase / 2),
            geometry.Translation2d(-self.trackWidth / 2, -self.wheelBase / 2))
        
        self.frontLeft = SwerveModule(self.config["SwerveModules"]["frontLeft"], "frontLeft")
        self.frontRight = SwerveModule(self.config["SwerveModules"]["frontRight"], "frontRight")
        self.rearLeft = SwerveModule(self.config["SwerveModules"]["rearLeft"], "rearLeft")
        self.rearRight = SwerveModule(self.config["SwerveModules"]["rearRight"], "rearRight")
        
        teleopConstants = self.config["driverStation"]["teleoperatedRobotConstants"]
        self.maxAngularVelocity = teleopConstants["teleopVelLimit"] / hypot(self.config["RobotDimensions"]["trackWidth"] / 2, self.config["RobotDimensions"]["wheelBase"] / 2)
        
    def joystickDrive(self, inputs: list, fieldOrient: bool) -> None:
        x, y, z = inputs[0], inputs[1], inputs[2]
        x *= self.kMaxSpeed
        y *= self.kMaxSpeed
        z*= self.maxAngularVelocity
        if x == 0 and y == 0 and z == 0:
            self.coast()
        else:
            if False: # if fieldOrient:
                swerveModuleStates = self.KINEMATICS.toSwerveModuleStates(kinematics.ChassisSpeeds.fromFieldRelativeSpeeds(kinematics.ChassisSpeeds(x, y, z), self.getNAVXRotation2d()))
            else:
                swerveModuleStates = self.KINEMATICS.toSwerveModuleStates(kinematics.ChassisSpeeds(x, y, z))
            
            self.KINEMATICS.desaturateWheelSpeeds(swerveModuleStates, self.kMaxSpeed)
            self.frontLeft.setState(swerveModuleStates[0])
            self.frontRight.setState(swerveModuleStates[1])
            self.rearLeft.setState(swerveModuleStates[2])
            self.rearRight.setState(swerveModuleStates[3])
    
    def getNAVXRotation2d(self):
        return self.navx.getRotation2d()
    
    def coast(self):
        self.frontLeft.coast()
        self.frontRight.coast()
        self.rearRight.coast()
        self.rearLeft.coast()
    
    def periodic(self) -> None:
        self.frontLeft.reportPosition()
        self.frontRight.reportPosition()
        self.rearRight.reportPosition()
        self.rearLeft.reportPosition()
        SmartDashboard.putNumber("NAVX ROTATION", self.getNAVXRotation2d().degrees())