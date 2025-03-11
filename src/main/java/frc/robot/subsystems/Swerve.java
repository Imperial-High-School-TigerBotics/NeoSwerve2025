package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.SwerveModule;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;


public class Swerve extends SubsystemBase {
    private final SwerveModule frontLeft = new SwerveModule(
        Constants.Swerve.Mod0.driveMotorID,
        Constants.Swerve.Mod0.angleMotorID,
        Constants.Swerve.Mod0.canCoderID,
        Constants.Swerve.Mod0.angleOffset
    );

    private final SwerveModule frontRight = new SwerveModule(
        Constants.Swerve.Mod1.driveMotorID,
        Constants.Swerve.Mod1.angleMotorID,
        Constants.Swerve.Mod1.canCoderID,
        Constants.Swerve.Mod1.angleOffset
    );

    private final SwerveModule backLeft = new SwerveModule(
        Constants.Swerve.Mod2.driveMotorID,
        Constants.Swerve.Mod2.angleMotorID,
        Constants.Swerve.Mod2.canCoderID,
        Constants.Swerve.Mod2.angleOffset
    );

    private final SwerveModule backRight = new SwerveModule(
        Constants.Swerve.Mod3.driveMotorID,
        Constants.Swerve.Mod3.angleMotorID,
        Constants.Swerve.Mod3.canCoderID,
        Constants.Swerve.Mod3.angleOffset
    );

    private AHRS gyro = new AHRS(NavXComType.kMXP_SPI);

    public Swerve() {

    }

    public void zeroHeading() {
        gyro.reset();
    }

    public double getHeading() {
        return Math.IEEEremainder(gyro.getAngle(), 360);
    }

    public Rotation2d getRotation2d(){
        return Rotation2d.fromDegrees(getHeading());
    }

    public void stopModules(){
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void setModuleStates(SwerveModuleState[] desiredStates){
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putNumber("Front Left Encoder", frontLeft.getAbsoluteEncoderPosition());
        SmartDashboard.putNumber("Front Right Encoder", frontRight.getAbsoluteEncoderPosition());
        SmartDashboard.putNumber("Back Left Encoder", backLeft.getAbsoluteEncoderPosition());
        SmartDashboard.putNumber("Back Right Encoder", backRight.getAbsoluteEncoderPosition());

    }
}