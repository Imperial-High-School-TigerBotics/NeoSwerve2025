package frc.robot;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;

public class SwerveModule{

    private final SparkMax driveMotor;
    private final SparkMax angleMotor;

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder angleEncoder;

    private final PIDController turningPIDController;

    private final CANcoder swerveAbsoluteEncoder;
    private final double absoluteEncoderOffset;


    public SwerveModule(int driveMotorID, int angleMotorID, int swerveAbsoluteEncoderID, Rotation2d absoluteEncoderOffset){
        this.absoluteEncoderOffset = absoluteEncoderOffset.getDegrees();
        swerveAbsoluteEncoder = new CANcoder(swerveAbsoluteEncoderID);

        driveMotor = new SparkMax(driveMotorID, MotorType.kBrushless);
        angleMotor = new SparkMax(angleMotorID, MotorType.kBrushless);

        driveEncoder = driveMotor.getEncoder();
        angleEncoder = angleMotor.getEncoder();
        
        turningPIDController = new PIDController(Constants.Swerve.angleKP, Constants.Swerve.angleKI, Constants.Swerve.angleKD);
        turningPIDController.enableContinuousInput(-Math.PI, Math.PI);

        resetEncoders();

    }

    public double getDrivePosition(){
        return driveEncoder.getPosition() * Constants.Swerve.drivePositionFactor;
    }
    
    public double getTurningPosition(){
        return angleEncoder.getPosition() * (2 * Math.PI);
    }
    
    public double getDriveVelocity(){
        return driveEncoder.getVelocity() * Constants.Swerve.driveVelocityFactor;
    }
    
    public double getTurningVelocity(){
        return angleEncoder.getVelocity() * (2 * Math.PI / 60.0);
    }

    public double getAbsoluteEncoderPosition(){

        double pos = (swerveAbsoluteEncoder.getAbsolutePosition().getValueAsDouble() * 360) - absoluteEncoderOffset;

        return Math.IEEEremainder(pos, 360);
    }

    public void resetEncoders(){
        driveEncoder.setPosition(0);
        angleEncoder.setPosition(getAbsoluteEncoderPosition());
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    public double rawAbsoluteEncoder(){
        return swerveAbsoluteEncoder.getAbsolutePosition().getValueAsDouble() * 360;
    }

    public void setDesiredState(SwerveModuleState desiredState){

        if(Math.abs(desiredState.speedMetersPerSecond) < 0.001){
            stop();
            return;
        }
        desiredState = SwerveModuleState.optimize(desiredState, getState().angle);
        driveMotor.set(desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed);
        angleMotor.set(turningPIDController.calculate(getTurningPosition(), desiredState.angle.getRadians()));

        SmartDashboard.putString("Swerve[" + swerveAbsoluteEncoder.getDeviceID() + "] desired state: ", desiredState.toString());
    }

    public void stop(){
        driveMotor.set(0);
        angleMotor.set(0);
    }
}