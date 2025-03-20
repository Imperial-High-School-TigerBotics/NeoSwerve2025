package frc.robot;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModule {
    public int moduleNumber;
    private Rotation2d angleOffset;

    private SparkMax angleMotor;
    private SparkMax driveMotor;

    private final SimpleMotorFeedforward driveFeedForward = new SimpleMotorFeedforward(
        Constants.Swerve.driveKS, Constants.Swerve.driveKV, Constants.Swerve.driveKA
    );

    public SwerveModule(int moduleNumber) {
        this.moduleNumber = moduleNumber;

        // Assign motor IDs and offsets based on module number
        switch (moduleNumber) {
            case 0:
                angleMotor = new SparkMax(Constants.Swerve.Mod0.angleMotorID, MotorType.kBrushless);
                driveMotor = new SparkMax(Constants.Swerve.Mod0.driveMotorID, MotorType.kBrushless);
                angleOffset = Constants.Swerve.Mod0.angleOffset;
                break;
            case 1:
                angleMotor = new SparkMax(Constants.Swerve.Mod1.angleMotorID, MotorType.kBrushless);
                driveMotor = new SparkMax(Constants.Swerve.Mod1.driveMotorID, MotorType.kBrushless);
                angleOffset = Constants.Swerve.Mod1.angleOffset;
                break;
            case 2:
                angleMotor = new SparkMax(Constants.Swerve.Mod2.angleMotorID, MotorType.kBrushless);
                driveMotor = new SparkMax(Constants.Swerve.Mod2.driveMotorID, MotorType.kBrushless);
                angleOffset = Constants.Swerve.Mod2.angleOffset;
                break;
            case 3:
                angleMotor = new SparkMax(Constants.Swerve.Mod3.angleMotorID, MotorType.kBrushless);
                driveMotor = new SparkMax(Constants.Swerve.Mod3.driveMotorID, MotorType.kBrushless);
                angleOffset = Constants.Swerve.Mod3.angleOffset;
                break;
            default:
                throw new IllegalArgumentException("Invalid module number: " + moduleNumber);
        }

        driveMotor.set(0);
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        desiredState = SwerveModuleState.optimize(desiredState, getState().angle);
        angleMotor.set(desiredState.angle.getDegrees() / 180.0); // Simple conversion
        driveMotor.set(desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed);
        updateDashboard();
    }

    public void updateDashboard() {
        SmartDashboard.putNumber("Swerve Module " + moduleNumber + " Angle", getState().angle.getDegrees());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
            driveMotor.getEncoder().getVelocity(),
            Rotation2d.fromDegrees(angleMotor.getEncoder().getPosition())
        );
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            driveMotor.getEncoder().getPosition(),
            Rotation2d.fromDegrees(angleMotor.getEncoder().getPosition())
        );
    }
}
