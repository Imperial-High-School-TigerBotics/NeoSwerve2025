package frc.robot;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Minimal swerve module that:
 *  - Reads the absolute steering angle from a CANCoder
 *  - Controls that angle with a PID loop (output -> angleMotor.set())
 *  - Drives the wheel in open-loop (fraction of maxSpeed)
 *  - No integrated encoders, no distance measurement
 */
public class SwerveModule {
    public final int moduleNumber;

    // The two motors
    private final SparkMax driveMotor;
    private final SparkMax angleMotor;

    // The CANCoder for absolute steering angle
    private final CANcoder angleCANCoder;

    // Mechanical offset
    private final Rotation2d angleOffset;

    // Angle PID (if you want closed-loop turning)
    private final PIDController anglePID;

    public SwerveModule(int moduleNumber) {
        this.moduleNumber = moduleNumber;

        int driveMotorID, angleMotorID, canCoderID;
        Rotation2d offset;

        switch (moduleNumber) {
            case 0:
                driveMotorID = Constants.Swerve.Mod0.driveMotorID;
                angleMotorID = Constants.Swerve.Mod0.angleMotorID;
                canCoderID   = Constants.Swerve.Mod0.canCoderID;
                offset       = Constants.Swerve.Mod0.angleOffset;
                break;
            case 1:
                driveMotorID = Constants.Swerve.Mod1.driveMotorID;
                angleMotorID = Constants.Swerve.Mod1.angleMotorID;
                canCoderID   = Constants.Swerve.Mod1.canCoderID;
                offset       = Constants.Swerve.Mod1.angleOffset;
                break;
            case 2:
                driveMotorID = Constants.Swerve.Mod2.driveMotorID;
                angleMotorID = Constants.Swerve.Mod2.angleMotorID;
                canCoderID   = Constants.Swerve.Mod2.canCoderID;
                offset       = Constants.Swerve.Mod2.angleOffset;
                break;
            case 3:
                driveMotorID = Constants.Swerve.Mod3.driveMotorID;
                angleMotorID = Constants.Swerve.Mod3.angleMotorID;
                canCoderID   = Constants.Swerve.Mod3.canCoderID;
                offset       = Constants.Swerve.Mod3.angleOffset;
                break;
            default:
                throw new IllegalArgumentException("Invalid module number: " + moduleNumber);
        }

        angleOffset = offset;

        // Instantiate the SparkMax motors
        driveMotor = new SparkMax(driveMotorID, MotorType.kBrushless);
        angleMotor = new SparkMax(angleMotorID, MotorType.kBrushless);

        driveMotor.set(0);
        angleMotor.set(0);

        // Create the CANCoder
        angleCANCoder = new CANcoder(canCoderID);


        // Create the angle PID
        anglePID = new PIDController(
            Constants.Swerve.angleKP,
            Constants.Swerve.angleKI,
            Constants.Swerve.angleKD
        );
        // optional if you want -180..180 wrap
        anglePID.enableContinuousInput(-180, 180);
    }

    /**
     * The only data we keep is the angle from the CANCoder and the open-loop drive speed.
     * - We'll do WPILib "optimize" so we pick the shortest turning direction.
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        // 1) Figure out the current angle
        double currentAngle = getCurrentAngleDegrees();

        // 2) Optimize
        SwerveModuleState optimized = SwerveModuleState.optimize(
            desiredState,
            Rotation2d.fromDegrees(currentAngle)
        );

        // 3) Angle PID
        double angleOutput = anglePID.calculate(currentAngle, optimized.angle.getDegrees());

        // 4) Open-loop drive: fraction of max speed
        //    If desired speed is e.g. 2 m/s and maxSpeed = 4 m/s, we do 2/4 => 0.5
        double speedFraction = optimized.speedMetersPerSecond / Constants.Swerve.maxSpeed;

        // 5) Command the motors
        angleMotor.set(angleOutput);
        driveMotor.set(speedFraction);

        // 6) Dashboard
        SmartDashboard.putNumber("Module " + moduleNumber + " Current Angle", currentAngle);
        SmartDashboard.putNumber("Module " + moduleNumber + " Desired Angle", optimized.angle.getDegrees());
        SmartDashboard.putNumber("Module " + moduleNumber + " Speed Fraction", speedFraction);
    }

    /**
     * We'll just define "current angle" as:
     *   canCoder - angleOffset
     */
    private double getCurrentAngleDegrees() {
        double rawAbsAngle = angleCANCoder.getPosition().getValueAsDouble(); // [0..360)
        double adjusted     = rawAbsAngle - angleOffset.getDegrees();
        // Optionally wrap into [-180..180] or just leave it
        return wrapAngleDeg(adjusted);
    }

    /**
     * If you want to wrap angles from any range into [-180..180]
     */
    private double wrapAngleDeg(double deg) {
        double wrapped = deg % 360.0;
        if (wrapped > 180.0) {
            wrapped -= 360.0;
        } else if (wrapped < -180.0) {
            wrapped += 360.0;
        }
        return wrapped;
    }

    /**
     * For debugging: return the raw absolute angle from the CANCoder
     */
    public double getCanCoderAngle() {
        return angleCANCoder.getPosition().getValueAsDouble();
    }
}
