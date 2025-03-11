package frc.robot;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;

import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Minimal swerve module that:
 *  - Reads the absolute steering angle from a CANCoder
 *  - Controls that angle with a PID loop (output -> angleMotor.set())
 *  - Drives the wheel in open-loop (fraction of maxSpeed)
 *  - Also provides odometry data: distance traveled + steering angle
 */
public class SwerveModule {
    public final int moduleNumber;

    private final SparkMax driveMotor;
    private final SparkMax angleMotor;
    private final CANcoder angleCANCoder;

    private final Rotation2d angleOffset;
    private final PIDController anglePID;

    // If your SparkMax class can give us position in rotations:
    // (If not, adapt accordingly to read from an alternate encoder.)
    private final RelativeEncoder driveEncoder;

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

        // Get the drive encoder from the SparkMax
        // (Adjust as needed based on your actual SparkMax library methods.)
        driveEncoder = driveMotor.getEncoder();

        // Optionally, you could set position conversion factors if your library supports it:
        // driveEncoder.setPositionConversionFactor(Constants.Swerve.drivePositionFactor);
        // driveEncoder.setVelocityConversionFactor(Constants.Swerve.driveVelocityFactor);

        // Create the angle PID
        anglePID = new PIDController(
            Constants.Swerve.angleKP,
            Constants.Swerve.angleKI,
            Constants.Swerve.angleKD
        );
        anglePID.enableContinuousInput(-180, 180);
    }

    /**
     * The only data we keep is the angle from the CANCoder and open-loop drive speed.
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
        double speedFraction = optimized.speedMetersPerSecond / Constants.Swerve.maxSpeed;

        // 5) Command the motors
        angleMotor.set(angleOutput);
        driveMotor.set(speedFraction);

        // 6) Dashboard (debug)
        SmartDashboard.putNumber("Module " + moduleNumber + " Current Angle", currentAngle);
        SmartDashboard.putNumber("Module " + moduleNumber + " Desired Angle", optimized.angle.getDegrees());
        SmartDashboard.putNumber("Module " + moduleNumber + " Speed Fraction", speedFraction);
    }

    /**
     * Return the distance traveled by this module's drive wheel, in meters.
     */
    public double getDriveDistanceMeters() {
        // If your SparkMax encoder position is in 'motor rotations':
        double motorRotations = driveEncoder.getPosition();
        return motorRotations * Constants.Swerve.drivePositionFactor;
    }

    /**
     * Returns the current steering angle (post-offset), in degrees from -180..180.
     */
    private double getCurrentAngleDegrees() {
        double rawAbsAngle = angleCANCoder.getPosition().getValueAsDouble(); // [0..360)
        double adjusted = rawAbsAngle - angleOffset.getDegrees();
        return wrapAngleDeg(adjusted);
    }

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
     * For WPILib odometry, we need a SwerveModulePosition: distance + module angle.
     */
    public SwerveModulePosition getModulePosition() {
        // "angle" must match the same reference used in setDesiredState (i.e. current angle)
        return new SwerveModulePosition(
            getDriveDistanceMeters(),
            Rotation2d.fromDegrees(getCurrentAngleDegrees())
        );
    }

    /**
     * For debugging: return the raw absolute angle from the CANCoder
     */
    public double getCanCoderAngle() {
        return angleCANCoder.getPosition().getValueAsDouble();
    }
}
