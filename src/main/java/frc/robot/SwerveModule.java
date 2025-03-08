package frc.robot;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Each SwerveModule handles a drive motor and angle motor, plus the associated offsets/PID.
 */
public class SwerveModule {
    public final int moduleNumber;
    private final Rotation2d angleOffset;

    private final SparkMax angleMotor;
    private final SparkMax driveMotor;

    // PID for angle
    private final PIDController anglePID;

    // PID for drive speed
    private final PIDController drivePID;

    // Feedforward for drive
    private final SimpleMotorFeedforward driveFeedForward =
            new SimpleMotorFeedforward(Constants.Swerve.driveKS,
                                       Constants.Swerve.driveKV,
                                       Constants.Swerve.driveKA);

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

        // Zero them out initially
        driveMotor.set(0);
        angleMotor.set(0);

        // Create angle PID controller
        anglePID = new PIDController(
            Constants.Swerve.angleKP,
            Constants.Swerve.angleKI,
            Constants.Swerve.angleKD
        );
        // Optional: anglePID.enableContinuousInput(-180, 180); // if you want wrapping

        // Create drive PID controller
        drivePID = new PIDController(
            Constants.Swerve.driveKP,
            Constants.Swerve.driveKI,
            Constants.Swerve.driveKD
        );

        // Example if you want to limit integral or something:
        // drivePID.setIntegratorRange(-0.2, 0.2);
    }

    /**
     * Set the desired state for this swerve module.
     * - Performs WPILib's optimize() to handle shortest rotation direction.
     * - Closes loop on angle with anglePID.
     * - Closes loop on drive speed with drivePID + feedforward.
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        // 1) Optimize the command to avoid spinning more than 90 degrees
        SwerveModuleState state = SwerveModuleState.optimize(desiredState, getState().angle);

        // 2) Find current angle and speed
        double currentAngle = getState().angle.getDegrees();
        double desiredAngle = state.angle.getDegrees();

        // 3) Calculate angle PID output
        double angleOutput = anglePID.calculate(currentAngle, desiredAngle);

        // 4) Calculate drive output (speed control)
        double currentSpeed = getState().speedMetersPerSecond;           // read from encoder
        double desiredSpeed = state.speedMetersPerSecond;
        double driveOutput = drivePID.calculate(currentSpeed, desiredSpeed);

        // 5) Add feedforward for the drive
        double feedforwardVolts = driveFeedForward.calculate(desiredSpeed);

        // 6) If your SparkMax set(...) method expects [-1, 1], we can divide by 12 to convert volts
        //    to a fraction of nominal battery voltage (approx). Adjust as needed for your library.
        double drivePercent = (driveOutput + feedforwardVolts) / 12.0;

        // 7) Command the motors
        angleMotor.set(angleOutput);  // open-loop, but from the PID output
        driveMotor.set(drivePercent);

        // 8) Update dashboard
        updateDashboard(state);
    }

    /**
     * Return the current state of the module (speed in m/s, angle as Rotation2d).
     * Here we subtract the angleOffset to get a "true" module heading.
     */
    public SwerveModuleState getState() {
        // NOTE: getEncoder().getPosition() might be raw rotations or degrees—depends on your library
        // We'll assume for example angleMotor.getEncoder().getPosition() returns degrees.
        double rawAngleDegrees = angleMotor.getEncoder().getPosition();
        double adjustedAngle = rawAngleDegrees - angleOffset.getDegrees();

        // We'll assume driveMotor.getEncoder().getVelocity() returns m/s directly.
        // If your library returns RPM, you must convert to m/s based on gear ratio & wheel circumference.
        double driveVelocity = driveMotor.getEncoder().getVelocity();

        return new SwerveModuleState(driveVelocity, Rotation2d.fromDegrees(adjustedAngle));
    }

    /**
     * Return the module position (distance traveled in meters, current angle).
     */
    public SwerveModulePosition getPosition() {
        // We'll assume driveMotor.getEncoder().getPosition() is the distance traveled in meters
        // If it's actually rotations, multiply by (wheelCircumference * gearRatio).
        double driveMeters = driveMotor.getEncoder().getPosition();

        double rawAngleDegrees = angleMotor.getEncoder().getPosition();
        double adjustedAngle  = rawAngleDegrees - angleOffset.getDegrees();

        return new SwerveModulePosition(driveMeters, Rotation2d.fromDegrees(adjustedAngle));
    }

    /**
     * Reports the "absolute" angle from the angle motor's encoder, without subtracting offset.
     * Typically used for debugging or calibrating offsets.
     */
    public Rotation2d getCanCoderAngle() {
        return Rotation2d.fromDegrees(angleMotor.getEncoder().getPosition());
    }

    private void updateDashboard(SwerveModuleState desiredState) {
        SmartDashboard.putNumber("Swerve Module " + moduleNumber + " Current Angle", getState().angle.getDegrees());
        SmartDashboard.putNumber("Swerve Module " + moduleNumber + " Desired Angle", desiredState.angle.getDegrees());
        SmartDashboard.putNumber("Swerve Module " + moduleNumber + " Speed (m/s)", getState().speedMetersPerSecond);
        SmartDashboard.putNumber("Swerve Module " + moduleNumber + " Desired Speed", desiredState.speedMetersPerSecond);
    }
}
