package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class TeleopSwerve extends Command {
    private final Swerve s_Swerve;
    private final DoubleSupplier translationSup;
    private final DoubleSupplier strafeSup;
    private final DoubleSupplier rotationSup;
    private final BooleanSupplier robotCentricSup;

    public TeleopSwerve(
        Swerve s_Swerve,
        DoubleSupplier translationSup,
        DoubleSupplier strafeSup,
        DoubleSupplier rotationSup,
        BooleanSupplier robotCentricSup
    ) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;
    }

    @Override
    public void execute() {
        // Just for debugging
        SmartDashboard.putNumber("Gyro Yaw in Teleop", s_Swerve.getGyroYaw().getDegrees());

        // Deadband
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
        double strafeVal      = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband);
        double rotationVal    = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband);

        // Convert to m/s and rad/s (for the WPILib kinematics, if you still want them)
        Translation2d translation = new Translation2d(translationVal, strafeVal)
                                        .times(Constants.Swerve.maxSpeed);
        double rot = rotationVal * Constants.Swerve.maxAngularVelocity;

        // Drive
        s_Swerve.drive(translation, rot, !robotCentricSup.getAsBoolean(), true);
    }
}
