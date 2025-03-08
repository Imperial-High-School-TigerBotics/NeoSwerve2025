package frc.robot;

import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.Swerve;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * RobotContainer holds subsystems and binds controls to commands.
 */
public class RobotContainer {
    /* Controllers */
    private final XboxController driver = new XboxController(0);

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis      = XboxController.Axis.kLeftX.value;
    private final int rotationAxis    = XboxController.Axis.kRightX.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kA.value);

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();

    /* Auto Selector */
    private final SendableChooser<Command> chooser = new SendableChooser<>();

    public RobotContainer() {
        // Default teleop command:
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve,
                () -> -driver.getRawAxis(translationAxis),
                () -> -driver.getRawAxis(strafeAxis),
                () -> -driver.getRawAxis(rotationAxis),
                () -> false
            )
        );

        configureButtonBindings();
        configureAutoSelector();
    }

    private void configureButtonBindings() {
        // Zero gyro when A is pressed
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));
    }

    private void configureAutoSelector() {
        chooser.setDefaultOption("Default Auto", new InstantCommand());
        SmartDashboard.putData("Auto Mode", chooser);
    }

    public Command getAutonomousCommand() {
        return chooser.getSelected();
    }
}
