package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public final class Constants {
    public static final double stickDeadband = 0.08;

    public static class SpeedScaleFactors {
        public static final double autoTurnSpeed = 0.27;
    }

    public static final class Swerve {
        public static final int SwerveStartHeading = 0;

        /* Drivetrain Dimensions (meters) */
        public static final double trackWidth = 0.61595; // meters
        public static final double wheelBase  = 0.61595; // meters
        public static final double wheelCircumference = 0.319; // meters

        /* Swerve Kinematics */
        public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0,  trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0,  trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0)
        );

        /* Drive Motor PID Values (used for velocity) */
        public static final double driveKP = 0.5;
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.1;
        public static final double driveKF = 0.0; // If you want to do manual feedforward in the PID

        /* Drive Motor Characterization Values (for feedforward) */
        public static final double driveKS = 2.1;
        public static final double driveKV = 0.0;
        public static final double driveKA = 0.0;

        /* Turn (Angle) Motor PID Values */
        // These need to be tuned for your setup.
        public static final double angleKP = 0.01;
        public static final double angleKI = 0.0;
        public static final double angleKD = 0.0;

        /* Swerve Profiling Values */
        public static final double maxSpeed = 3.8;             // m/s
        public static final double maxAngularVelocity = 2*Math.PI; // rad/s

        /* Module Specific Constants */
        public static final class Mod0 { // Front Left
            public static final int driveMotorID = 1;
            public static final int angleMotorID = 3;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(75.234375);
        }

        public static final class Mod1 { // Front Right
            public static final int driveMotorID = 4;
            public static final int angleMotorID = 6;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(11.513672);
        }

        public static final class Mod2 { // Back Left
            public static final int driveMotorID = 7;
            public static final int angleMotorID = 9;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(108.193359);
        }

        public static final class Mod3 { // Back Right
            public static final int driveMotorID = 10;
            public static final int angleMotorID = 12;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-58.710938);
        }
    }
}
