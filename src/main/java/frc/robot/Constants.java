package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public final class Constants {
    public static final double stickDeadband = 0.08;

    public static final class Swerve {
        // Drivetrain geometry (used by the kinematics if you want to keep them)
        public static final double trackWidth = 0.61595; // meters
        public static final double wheelBase  = 0.61595; // meters

        // Kinematics (still used if you want field-relative)
        public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0,  trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0,  trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0)
        );

        // Drive open-loop max speed
        public static final double maxSpeed           = 3.8;         // m/s
        public static final double maxAngularVelocity = 2 * Math.PI; // rad/s

        // Angle PID Gains
        public static final double angleKP = 0.01;
        public static final double angleKI = 0.0;
        public static final double angleKD = 0.0;

        // Module IDs + Offsets
        public static final class Mod0 { // Front Left
            public static final int driveMotorID = 1;
            public static final int angleMotorID = 3;
            public static final int canCoderID   = 21;
            public static final Rotation2d angleOffset  = Rotation2d.fromDegrees(75.234375);
        }

        public static final class Mod1 { // Front Right
            public static final int driveMotorID = 4;
            public static final int angleMotorID = 6;
            public static final int canCoderID   = 22;
            public static final Rotation2d angleOffset  = Rotation2d.fromDegrees(11.513672);
        }

        public static final class Mod2 { // Back Left
            public static final int driveMotorID = 7;
            public static final int angleMotorID = 9;
            public static final int canCoderID   = 23;
            public static final Rotation2d angleOffset  = Rotation2d.fromDegrees(108.193359);
        }

        public static final class Mod3 { // Back Right
            public static final int driveMotorID = 10;
            public static final int angleMotorID = 12;
            public static final int canCoderID   = 24;
            public static final Rotation2d angleOffset  = Rotation2d.fromDegrees(-58.710938);
        }
    }
}
