package frc.robot.subsystems;

import frc.robot.SwerveModule;
import frc.robot.Constants;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * The main Swerve subsystem, which holds four SwerveModules, the gyro, and does odometry.
 */
public class Swerve extends SubsystemBase {
    private final SwerveDriveOdometry swerveOdometry;
    public final SwerveModule[] mSwerveMods;
    public final AHRS gyro;
    public boolean autonMovingEnabled;

    public Swerve() {
        gyro = new AHRS(NavXComType.kMXP_SPI);
        gyro.reset();

        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0),
            new SwerveModule(1),
            new SwerveModule(2),
            new SwerveModule(3)
        };

        swerveOdometry = new SwerveDriveOdometry(
            Constants.Swerve.swerveKinematics,
            getGyroYaw(),
            getModulePositions()
        );

        autonMovingEnabled = true;
    }

    /**
     * Primary drive method. Takes in translation (x, y) in m/s, rotation in rad/s,
     * and whether we want field-relative driving or not.
     */
    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        // 1) Convert driver commands to desired module states
        SwerveModuleState[] swerveModuleStates =
                Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                        fieldRelative
                            ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                  translation.getX(),
                                  translation.getY(),
                                  rotation,
                                  getGyroYaw()
                              )
                            : new ChassisSpeeds(
                                  translation.getX(),
                                  translation.getY(),
                                  rotation
                              )
                );

        // 2) Desaturate wheel speeds if any goes above max
        SwerveDriveKinematics.desaturateWheelSpeeds(
                swerveModuleStates,
                Constants.Swerve.maxSpeed
        );

        // 3) Send each module its desired state
        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber]);
        }
    }

    /**
     * Return array of all module states, front-left = index 0, etc.
     */
    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule mod : mSwerveMods) {
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    /**
     * Return array of all module positions (distance + angle).
     */
    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (SwerveModule mod : mSwerveMods) {
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    /**
     * Set (reset) the odometry pose to a chosen Pose2d.
     */
    public void setPose(Pose2d pose) {
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), pose);
    }

    /**
     * Returns gyro's heading as Rotation2d, negated to match WPILib's standard "counter-clockwise positive" convention.
     */
    public Rotation2d getGyroYaw() {
        return Rotation2d.fromDegrees(-gyro.getYaw());
    }

    /**
     * Zero the heading and also adjust the odometry so the current heading is now 0.
     */
    public void zeroHeading() {
        gyro.reset();
        swerveOdometry.resetPosition(
            getGyroYaw(),
            getModulePositions(),
            new Pose2d(getPose().getTranslation(), new Rotation2d())
        );
    }

    /**
     * Log the raw CANcoder angles for each module to SmartDashboard (mainly for debugging offsets).
     */
    public void logCanCoderOffsets() {
        for (SwerveModule mod : mSwerveMods) {
            SmartDashboard.putNumber(
                "Module " + mod.moduleNumber + " CANcoder Offset",
                mod.getCanCoderAngle().getDegrees()
            );
        }
    }

    @Override
    public void periodic() {
        // Update odometry with the latest gyro heading and module positions
        swerveOdometry.update(getGyroYaw(), getModulePositions());
        SmartDashboard.putNumber("Gyro Yaw", gyro.getYaw());
        logCanCoderOffsets();
    }
}
