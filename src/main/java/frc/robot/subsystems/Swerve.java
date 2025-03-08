package frc.robot.subsystems;

import frc.robot.SwerveModule;
import frc.robot.Constants;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swerve extends SubsystemBase {
    private final SwerveModule[] mSwerveMods;
    private final AHRS gyro;

    // WPILib swerve odometry
    private final SwerveDriveOdometry swerveOdometry;

    public Swerve() {
        gyro = new AHRS(NavXComType.kMXP_SPI);
        gyro.reset();

        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0),
            new SwerveModule(1),
            new SwerveModule(2),
            new SwerveModule(3)
        };

        // Build an array of the modules' initial positions (likely all zeros at startup).
        SwerveModulePosition[] initialPositions = new SwerveModulePosition[] {
            mSwerveMods[0].getModulePosition(),
            mSwerveMods[1].getModulePosition(),
            mSwerveMods[2].getModulePosition(),
            mSwerveMods[3].getModulePosition()
        };

        // Create the odometry object
        swerveOdometry = new SwerveDriveOdometry(
            Constants.Swerve.swerveKinematics,
            getGyroYaw(),
            initialPositions
        );
    }

    /**
     * Main drive method.
     * @param translation X/Y speeds in m/s
     * @param rotation    angular speed in rad/s
     * @param fieldRelative if true => field-oriented
     * @param isOpenLoop   not used here
     */
    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        // Convert to module states
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

        // Because you're open loop, you can still use desaturateWheelSpeeds to limit speeds
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        // Command each module
        for (int i = 0; i < mSwerveMods.length; i++) {
            mSwerveMods[i].setDesiredState(swerveModuleStates[i]);
        }
    }

    /**
     * Basic gyro reading: returns heading in standard WPILib CCW-positive format.
     */
    public Rotation2d getGyroYaw() {
        // Negate the navX yaw to match standard CCW-positive in WPILib
        return Rotation2d.fromDegrees(-gyro.getYaw());
    }

    /**
     * Zero the heading
     */
    public void zeroHeading() {
        gyro.reset();
    }

    @Override
    public void periodic() {
        // Update odometry with the gyro heading + each module's distance/angle
        SwerveModulePosition[] modulePositions = new SwerveModulePosition[mSwerveMods.length];
        for (int i = 0; i < mSwerveMods.length; i++) {
            modulePositions[i] = mSwerveMods[i].getModulePosition();
        }
        swerveOdometry.update(getGyroYaw(), modulePositions);

        // Log to SmartDashboard
        SmartDashboard.putNumber("Gyro Yaw", gyro.getYaw());
        SmartDashboard.putString("Odometry Pose", swerveOdometry.getPoseMeters().toString());
    }

    /**
     * Returns the current estimated pose of the robot on the field (meters).
     */
    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    /**
     * Reset odometry to the specified pose.
     */
    public void resetOdometry(Pose2d pose) {
        // Also re-sample current module distances to keep everything in sync
        SwerveModulePosition[] modulePositions = new SwerveModulePosition[mSwerveMods.length];
        for (int i = 0; i < mSwerveMods.length; i++) {
            modulePositions[i] = mSwerveMods[i].getModulePosition();
        }
        swerveOdometry.resetPosition(getGyroYaw(), modulePositions, pose);
    }
}
