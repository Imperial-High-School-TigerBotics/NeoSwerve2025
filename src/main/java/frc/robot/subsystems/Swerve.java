package frc.robot.subsystems;

import frc.robot.SwerveModule;
import frc.robot.Constants;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
// import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
// import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swerve extends SubsystemBase {
    // If you don't want odometry at all, remove it
    // private SwerveDriveOdometry swerveOdometry;

    private final SwerveModule[] mSwerveMods;
    private final AHRS gyro;

    public Swerve() {
        gyro = new AHRS(NavXComType.kMXP_SPI);
        gyro.reset();

        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0),
            new SwerveModule(1),
            new SwerveModule(2),
            new SwerveModule(3)
        };

        // If we had odometry, we'd do something like:
        // swerveOdometry = new SwerveDriveOdometry(
        //     Constants.Swerve.swerveKinematics,
        //     getGyroYaw(),
        //     new SwerveModulePosition[] {
        //         ... positions ...
        //     }
        // );
    }

    /**
     * Main drive method.
     * @param translation  X/Y speeds in m/s
     * @param rotation     angular speed in rad/s
     * @param fieldRelative if true => fromFieldRelativeSpeeds
     * @param isOpenLoop   not used here (everything is open loop on drive anyway)
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
        SwerveDriveKinematics.desaturateWheelSpeeds(
            swerveModuleStates,
            Constants.Swerve.maxSpeed
        );

        // Command each module
        for (int i = 0; i < mSwerveMods.length; i++) {
            mSwerveMods[i].setDesiredState(swerveModuleStates[i]);
        }
    }

    /**
     * Basic gyro reading
     */
    public Rotation2d getGyroYaw() {
        // negate to match standard CCW-positive
        return Rotation2d.fromDegrees(-gyro.getYaw());
    }

    /**
     * Zero the heading
     */
    public void zeroHeading() {
        gyro.reset();
    }

    /**
     * Periodic: if you don't do odometry, you can just log the gyro
     */
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Gyro Yaw", gyro.getYaw());
    }
}
