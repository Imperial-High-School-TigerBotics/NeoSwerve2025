package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;


public class TeleopSwerve extends Command{
    private final Swerve swerve;
    private final Supplier<Double> xSpdFunction, ySpdFunction, rotFunction;
    private final Supplier<Boolean> fieldOrientedFunction;
    private final SlewRateLimiter xLimiter, yLimiter, rotLimiter;
    
    public TeleopSwerve(Swerve swerve, Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction,
    Supplier<Double> rotFunction, Supplier<Boolean> fieldOrientedFunction){
        this.swerve = swerve;
        this.xSpdFunction = xSpdFunction;
        this.ySpdFunction = ySpdFunction;
        this.rotFunction = rotFunction;
        this.fieldOrientedFunction = fieldOrientedFunction;
        this.xLimiter = new SlewRateLimiter(Constants.Swerve.maxAcceleration);
        this.yLimiter = new SlewRateLimiter(Constants.Swerve.maxAcceleration);
        this.rotLimiter = new SlewRateLimiter(Constants.Swerve.maxAngularVelocity);
        addRequirements(swerve);
    }
    @Override
    public void execute(){
        double xSpeed = xSpdFunction.get();
        double ySpeed = ySpdFunction.get();
        double rot = rotFunction.get();

        xSpeed = Math.abs(xSpeed) < Constants.stickDeadband ? xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) < Constants.stickDeadband ? ySpeed : 0.0;
        rot = Math.abs(rot) < Constants.stickDeadband ? rot : 0.0;

        xSpeed = xLimiter.calculate(xSpeed) * Constants.Swerve.maxSpeed * Constants.Swerve.maxSpeedPercentScalar;
        ySpeed = yLimiter.calculate(ySpeed) * Constants.Swerve.maxSpeed * Constants.Swerve.maxSpeedPercentScalar;
        rot = rotLimiter.calculate(rot) * Constants.Swerve.maxAngularVelocity * Constants.Swerve.maxAngularVelocityPercentScalar;

        ChassisSpeeds chassisSpeeds;
        if(fieldOrientedFunction.get()){
            //Relative to the field
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, 
            rot, swerve.getRotation2d());
        } else {
            // Relative to the robot
            chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, rot);
        }

        SwerveModuleState[] moduleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(chassisSpeeds);

        swerve.setModuleStates(moduleStates);
    }

    @Override
    public void end(boolean interrupted){
        swerve.stopModules();
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}