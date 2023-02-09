package org.team1515.SwerveTest;

import com.team364.swervelib.util.SwerveConstants;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveDist extends CommandBase {
    private Swerve drivetrainSubsystem;
    private double targetDist;
    private double direction;
    private double lastTime;

    private double distTraveled = 0.0;
    private double maxSpeed;
    private Rotation2d startGyroAngle;

    public DriveDist(Swerve drivetrainSubsystem, double targetDist) {
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.targetDist = targetDist;
        this.maxSpeed = 0.1 * SwerveConstants.Swerve.maxSpeed;

        this.direction = 1;

        SmartDashboard.putNumber("target dist", targetDist);
        addRequirements(drivetrainSubsystem);
    }

    public DriveDist(Swerve drivetrainSubsystem, double targetDist, double direction) {
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.targetDist = targetDist;
        this.direction = direction;
        this.maxSpeed = 0.1 * SwerveConstants.Swerve.maxSpeed;

        SmartDashboard.putNumber("target dist", targetDist);
        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void initialize() {
        this.lastTime = System.currentTimeMillis();
        this.distTraveled = 0.0;
        this.startGyroAngle = RobotContainer.gyro.getGyroscopeRotation();
    }

    @Override
    public void execute() {
        distanceTraveled();
        drivetrainSubsystem.drive(new Translation2d(maxSpeed * direction, 0.0),
                MathUtil.applyDeadband(startGyroAngle.minus(RobotContainer.gyro.getGyroscopeRotation()).getRadians(), 0.01),
                false, false);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrainSubsystem.drive(new Translation2d(0.0, 0.0), 0.0, false, false);
    }

    /**
     * Gets the distance traveled by multiplying rate (drive velocity) and time
     * (milliseconds)
     */
    public void distanceTraveled() {
        double deltaTime = System.currentTimeMillis() - lastTime;
        distTraveled += Math.abs(drivetrainSubsystem.getModuleStates()[0].speedMetersPerSecond) * (1.0 / 1000)
                * deltaTime;
        lastTime = System.currentTimeMillis();
    }

    @Override
    public boolean isFinished() {
        return distTraveled >= targetDist;
    }
} 