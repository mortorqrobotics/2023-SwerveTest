package org.team1515.SwerveTest;

import com.team364.swervelib.util.SwerveConstants;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutoBalance extends CommandBase {
    private PIDController controller;
    private Swerve drivetrain;
    private double maxSpeed;
    private LinearFilter filter = LinearFilter.movingAverage(5);

    private double p = 11;
    private double i = 2;
    private double d = 0.1;

    public AutoBalance(Swerve drivetrain) {
        this.drivetrain = drivetrain;
        this.maxSpeed = 0.5 * SwerveConstants.Swerve.maxSpeed;

        controller = new PIDController(p,i,d); // retun PID
        controller.setTolerance(Units.degreesToRadians(2));
        controller.enableContinuousInput(-Math.PI, Math.PI);
        controller.setSetpoint(0.0);

        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        double error = Math.toRadians(filter.calculate(RobotContainer.gyro.getRoll()));
        double speed = MathUtil.clamp(controller.calculate(-error, 0.0), -maxSpeed, maxSpeed);
        drivetrain.drive(new Translation2d(speed, 0.0), 0.0, false, true);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}