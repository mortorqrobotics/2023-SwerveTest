package org.team1515.SwerveTest;

import com.team364.swervelib.util.SwerveConstants;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;

public class ZeroRobot extends CommandBase {
    private Swerve drivetrainSubsystem;
    // l
    private PIDController angleController;
    private double maxRotate;

    private double p = 0;
    private double i = 0;
    private double d = 0;

    /**
     * Align robot with the target using the limelight
     * 
     * @param drivetrainSubsystem
     * @param limelight
     */
    public ZeroRobot(Swerve drivetrainSubsystem) {
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.maxRotate = 0.5 * SwerveConstants.Swerve.maxAngularVelocity;

        angleController = new PIDController(p, i, d);
        // TODO retune PID
        angleController.setTolerance(0.025);
        angleController.enableContinuousInput(-Math.PI, Math.PI);
        angleController.setSetpoint(0.0);

        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void execute() {
        double error = drivetrainSubsystem.getPose().getRotation().getRadians();
        if (error == 0) // Stop auto align if camera has no target in view
            this.end(true);
        double rotation = MathUtil.clamp(angleController.calculate(error, 0.0), -maxRotate, maxRotate);
        drivetrainSubsystem.drive(new Translation2d(0.0, 0.0), rotation, true, true);
    }

    @Override
    public boolean isFinished() {
        return angleController.atSetpoint();
    }
}
