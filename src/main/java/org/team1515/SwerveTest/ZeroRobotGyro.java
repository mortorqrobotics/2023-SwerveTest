package org.team1515.SwerveTest;

import com.team364.swervelib.util.SwerveConstants;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ZeroRobotGyro extends CommandBase {
    private Swerve drivetrainSubsystem;
    // l
    private PIDController angleController;
    private double maxRotate;

    private double p = 9;
    private double i = 2;
    private double d = 0.1;
    private double ff = 3.26;
    private double angle;

    /**
     * Align robot with the target using the limelight
     * 
     * @param drivetrainSubsystem
     * @param limelight
     */
    public ZeroRobotGyro(Swerve drivetrainSubsystem, double angle) {
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.maxRotate = SwerveConstants.Swerve.maxAngularVelocity;
        this.angle = angle;
        angleController = new PIDController(p, i, d);
        // TODO retune PID
        angleController.setTolerance(Units.degreesToRadians(3.5));
        angleController.enableContinuousInput(-Math.PI, Math.PI);
        angleController.setSetpoint(0.0);

        //addRequirements(drivetrainSubsystem);
    }

    @Override
    public void execute() {
        double error = MathUtil.angleModulus(RobotContainer.gyro.getGyroscopeRotation().getRadians()) - (drivetrainSubsystem.getRealZero().getRadians() + angle);
        System.out.println("Error: " + error);
        double rotation = (MathUtil.clamp(angleController.calculate(error, 0.0)+(ff*Math.signum(-error)), -maxRotate, maxRotate));
        System.out.println("Rotatiom: " + rotation);
        System.out.println("FF: " + ff);
        drivetrainSubsystem.drive(new Translation2d(0.0, 0.0), rotation, true, true);
    }

    @Override
    public boolean isFinished() {
        return angleController.atSetpoint();
    }
}
