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

    private double p = 54;
    private double i = 0;
    private double d = 0;
    private double ff = 3.23;

    /**
     * Align robot with the target using the limelight
     * 
     * @param drivetrainSubsystem
     * @param limelight
     */
    public ZeroRobotGyro(Swerve drivetrainSubsystem) {
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.maxRotate = SwerveConstants.Swerve.maxAngularVelocity;

        angleController = new PIDController(p, i, d);
        // TODO retune PID
        angleController.setTolerance(0.025);
        angleController.enableContinuousInput(-Math.PI, Math.PI);
        angleController.setSetpoint(0.0);

        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void execute() {
        double error = MathUtil.angleModulus(RobotContainer.gyro.getGyroscopeRotation().getRadians()) - drivetrainSubsystem.getRealZero().getRadians();
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
