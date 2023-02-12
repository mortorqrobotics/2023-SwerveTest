package org.team1515.SwerveTest;

import com.team364.swervelib.math.Conversions;
import com.team364.swervelib.util.SwerveConstants;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TuneFF extends CommandBase {
    private Swerve drivetrain;
    private double speed = 0;
    
    public TuneFF(Swerve drivetrain) {
        this.drivetrain = drivetrain;

        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        // drivetrain.drive(new Translation2d(speed, 0.0), 0.0, false, true);
        speed += 0.001;
        drivetrain.setVolts(speed);
        System.out.println(speed);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.drive(new Translation2d(0.0,0.0), 1, false, true);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}