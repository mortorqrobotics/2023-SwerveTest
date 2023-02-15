package org.team1515.SwerveTest;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.team364.swervelib.util.SwerveConstants;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SwerveCommand extends CommandBase {
    private Swerve drivetrain;
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier robotCentricSup;

    private SlewRateLimiter filterX = new SlewRateLimiter(0.5, -0.5, 0);
    private SlewRateLimiter filterY = new SlewRateLimiter(0.5, -0.5, 0);

    public SwerveCommand(Swerve drivetrain, DoubleSupplier translationSup, DoubleSupplier strafeSup,
            DoubleSupplier rotationSup, BooleanSupplier robotCentricSup) {
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;
    }

    @Override
    public void execute() {
        /* Get Values, Deadband */
        double translationVal = filterY.calculate(translationSup.getAsDouble());
        double strafeVal = filterX.calculate(strafeSup.getAsDouble());
        double rotationVal = rotationSup.getAsDouble();

        System.out.println(translationVal);
        System.out.println(strafeVal);
        System.out.println(rotationVal);

        /* Drive */
        drivetrain.drive(
                new Translation2d(translationVal, strafeVal).times(SwerveConstants.Swerve.maxSpeed),
                rotationVal * SwerveConstants.Swerve.maxAngularVelocity,
                !robotCentricSup.getAsBoolean(),
                false);
    }
}