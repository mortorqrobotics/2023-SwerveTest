package org.team1515.SwerveTest;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import edu.wpi.first.wpilibj2.command.WaitCommand;

public class AutoCommandBalance extends SequentialCommandGroup {

    private double chargingStation = (29 + 48) / 12;

    /**
     * Runs auto one command after another (finished when the isFinished method
     * returns true)
     * 
     * @param # add params
     */
    public AutoCommandBalance(Swerve drivetrain) { // add params
        addCommands(
                new InstantCommand(() -> drivetrain.zeroGyro()),
                new DriveDist(drivetrain, Units.feetToMeters(3+chargingStation+6), 1),
                new DriveDist(drivetrain, Units.feetToMeters(4.5), -1),
                new AutoBalance(drivetrain));
    }
}