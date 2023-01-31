// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team1515.SwerveTest;

import com.team364.swervelib.util.SwerveConstants;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RobotContainer {
  public static XboxController mainController;
  public static XboxController secondController;

  public static Gyroscope gyro;
  public static Swerve drivetrain;
  public static PhotonVisionWrapper pvw;

  public RobotContainer() {
    mainController = new XboxController(0);

    gyro = new Gyroscope();

    pvw = new PhotonVisionWrapper();

    drivetrain = new Swerve(new Pose2d(new Translation2d(0.0, 0.0), new Rotation2d(0.0)));

    configureBindings();
  }

  private void configureBindings() {

    drivetrain.setDefaultCommand(
        new SwerveCommand(drivetrain,
            () -> -modifyAxis(-mainController.getLeftY() * getRobotSpeed()),
            () -> -modifyAxis(-mainController.getLeftX() * getRobotSpeed()),
            () -> -modifyAxis(mainController.getRightX() * getRobotSpeed()),
            () -> Controls.DRIVE_ROBOT_ORIENTED.getAsBoolean()));

    Controls.RESET_GYRO.onTrue(new InstantCommand(() -> drivetrain.zeroGyro())); // drivetrain::zeroGyro not working
    Controls.ZERO_ROBOT.onTrue(new ZeroRobotGyro(drivetrain));

    // Controls.ALIGN.onTrue(new Align(drivetrain));

  }

  // public Command getAutonomousCommand() {
  // return new AutoCommandScore(drivetrain);
  // }

  public static double getRobotSpeed() {
    return Controls.getLeftTrigger() ? 0.4 : 0.7;
    // return 0.7;
  }

  public static double modifyAxis(double value) {
    value = deadband(value, 0.08);

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }

  public static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

}
