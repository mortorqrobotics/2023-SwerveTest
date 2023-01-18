package org.team1515.SwerveTest;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Controls {
    // change botton maps later

    public static final Trigger ALIGN = new Trigger(RobotContainer.mainController::getStartButton); // will limelight
                                                                                                    // and apriltag
                                                                                                    // overlap?
    public static final Trigger RESET_GYRO = new Trigger(RobotContainer.mainController::getBackButton);
    public static final BooleanSupplier DRIVE_ROBOT_ORIENTED = () -> RobotContainer.mainController.getLeftBumper();

    public static boolean getRightTrigger() {
        return RobotContainer.mainController.getRightTriggerAxis() >= 0.250;
    }

    public static boolean getLeftTrigger() {
        return RobotContainer.mainController.getLeftTriggerAxis() >= 0.250;
    }

    public static boolean secondRightStickTrigger() {
        return RobotContainer.secondController.getRightTriggerAxis() >= 0.250;
    }
}