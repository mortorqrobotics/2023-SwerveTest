package org.team1515.SwerveTest;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.SPI;

public class Gyroscope {
    private final AHRS navx;
    private double offset = 0;

    public Gyroscope() {
        navx = new AHRS(SPI.Port.kMXP, (byte) 200);
    }

    /**
     * @return Rotation2d representing the angle the robot is facing
     */
    public Rotation2d getGyroscopeRotation() {
        if (navx.isMagnetometerCalibrated()) {
            // We will only get valid fused headings if the magnetometer is calibrated
            return Rotation2d.fromDegrees(navx.getFusedHeading());
        }

        // We have to invert the angle of the NavX so that rotating the robot
        // counter-clockwise makes the angle increase.
        return Rotation2d.fromDegrees(360.0 - navx.getYaw() + offset); // Add offset to make the shooter the front
                                                                       // instead of the intake
    }

    public void zeroYaw() {
        navx.zeroYaw();
    }

    public float getYaw() {
        return navx.getYaw();
    }
}
