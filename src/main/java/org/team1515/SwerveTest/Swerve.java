package org.team1515.SwerveTest;

import com.team364.swervelib.util.SwerveModule;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import com.team364.swervelib.util.SwerveConstants;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swerve extends SubsystemBase {
    private SwerveDriveOdometry swerveOdometry;
    private SwerveModule[] mSwerveMods;
    private SwerveDrivePoseEstimator poseEstimator;
    private Pose2d initialPos = new Pose2d(0.0, 0.0, new Rotation2d(0.0));
    private Rotation2d realZero;

    public Swerve(Pose2d initiallPose) {
        

        realZero = initialPos.getRotation();

        zeroGyro();
        SmartDashboard.putData("gyro", RobotContainer.gyro.navx);

        mSwerveMods = new SwerveModule[] {
                new SwerveModule(0, SwerveConstants.Swerve.Mod0.constants),
                new SwerveModule(1, SwerveConstants.Swerve.Mod1.constants),
                new SwerveModule(2, SwerveConstants.Swerve.Mod2.constants),
                new SwerveModule(3, SwerveConstants.Swerve.Mod3.constants)
        };

        poseEstimator = new SwerveDrivePoseEstimator(SwerveConstants.Swerve.swerveKinematics, Rotation2d.fromRadians(0), getModulePositions(), initialPos);

        swerveOdometry = new SwerveDriveOdometry(SwerveConstants.Swerve.swerveKinematics, getYaw(),
                getModulePositions());
    }

    public void drive(Translation2d translation, double rotation, boolean robotRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates = SwerveConstants.Swerve.swerveKinematics.toSwerveModuleStates(
                robotRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        translation.getX(),
                        translation.getY(),
                        rotation,
                        getYaw())
                        : new ChassisSpeeds(
                        translation.getX(),
                        translation.getY(),
                        rotation));
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveConstants.Swerve.maxSpeed);

        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveConstants.Swerve.maxSpeed);

        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }



    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule mod : mSwerveMods) {
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (SwerveModule mod : mSwerveMods) {
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public void zeroGyro() {

        realZero = realZero.minus(RobotContainer.gyro.getGyroscopeRotation());
        RobotContainer.gyro.zeroYaw();
    }

    public Rotation2d getYaw() {
        return (SwerveConstants.Swerve.invertGyro) ? Rotation2d.fromDegrees(360 - RobotContainer.gyro.getYaw())
                : Rotation2d.fromDegrees(RobotContainer.gyro.getYaw());
    }

    public void resetModulesToAbsolute() {
        for (SwerveModule mod : mSwerveMods) {
            mod.resetToAbsolute();
        }
    }

    public Pose2d getPose(){
        return poseEstimator.getEstimatedPosition();
    }

    public void updateOdometry() {
        poseEstimator.update(
                RobotContainer.gyro.getGyroscopeRotation(), getModulePositions());

        // Also apply vision measurements. We use 0.3 seconds in the past as an example
        // -- on
        // a real robot, this must be calculated based either on latency or timestamps.
        Optional<EstimatedRobotPose> result =
                RobotContainer.pvw.getEstimatedGlobalPose(poseEstimator.getEstimatedPosition());

        if (result.isPresent()) {
            EstimatedRobotPose camPose = result.get();
            poseEstimator.addVisionMeasurement(
                    camPose.estimatedPose.toPose2d(), camPose.timestampSeconds);
        }
    }

    @Override
    public void periodic() {
        updateOdometry();
        SmartDashboard.putNumber("gyro angle", RobotContainer.gyro.getGyroscopeRotation().getDegrees());
        if (DriverStation.isDisabled()) {
            resetModulesToAbsolute();
        }

        for (SwerveModule mod : mSwerveMods) {
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
        }
    }

    public Rotation2d getRealZero() {
        return realZero;
    }
}