package frc.robot.utils;

import com.ctre.phoenix6.swerve.jni.SwerveJNI;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

public class SwerveDriveOdometryMeasurement {
    private final Rotation2d gyroAngle;
    private final SwerveModulePosition[] swerveModulePositions;
    SwerveDriveOdometry odometry;

    public SwerveDriveOdometryMeasurement(Rotation2d gyroAngle, SwerveModulePosition[] swerveModulePositions)
    {
        this.gyroAngle = gyroAngle;
        this.swerveModulePositions = swerveModulePositions;
    }

    public Rotation2d getGyroAngle() {
        return gyroAngle;
    }

    public SwerveModulePosition[] getSwerveModulePositions() {
        return swerveModulePositions;
    }
}
