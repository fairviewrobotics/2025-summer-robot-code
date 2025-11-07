package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.SwerveDriveOdometryMeasurement;
import java.sql.Array;
import java.util.Arrays;
import java.util.HashMap;

public class SwerveLocalizer extends SubsystemBase {

    public static final AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);


    private final int[] tagIDsToTrack = {};

    // Assign time to an ID
    private final HashMap<Integer, Double>IDtoTimeMap = new HashMap<>();

    // SwerveMeasurement:Time
    private final HashMap<SwerveDriveOdometryMeasurement, Double>SwerveMeasurementToTimeMap = new HashMap<>();

    private final double BUFFER_SIZE_SECONDS = 5.0;

    private final double BUFFER_TOLERANCE = 0.1;

    // Vision Buffer
    TimeInterpolatableBuffer buffer = TimeInterpolatableBuffer.createBuffer(BUFFER_SIZE_SECONDS);

    // Chassis Speed Buffer


}
