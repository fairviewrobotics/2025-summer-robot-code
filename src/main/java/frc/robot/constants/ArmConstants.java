package frc.robot.constants;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class ArmConstants {
    public static final int ARM_MOTOR_ID = 14;
    public static final double ARM_P = 0.3;
    public static final double ARM_I = 0;
    public static final double ARM_D = 0;
    public static final double ARM_TOLERANCE = 0.1;
    public static final double ARM_MAX_ANGLE = Math.PI/2;
    public static final double ARM_MIN_ANGLE = -Math.PI/2;
    public static final double ARM_MAX_VELOCITY = 4 * Math.PI;
    public static final double ARM_MAX_ACCELERATION = 4 * Math.PI;
    public static final TrapezoidProfile.Constraints ARM_CONSTRAINTS = new TrapezoidProfile.Constraints(ARM_MAX_VELOCITY, ARM_MAX_ACCELERATION);

    // Radians
    public static final double INTAKE_POSITION = 0.4;
    public static final double DEFAULT_POSITION = 1.3;

}
