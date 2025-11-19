package frc.robot.constants;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class ArmConstants {
    public static final int armMotorID = 39;
    public static final double armP = 0;
    public static final double armI = 0;
    public static final double armD = 0;
    public static final double armTolerance = 0;
    public static final double armMaxAngle = Math.PI/2;
    public static final double armMinAngle = 0;
    public static final double armMaxVelocity = 4 * Math.PI;
    public static final double armMaxAcceleration = 4 * Math.PI;
    public static final TrapezoidProfile.Constraints armConstraints = new TrapezoidProfile.Constraints(armMaxVelocity, armMaxAcceleration);
}
