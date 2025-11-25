package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;



public class MathUtils {

    public static double RPMtoRadians(double rpm) {
        return rpm * (2 * Math.PI / 60);
    }

    public static final Pose2d zeroPose = new Pose2d(0, 0, new Rotation2d(0));
    public static final Translation2d zeroTranslation = new Translation2d(0.0, 0.0);

    public static final Pose2d getPoseFromRotation(Rotation2d rotation) {return new Pose2d(zeroTranslation, rotation);}
    public static final Pose2d getPoseFromTranslation(Translation2d translation) {return new Pose2d(translation, Rotation2d.fromDegrees(0));}
    public static final Transform2d getTransform2dFromTranslation(Translation2d translation) {return new Transform2d(translation, Rotation2d.kZero);}

}


