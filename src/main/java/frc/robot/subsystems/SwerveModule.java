package frc.robot.subsystems;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import frc.robot.constants.DriveConstants;
import frc.robot.utils.TunableNumber;

import static com.ctre.phoenix6.signals.NeutralModeValue.Brake;
import static com.ctre.phoenix6.signals.NeutralModeValue.Coast;

public class SwerveModule {

    private int moduleNumber;
    private int driveID;
    private int turnID;
    private int encoderID;
    private double xOffset;
    private double yOffset;
    private boolean invertDrive = false;
    private boolean invertTurn = false;
    private double encoderOffset;

    TunableNumber driveP = new TunableNumber("SWERVE_DRIVE_P", 0.0020645);
    TunableNumber driveD = new TunableNumber("SWERVE_DRIVE_D", 0.0);

    TunableNumber turnP = new TunableNumber("SWERVE_TURN_P", 45.0);
    TunableNumber turnD = new TunableNumber("SWERVE_TURN_D", 0.0);

    private TalonFXConfiguration driveConfig = new TalonFXConfiguration();
    private TalonFXConfiguration turnConfig = new TalonFXConfiguration();

    private TalonFX driveMotor;
    private TalonFX turnMotor;
    private CANcoder encoder;

    public SwerveModule(int moduleNumber, int driveID, int turnID, int encoderID, double xOffset, double yOffset, boolean invertDrive, boolean invertTurn, double encoderOffset) {
        this.moduleNumber = moduleNumber;
        this.driveID = driveID;
        this.turnID = turnID;
        this.encoderID = encoderID;
        this.xOffset = xOffset;
        this.yOffset = yOffset;
        this.invertDrive = invertDrive;
        this.invertTurn = invertTurn;
        this.encoderOffset = encoderOffset;

    }

    public void configureModules() {


        driveConfig.MotorOutput.NeutralMode = Coast;
        turnConfig.MotorOutput.NeutralMode = Brake;

        turnConfig.Slot0.kP = turnP.get();
        turnConfig.Slot0.kD = turnD.get();

        //idk what the actual values for this stuff is
        turnConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.05;
        turnConfig.MotorOutput.NeutralMode = Brake;
        turnConfig.CurrentLimits.SupplyCurrentLimit = 30.0;        // limit to 30 amps
        turnConfig.CurrentLimits.SupplyCurrentLowerLimit = 50.0;    // if we exceed 50 amps
        turnConfig.CurrentLimits.SupplyCurrentLowerTime = 1.0;        // for at least 1 second
        turnConfig.CurrentLimits.SupplyCurrentLimitEnable = true;  // and enable it
        turnConfig.CurrentLimits.StatorCurrentLimit = 80.0;        // limit stator to 80 amps
        turnConfig.CurrentLimits.StatorCurrentLimitEnable = true;  // and enable it
        turnConfig.Feedback.FeedbackRemoteSensorID = encoderID;
        turnConfig.Feedback.RotorToSensorRatio = DriveConstants.TURN_GEAR_RATIO;


        driveConfig.Slot0.kP = driveP.get();
        driveConfig.Slot0.kD = driveD.get();
        driveConfig.MotorOutput.NeutralMode = Coast;

        //idk about this either
        driveConfig.CurrentLimits.SupplyCurrentLimit = 60.0;        // limit to 60 amps
        driveConfig.CurrentLimits.SupplyCurrentLowerLimit = 80.0;    // if we exceed 80 amps
        driveConfig.CurrentLimits.SupplyCurrentLowerTime = 2.0;        // for at least 2 second
        driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;  // and enable it
        driveConfig.CurrentLimits.StatorCurrentLimit = 100.0;       // limit stator to 100 amp
        driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;  // and enable it
        driveConfig.Feedback.RotorToSensorRatio = DriveConstants.DRIVE_GEAR_RATIO;

        if (invertDrive) {
            driveConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        } else {
            driveConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        }

        if (invertTurn) {
            turnConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        } else {
            turnConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        }

        driveMotor.getConfigurator().apply(driveConfig);
        turnMotor.getConfigurator().apply(turnConfig);
    }

    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(
                driveMotor.getPosition().getValueAsDouble() * DriveConstants.WHEEL_DIAMETER_INCHES * Math.PI,
                new Rotation2d(this.getModulePose().getRotation().getRadians())
        );
    }

    public Pose2d getModulePose() {
        return new Pose2d(xOffset, yOffset, Rotation2d.kZero);
    }

    public void teleopInit() {
        turnMotor.setControl(new PositionDutyCycle((Angle) turnMotor.getPosition()));
    }

    public void setVelocity(SwerveModuleState desiredState) {
        driveMotor.setControl(driveVelocityVoltage(desiredState.speedMetersPerSecond));
        turnMotor.setControl(turnPositionVoltage(desiredState.angle.getRadians()));
    }

    public void resetDriveEncoder() {
        driveMotor.setPosition(0.0);
    }

    public void resetAbsoluteEncoder() {
        turnMotor.setPosition(0.0);
    }

    private VelocityVoltage driveVelocityVoltage(double speed) {
        return new VelocityVoltage(speed);
    }

    private PositionVoltage turnPositionVoltage (double position) {
        return new PositionVoltage(position);
    }



}


