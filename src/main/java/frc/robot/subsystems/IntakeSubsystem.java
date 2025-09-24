package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IntakeConstants;
import frc.robot.utils.ConfigManager;
import frc.robot.utils.MathUtils;
import frc.robot.utils.NetworkTablesUtils;

public class IntakeSubsystem extends SubsystemBase {
    private final ConfigManager config = ConfigManager.getInstance();
    private final NetworkTablesUtils table = NetworkTablesUtils.getTable("debug");

    //private final SparkFlex armMotor = new SparkFlex((int) config.get("ArmMotorId", IntakeConstants.armMotorId), MotorType.kBrushless);
    //private final RelativeEncoder armEncoder = armMotor.getEncoder();
    //private final SparkFlex clawMotor = new SparkFlex((int) config.get("ClawMotorId", IntakeConstants.clawMotorId), MotorType.kBrushless);
    //private final RelativeEncoder clawEncoder = clawMotor.getEncoder();

    private final PIDController armPidController = new PIDController(
        config.get("ArmP", IntakeConstants.armP),
        config.get("ArmI", IntakeConstants.armI),
        config.get("ArmD", IntakeConstants.armD)
    );

    @Override
    public void periodic() {
        // Update Arm PID, position based
        armPidController.setSetpoint(table.getEntry("ArmSetpoint", 0));
    }

    public IntakeSubsystem() {}

    public void setArmPos(double pos) {
        table.setEntry("ArmSetpoint", pos);
        //armMotor.setVoltage(config.get("ArmMaxVolt", IntakeConstants.armMaxVolt) * armPidController.calculate(MathUtils.RPMtoRadians(armEncoder.getPosition())));
    }

    public void setClawVel(double vel) {
        //clawMotor.set(vel);
    }

    public void toggleArm() {
        if (table.getEntry("ArmSetPoint", 0) == 0) {
            table.setEntry("ArmSetPoint", config.get("ArmTargPos", IntakeConstants.armTargPos));
        } else {
            table.setEntry("ArmSetPoint", config.get("ArmTargPos", 0));
        }
    }

    public void toggleClaw() {
        if (table.getEntry("ClawSetPoint", 0) == 0) {
            //clawMotor.set(config.get("ClawTargVel", IntakeConstants.clawTargVel));
        } else {
            //clawMotor.set(config.get("ClawTargVel", 0));
        }
    }
}
