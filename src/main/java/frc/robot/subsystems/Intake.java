package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.techhounds.houndutil.houndlib.subsystems.BaseIntake;
import com.techhounds.houndutil.houndlog.annotations.Log;
import com.techhounds.houndutil.houndlog.annotations.LoggedObject;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Elevator.ElevatorPosition;
import frc.robot.Constants.Intake.IntakePosition;

import static frc.robot.Constants.Intake.*;

import java.util.function.Supplier;

@LoggedObject
public class Intake extends SubsystemBase implements BaseIntake {
    @Log
    private final SparkMax armL_motor;
    @Log
    private final SparkMax armR_motor;
    @Log
    private final SparkMax intakeL_motor;
    @Log
    private final SparkMax intakeC_motor;
    @Log
    private final SparkMax intakeR_motor;

    SparkMaxConfig config = new SparkMaxConfig();
    

    public Intake() {
        config.closedLoop
            .p(0.2)
            .i(0)
            .d(0)
            .outputRange(-0.25, 0.25);
        armL_motor = createMotor(ARML_MOTOR_INVERTED, ARML_CURRENT_LIMIT, ARML_MOTOR_ID);
        armR_motor = createMotor(ARMR_MOTOR_INVERTED, ARMR_CURRENT_LIMIT, ARMR_MOTOR_ID);
        intakeL_motor = createMotor(INTAKEL_MOTOR_INVERTED, 40, INTAKEL_MOTOR_ID);
        intakeC_motor = createMotor(INTAKEC_MOTOR_INVERTED, INTAKEC_CURRENT_LIMIT, INTAKEC_MOTOR_ID);
        intakeR_motor = createMotor(false,40, INTAKER_MOTOR_ID);
        intakeL_motor.configure(config, null, PersistMode.kPersistParameters);
        intakeR_motor.configure(config, null, PersistMode.kPersistParameters);
        
        
    }

    public void setIntakeVoltage(double voltage) {
        intakeL_motor.setVoltage(voltage);
        intakeR_motor.setVoltage(voltage);
    }

    public void setShootVoltage(double voltage) {
        intakeC_motor.setVoltage(voltage);
    }

    public Command deployIntakes() {
        return Commands.parallel(
            Commands.runOnce(() -> {
                armL_motor.getClosedLoopController().setReference(IntakePosition.OUT.value, ControlType.kPosition);
                SmartDashboard.putNumber("intakemotarpos", intakeL_motor.getAbsoluteEncoder().getPosition());
            }),
            Commands.runOnce(() -> armR_motor.getClosedLoopController().setReference(IntakePosition.OUT.value, ControlType.kPosition)))
            .withName("intake.deployIntakes");
    }

    public Command retractIntakes() {
        return Commands.parallel(
            Commands.runOnce(() -> armL_motor.getClosedLoopController().setReference(IntakePosition.HOME.value, ControlType.kPosition)),
            Commands.runOnce(() -> armR_motor.getClosedLoopController().setReference(IntakePosition.HOME.value, ControlType.kPosition)))
            .withName("intake.retractIntakes");
    }
 
    public Command shootCoral() {
        return Commands.startEnd(
                () -> setShootVoltage(3),
                () -> setShootVoltage(0))
                .withName("intake.shootCoral");
    }

    public Command shootAlgae() {
        return Commands.startEnd(
                () -> setShootVoltage(-3),
                () -> setShootVoltage(0))
                .withName("intake.shootAlgae");
    }

    public Command shootAuto(Supplier<ElevatorPosition> supp) {
        var position = supp.get();
        switch (position) {
            case ALGAE:
            case PROCESSOR:
                return shootAlgae().withTimeout(0.5);
            case L1:
            case L2:
            case L3:
            case L4:
                return shootCoral().withTimeout(0.5);
            default:
                return Commands.none();
        }
    }

    private SparkMax createMotor(Boolean inverted, Integer currentLimit, Integer devID) {
        var motorConfig = new SparkMaxConfig();
        motorConfig
                .inverted(inverted)
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(currentLimit);

        var motor = new SparkMax(devID, MotorType.kBrushless);
        motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        return motor;
    }

    @Override
    public Command runRollersCommand() {
        return Commands.startEnd(
                () -> setIntakeVoltage(-12),
                () -> setIntakeVoltage(0))
                .withName("intake.runIntakes");
    }

    @Override
    public Command reverseRollersCommand() {
        return Commands.startEnd(
                () -> setIntakeVoltage(12),
                () -> setIntakeVoltage(0))
                .withName("intake.reverseIntakes");
    }
}
