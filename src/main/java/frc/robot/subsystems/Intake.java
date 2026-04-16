package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;

public class Intake extends SubsystemBase {

    Superstructure superstructure;

    TalonFX intakeRoller = new TalonFX(Constants.CANConstants.IntakeRollerID, Constants.CANConstants.rio);
    VelocityVoltage velocityControl = new VelocityVoltage(0).withSlot(0);
    DoublePublisher intakeRollerSpeedPublisher = NetworkTableInstance.getDefault().getDoubleTopic("Subsystems/Intake/Intake Roller Voltage").publish();

    TalonFX intakeDeploy = new TalonFX(Constants.CANConstants.IntakeDeployID, Constants.CANConstants.rio);
    PositionVoltage positionControl = new PositionVoltage(0).withSlot(0);
    public Trigger hardStop = new Trigger(() -> intakeDeploy.getSupplyCurrent().getValueAsDouble() >= 20).debounce(0.3);
    DoublePublisher intakeDeployCurrent = NetworkTableInstance.getDefault().getDoubleTopic("Subsystems/Intake/Deploy Current").publish();
    BooleanPublisher intakeHardStopPublisher = NetworkTableInstance.getDefault().getBooleanTopic("Subsystems/Intake/Deploy Hard Stop").publish();


    public Intake() {
        configureMotors();
        hardStop.onTrue(Commands.runOnce(() -> intakeDeploy.setVoltage(0)).asProxy());
    }

    public void configureMotors() {
        MotorOutputConfigs deployOutputConfigs = new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast);
        Slot0Configs deploySlotConfigs = new Slot0Configs()
                .withKP(3)
                .withKI(0)
                .withKD(0)
                .withKV(0.124);
        TalonFXConfiguration deployConfig = new TalonFXConfiguration().withMotorOutput(deployOutputConfigs).withSlot0(deploySlotConfigs).withCurrentLimits(new CurrentLimitsConfigs().withSupplyCurrentLimit(30).withSupplyCurrentLimitEnable(true));
        intakeDeploy.getConfigurator().apply(deployConfig);

        MotorOutputConfigs rollerOutputConfigs = new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast);
        Slot0Configs rollerSlotConfigs = new Slot0Configs()
                .withKP(0.1)
                .withKI(0)
                .withKD(0)
                .withKV(0.124);
        TalonFXConfiguration rollerConfig = new TalonFXConfiguration().withMotorOutput(rollerOutputConfigs);
        intakeDeploy.getConfigurator().apply(rollerConfig);
    }

    public void runRoller(double velocity) {
        intakeRoller.setVoltage(velocity);
    }

    public void runDeploy(double voltage) {
        intakeDeploy.setVoltage(voltage);
    }

    public void moveDeployToPosition(double position) {
        intakeDeploy.setControl(positionControl.withPosition(position));
    }

    public void stop() {
        intakeDeploy.stopMotor();
        intakeRoller.stopMotor();
    }

    public void provideSubsystemAccessToSuperstructure(Superstructure superstructure) {
        this.superstructure = superstructure;
    }

    @Override
    public void periodic() {
        intakeRollerSpeedPublisher.set(intakeRoller.getMotorVoltage().getValueAsDouble());
        
        intakeDeployCurrent.set(intakeDeploy.getSupplyCurrent().getValueAsDouble());
        intakeHardStopPublisher.set(hardStop.getAsBoolean());
    }

}