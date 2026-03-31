package frc.robot.subsystems;

import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
    
    Superstructure superstructure;

    TalonFX feeder = new TalonFX(Constants.CANConstants.FeederID, Constants.CANConstants.rio);
    double targetFeederVoltage = 0.0;
    DoublePublisher feederSpeedPublisher = NetworkTableInstance.getDefault().getDoubleTopic("Subsystems/Feeder/Speed").publish();
    DoublePublisher feederVoltagePublisher = NetworkTableInstance.getDefault().getDoubleTopic("Subsystems/Feeder/Voltage").publish();
    DoublePublisher feederTargetPublisher = NetworkTableInstance.getDefault().getDoubleTopic("Subsystems/Feeder/Target Voltage").publish();

    TalonFX indexer = new TalonFX(Constants.CANConstants.IndexerID, Constants.CANConstants.rio);
    double targetIndexerVoltage = 0.0;
    DoublePublisher indexerSpeedPublisher = NetworkTableInstance.getDefault().getDoubleTopic("Subsystems/Indexer/Speed").publish();
    DoublePublisher indexerVoltagePublisher = NetworkTableInstance.getDefault().getDoubleTopic("Subsystems/Indexer/Voltage").publish();
    DoublePublisher indexerTargetPublisher = NetworkTableInstance.getDefault().getDoubleTopic("Subsystems/Indexer/Target Voltage").publish();

    TalonFX shooter = new TalonFX(Constants.CANConstants.ShooterID, Constants.CANConstants.rio);
    VelocityVoltage velocityControl = new VelocityVoltage(0).withSlot(0);
    double targetShooterVoltage = 0.0;
    DoublePublisher shooterSpeedPublisher = NetworkTableInstance.getDefault().getDoubleTopic("Subsystems/Shooter/Speed").publish();
    DoublePublisher shooterVoltagePublisher = NetworkTableInstance.getDefault().getDoubleTopic("Subsystems/Shooter/Voltage").publish();
    DoublePublisher shooterTargetPublisher = NetworkTableInstance.getDefault().getDoubleTopic("Subsystems/Shooter/Target Voltage").publish();

    public enum State {
        IDLE,
        PREP_SHOT,
        SHOOTING,
        BACKUP,
        DEBUG;
    }

    public State state = State.IDLE;
    StringPublisher statePublisher = NetworkTableInstance.getDefault().getStringTopic("Subsystems/Shooter/Shooter State").publish();

    public Shooter() {
        configureMotors();
    }

    public void setState(State state) {
        this.state = state;
    }

    public double getShotSpeed(double distance) {
        return Constants.ShooterConstants.shotSpeedFromDistance.get(distance);
    }

    public void configureMotors() {
        TalonFXConfiguration config = new TalonFXConfiguration()
            .withSlot0(
                new Slot0Configs()
                .withKP(0.1)
                .withKI(0)
                .withKD(0)
                .withKV(0.124));
        shooter.getConfigurator().apply(config);
        feeder.getConfigurator().apply(config);
        indexer.getConfigurator().apply(config);
    }

    public void stop() {
        feeder.stopMotor();
        indexer.stopMotor();
        shooter.stopMotor();
    }

    public void runFeeder(double voltage) {
        feeder.setVoltage(voltage);
        feederTargetPublisher.set(voltage);
    }

    public void runIndexer(double velocity) {
        indexer.setVoltage(velocity);
        indexerTargetPublisher.set(velocity);
    }

    public void runShooter(double velocity) {
        shooter.setControl(velocityControl.withVelocity(velocity));
        shooterTargetPublisher.set(velocity);
    }

    public void provideSubsystemAccessToSuperstructure(Superstructure superstructure) {
        this.superstructure = superstructure;
    }

    @Override
    public void periodic() {
        feederSpeedPublisher.set(feeder.getVelocity().getValueAsDouble());
        feederVoltagePublisher.set(feeder.getMotorVoltage().getValueAsDouble());

        indexerSpeedPublisher.set(indexer.getVelocity().getValueAsDouble());
        indexerVoltagePublisher.set(indexer.getMotorVoltage().getValueAsDouble());

        shooterSpeedPublisher.set(shooter.getVelocity().getValueAsDouble());
        shooterVoltagePublisher.set(shooter.getMotorVoltage().getValueAsDouble());

        statePublisher.set(state.toString());

        if (state == State.IDLE) {
            runFeeder(0);
            runIndexer(0);
            runShooter(0);
        } else if (state == State.DEBUG) {
            // Debug shoot, run at preset voltage.
            runFeeder(Constants.ShooterConstants.debugFeederSpeed);
            runIndexer(Constants.ShooterConstants.debugIndexerSpeed);
        } else if (state == State.SHOOTING) {
            runFeeder(Constants.ShooterConstants.feederSpeed);
            runIndexer(Constants.ShooterConstants.indexerSpeed);
            // Will change speed based on distance if vision is working, otherwise run at preset voltage.
            if (superstructure.visionIsFine.getAsBoolean() && superstructure.shotData != null) {
                runShooter(Constants.ShooterConstants.fixedShooterSpeed);
            } else {
                runShooter(Constants.ShooterConstants.fixedShooterSpeed);
            }
        } else if (state == State.PREP_SHOT) {
            runFeeder(0);
            runIndexer(0);
            runShooter(100.0);
        } else if (state == State.BACKUP) {
            runFeeder(30);
            runIndexer(-30);
            runShooter(-30);
        }
    }
}
