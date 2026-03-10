package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Turret extends SubsystemBase {

    Superstructure superstructure;
    
    public TurretState state = TurretState.IDLE;
    StringPublisher statePublisher = NetworkTableInstance.getDefault().getStringTopic("Subsystems/Turret/State").publish();

    public double targetAngle = 0;
    DoublePublisher targetAnglePublisher = NetworkTableInstance.getDefault().getDoubleTopic("Subsystems/Turret/Target Angle").publish();
    StructPublisher<Pose3d> targetAngleSim = NetworkTableInstance.getDefault().getStructTopic("Subsystems/Turret/Target Angle Sim", Pose3d.struct).publish();

    public Turret() {
        configureMotors();
    }

    public enum TurretState {
        IDLE,
        FIXED,
        TRACKING_TARGET
    }

    public void configureMotors() {

    }

    public double calculateTargetAngle(Rotation2d angle) {
        Rotation2d targetAngle = angle;
        targetAngle = targetAngle.minus(superstructure.getRobotPose().getRotation());
        targetAngleSim.set(new Pose3d(superstructure.getRobotPose().getX(), superstructure.getRobotPose().getY(), 0.5588, new Rotation3d(targetAngle.plus(superstructure.getRobotPose().getRotation()))));
        targetAnglePublisher.set(targetAngle.getDegrees());
        return targetAngle.getDegrees();
    }

    public void moveToAngle(double angle) {
        // Tell motor to move to position.
    }

    public void fixTurretToAngle(double angle) {
        moveToAngle(angle);
    }

    public void startTrackingTarget() {
        
    }

    public boolean atTargetAngle() {
        return MathUtil.isNear(targetAngle, 0, 2);
    }

    public void stop() {

    }

    public void provideSubsystemAccessToSuperstructure(Superstructure superstructure) {
        this.superstructure = superstructure;
    }

    @Override
    public void periodic() {
        if (superstructure.shotData != null) {
            double angle = calculateTargetAngle(superstructure.shotData.shotAngle());
            if (state == TurretState.TRACKING_TARGET) {
                moveToAngle(angle);
            }
        }
    }

}