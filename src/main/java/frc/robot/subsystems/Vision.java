package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.util.Elastic;
import frc.robot.util.Elastic.Notification;
import frc.robot.util.Elastic.NotificationLevel;

public class Vision extends SubsystemBase {

    Superstructure superstructure;
    Drivetrain drivetrain;

    PhotonCamera tagCamera1 = new PhotonCamera("Tag-01");
    PhotonCamera tagCamera2 = new PhotonCamera("Tag-03");
    PhotonCamera tagCamera3 = new PhotonCamera("Tag-04");

    Optional<EstimatedRobotPose> tagCam1VisionEst;
    Optional<EstimatedRobotPose> tagCam2VisionEst;
    Optional<EstimatedRobotPose> tagCam3VisionEst;

    AprilTagFieldLayout layout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    public Trigger tag1Connected = new Trigger(() -> tagCamera1.isConnected());
    public Trigger tag2Connected = new Trigger(() -> tagCamera2.isConnected());
    public Trigger tag3Connected = new Trigger(() -> tagCamera3.isConnected());

    PhotonPoseEstimator tag1PhotonPoseEstimator = new PhotonPoseEstimator(layout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, Constants.VisionConstants.tag3Position);
    PhotonPoseEstimator tag2PhotonPoseEstimator = new PhotonPoseEstimator(layout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, new Transform3d(-0.32, 0.3, 0.38, new Rotation3d(0, Math.toDegrees(-15), 0)));
    PhotonPoseEstimator tag3PhotonPoseEstimator = new PhotonPoseEstimator(layout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, Constants.VisionConstants.tag1Position);

    StructPublisher<Pose2d> tag1PosePublisher = NetworkTableInstance.getDefault().getStructTopic("Subsystems/Vision/Tag 1 Pose", Pose2d.struct).publish();
    StructPublisher<Pose2d> tag2PosePublisher = NetworkTableInstance.getDefault().getStructTopic("Subsystems/Vision/Tag 2 Pose", Pose2d.struct).publish();
    StructPublisher<Pose2d> tag3PosePublisher = NetworkTableInstance.getDefault().getStructTopic("Subsystems/Vision/Tag 3 Pose", Pose2d.struct).publish();

    StructPublisher<Transform3d> tag1TranslationPublisher = NetworkTableInstance.getDefault().getStructTopic("Subsystems/Vision/Tag 1 Translation", Transform3d.struct).publish();
    StructPublisher<Transform3d> tag2TranslationPublisher = NetworkTableInstance.getDefault().getStructTopic("Subsystems/Vision/Tag 2 Translation", Transform3d.struct).publish();
    StructPublisher<Transform3d> tag3TranslationPublisher = NetworkTableInstance.getDefault().getStructTopic("Subsystems/Vision/Tag 2 Translation", Transform3d.struct).publish();

    BooleanPublisher tag1ConnectedPublisher = NetworkTableInstance.getDefault().getBooleanTopic("Subsystems/Vision/Tag 1 Connected").publish();
    BooleanPublisher tag2ConnectedPublisher = NetworkTableInstance.getDefault().getBooleanTopic("Subsystems/Vision/Tag 2 Connected").publish();
    BooleanPublisher tag3ConnectedPublisher = NetworkTableInstance.getDefault().getBooleanTopic("Subsystems/Vision/Tag 3 Connected").publish();

    public Vision(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;

        tag1TranslationPublisher.set(Constants.VisionConstants.tag1Position);
        tag2TranslationPublisher.set(Constants.VisionConstants.tag3Position);

        tag1Connected
            .onTrue(Commands.runOnce(() -> 
                Elastic.sendNotification(new Notification(NotificationLevel.INFO, "[Vision] Camera Status", "Tag Camera 1 Connected. This camera can send vision updates."))))
            .onFalse(Commands.runOnce(() -> 
                Elastic.sendNotification(new Notification(NotificationLevel.WARNING, "[Vision] Camera Status", "Tag Camera 1 Disconnected. No vision updates will be sent from this camera."))));
        tag2Connected
            .onTrue(Commands.runOnce(() -> 
                Elastic.sendNotification(new Notification(NotificationLevel.INFO, "[Vision] Camera Status", "Tag Camera 2 Connected. This camera can send vision updates."))))
            .onFalse(Commands.runOnce(() -> 
                Elastic.sendNotification(new Notification(NotificationLevel.WARNING, "[Vision] Camera Status", "Tag Camera 2 Disconnected. No vision updates will be sent from this camera."))));
        tag3Connected
            .onTrue(Commands.runOnce(() -> 
                Elastic.sendNotification(new Notification(NotificationLevel.INFO, "[Vision] Camera Status", "Tag Camera 3 Connected. This camera can send vision updates."))))
            .onFalse(Commands.runOnce(() -> 
                Elastic.sendNotification(new Notification(NotificationLevel.WARNING, "[Vision] Camera Status", "Tag Camera 3 Disconnected. No vision updates will be sent from this camera."))));
    }

    public void provideSubsystemAccessToSuperstructure(Superstructure superstructure) {
        this.superstructure = superstructure;
    }

    @Override
    public void periodic() {
        tag1ConnectedPublisher.set(tag1Connected.getAsBoolean());
        tag2ConnectedPublisher.set(tag2Connected.getAsBoolean());
        tag3ConnectedPublisher.set(tag3Connected.getAsBoolean());

        double omegaRps = Units.radiansToRotations(drivetrain.getState().Speeds.omegaRadiansPerSecond);
        tag1PhotonPoseEstimator.addHeadingData(drivetrain.getState().Timestamp, drivetrain.getState().Pose.getRotation());
        tag2PhotonPoseEstimator.addHeadingData(drivetrain.getState().Timestamp, drivetrain.getState().Pose.getRotation());
        tag3PhotonPoseEstimator.addHeadingData(drivetrain.getState().Timestamp, drivetrain.getState().Pose.getRotation());
        
        if (tag1Connected.getAsBoolean() && Math.abs(omegaRps) < 2.0) {
            var tag1Change = tagCamera1.getLatestResult();
            tagCam1VisionEst = tag1PhotonPoseEstimator.update(tag1Change);
            if (tagCam1VisionEst.isPresent() && tag1Change.hasTargets() && tag1Change.getBestTarget().getPoseAmbiguity() < 0.2) {
                drivetrain.addVisionMeasurement(tagCam1VisionEst.get().estimatedPose.toPose2d(), tagCam1VisionEst.get().timestampSeconds, VecBuilder.fill(0.0, 0.0, 9999999));
                tag1PosePublisher.set(tagCam1VisionEst.get().estimatedPose.toPose2d());
            }
        }
        if (tag2Connected.getAsBoolean() && Math.abs(omegaRps) < 2.0) {
            var tag2Change = tagCamera2.getLatestResult();
            tagCam2VisionEst = tag2PhotonPoseEstimator.update(tag2Change);
            if (tagCam2VisionEst.isPresent() && tag2Change.hasTargets() && tag2Change.getBestTarget().getPoseAmbiguity() < 0.2) {
                drivetrain.addVisionMeasurement(tagCam2VisionEst.get().estimatedPose.toPose2d(), tagCam2VisionEst.get().timestampSeconds, VecBuilder.fill(0.0, 0.0, 9999999));
                tag2PosePublisher.set(tagCam2VisionEst.get().estimatedPose.toPose2d());
            }
        }
        if (tag3Connected.getAsBoolean() && Math.abs(omegaRps) < 2.0) {
            var tag3Change = tagCamera3.getLatestResult();
            tagCam3VisionEst = tag3PhotonPoseEstimator.update(tag3Change);
            if (tagCam3VisionEst.isPresent() && tag3Change.hasTargets() && tag3Change.getBestTarget().getPoseAmbiguity() < 0.2) {
                drivetrain.addVisionMeasurement(tagCam3VisionEst.get().estimatedPose.toPose2d(), tagCam3VisionEst.get().timestampSeconds, VecBuilder.fill(0.0, 0.0, 9999999));
                tag3PosePublisher.set(tagCam3VisionEst.get().estimatedPose.toPose2d());
            }
        }
    }

}
