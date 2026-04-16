package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import frc.robot.generated.TunerConstants;
import frc.robot.util.Controller;

public class Constants {
    
    public static final class OperatorConstants {
        public static final Controller driverController = new Controller(0);
        public static final Controller operatorController = new Controller(1);
    }

    public static final class FieldConstants {
        public static final Pose2d blueHubCenter = new Pose2d(4.625, 4.025, Rotation2d.kZero);
        public static final Pose2d redHubCenter = new Pose2d(11.925, 4.025, Rotation2d.kZero);

        public static final Pose2d blueLeft = new Pose2d(2.085, 6.111, Rotation2d.kZero);
        public static final Pose2d blueRight = new Pose2d(2.085, 2.035, Rotation2d.kZero);
        public static final Pose2d redLeft = new Pose2d(14.51, 2.035, Rotation2d.kZero);
        public static final Pose2d redRight = new Pose2d(14.51, 6.111, Rotation2d.kZero);
    }

    public static final class CANConstants {
        public static final CANBus rio = CANBus.roboRIO();
        public static final CANBus canivore = new CANBus("omega");

        public static final int IntakeRollerID = 11;
        public static final int IntakeDeployID = 12;
        
        public static final int FeederID = 13;
        public static final int IndexerID = 14;
        public static final int ShooterID = 15;
    }

    public static final class DriveConstants {
        public static final double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
        public static final double MaxAngularRate = RotationsPerSecond.of(1.6).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    }

    public static final class ShooterConstants {
        // Used to test rollers when needing very limited speed. Will be a separate binding from normal shooting (maybe even a separate controller slot)
        public static double debugFeederSpeed = 0.0;
        public static double debugIndexerSpeed = 0.0;
        public static double debugShooterSpeed = 0.0;

        public static double feederSpeed = -12.0;
        public static double indexerSpeed = 12.0;

        // Will be switched to when vision is down for any reason.
        public static double fixedShooterSpeed = 100.0;

        public static final InterpolatingDoubleTreeMap shotSpeedFromDistance = new InterpolatingDoubleTreeMap();

        static {

        }
    }

    public static final class VisionConstants {
        public static final Transform3d tag1Position = new Transform3d(-0.32, -0.095, 0.38, new Rotation3d(0, Math.toRadians(-15), Math.toRadians(170)));
        public static final Transform3d tag3Position = new Transform3d(-0.32, 0.095, 0.38, new Rotation3d(0, Math.toRadians(-15), Math.toRadians(-170)));
    }

}
