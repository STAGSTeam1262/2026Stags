package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
    }

    public static final class DriveConstants {
        public static final double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
        public static final  double MaxAngularRate = RotationsPerSecond.of(1.6).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    }

}
