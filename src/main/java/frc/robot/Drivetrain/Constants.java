package frc.robot.Drivetrain;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.units.measure.Angle;
import frc.robot.Drivetrain.Constants.ModuleConstants.BackLeft;
import frc.robot.Drivetrain.Constants.ModuleConstants.BackRight;
import frc.robot.Drivetrain.Constants.ModuleConstants.FrontLeft;
import frc.robot.Drivetrain.Constants.ModuleConstants.FrontRight;

public class Constants {
    public static final double DriveGearRatio = 5.5;
    public static final double SteerGearRatio = 9424/203;
    public static final double WheelCirc = Inches.of(3).times(Math.PI).in(Meters);
    public static final int SlipCurrent = 40; //可以用pathplanner算
    public static final PathConstraints constraints = new PathConstraints(4, 4, 4, 4); //TODO: 要記得改
    public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(FrontLeft.place, FrontRight.place, BackLeft.place, BackRight.place);
    public static final Pose2d InitialPose = new Pose2d(7.6,7, Rotation2d.kZero);
    
    public class ModuleConstants {
        public static final double OffsetValue = Centimeters.of(65).div(2).in(Meters);

        public class FrontLeft {
            public static final int DriveID = 11;
            public static final int SteerID = 12;
            public static final Angle offset = Rotations.of(0);
            public static final boolean DriveInverted = false;
            public static final Translation2d place = new Translation2d(OffsetValue, -OffsetValue);
        }

        public class FrontRight {
            public static final int DriveID = 21;
            public static final int SteerID = 22;
            public static final Angle offset = Rotations.of(0);
            public static final boolean DriveInverted = true;
            public static final Translation2d place = new Translation2d(OffsetValue, OffsetValue);
        }

        public class BackLeft {
            public static final int DriveID = 31;
            public static final int SteerID = 32;
            public static final Angle offset = Rotations.of(0);
            public static final boolean DriveInverted = false;
            public static final Translation2d place = new Translation2d(-OffsetValue, -OffsetValue);
        }

        public class BackRight {
            public static final int DriveID = 41;
            public static final int SteerID = 42;
            public static final Angle offset = Rotations.of(0);
            public static final boolean DriveInverted = true;
            public static final Translation2d place = new Translation2d(-OffsetValue, OffsetValue);
        }

        
    }
}
