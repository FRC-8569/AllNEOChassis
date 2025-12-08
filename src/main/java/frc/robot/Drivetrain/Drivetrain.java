package frc.robot.Drivetrain;
import java.util.List;
import java.util.stream.IntStream;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import dev.doglog.DogLog;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Drivetrain.Constants.ModuleConstants.BackLeft;
import frc.robot.Drivetrain.Constants.ModuleConstants.BackRight;
import frc.robot.Drivetrain.Constants.ModuleConstants.FrontLeft;
import frc.robot.Drivetrain.Constants.ModuleConstants.FrontRight;

public class Drivetrain extends DogLog implements Subsystem{
    // 向量模組的列表，用於方便地處理所有模組的資料
    public List<SwerveMod> modules;
    // 陀螺儀，用於獲取機器人的旋轉角度
    public AHRS gyro;
    // 向量驅動位置估算器，用於追蹤機器人的位置
    public SwerveDrivePoseEstimator PoseEstimator;

    private static Drivetrain drivetrain;

    private Drivetrain(){
        // 初始化向量模組，分別對應四個輪子
        modules = List.of(
            new SwerveMod(
                1, 
                FrontLeft.DriveID, 
                FrontLeft.SteerID, 
                FrontLeft.offset, 
                FrontLeft.DriveInverted),

            new SwerveMod(
                2, 
                FrontRight.DriveID, 
                FrontRight.SteerID, 
                FrontRight.offset, 
                FrontRight.DriveInverted),

            new SwerveMod(
                3, 
                BackLeft.DriveID, 
                BackLeft.SteerID, 
                BackLeft.offset, 
                BackLeft.DriveInverted),

            new SwerveMod(
                4, 
                BackRight.DriveID, 
                BackRight.SteerID, 
                BackRight.offset, 
                BackRight.DriveInverted)
        );

        // 初始化陀螺儀，使用 NavX 的 SPI 通訊
        gyro = new AHRS(NavXComType.kMXP_SPI);
        // 初始化位置估算器，使用機器人運動學、陀螺儀角度和模組位置
        PoseEstimator = new SwerveDrivePoseEstimator(Constants.kinematics, gyro.getRotation2d(), getPositions(), Constants.InitialPose);

    }
    /**
     * 獲取所有向量模組的位置。
     * 
     * @return 一個包含所有模組位置的 SwerveModulePosition 陣列。
     */
    public SwerveModulePosition[] getPositions(){
        return modules.stream().map(SwerveMod::getPosition).toArray(SwerveModulePosition[]::new); //一次抓所有模組的位置
    }

    /**
     * 獲取所有向量模組的狀態。
     * 
     * @return 一個包含所有模組狀態的 SwerveModuleState 陣列。
     */
    public SwerveModuleState[] getStates(){
        return modules.stream().map(SwerveMod::getState).toArray(SwerveModuleState[]::new);
    }

    /**
     * 創建一個命令來根據指定的底盤速度驅動機器人。
     * 
     * @param speeds 底盤的目標速度。
     * @param isFieldRelative 是否使用場地相對速度。
     * @param isOpenLoop 是否使用開環控制。
     * @return 一個執行驅動操作的 Command 物件。
     */
    public Command drive(ChassisSpeeds speeds, boolean isFieldRelative, boolean isOpenLoop) {
        return run(() -> 
            // 如果是場地相對速度，將機器人相對速度轉換為場地相對速度
            setState(Constants.kinematics.toWheelSpeeds(
                isFieldRelative 
                    ? ChassisSpeeds.fromRobotRelativeSpeeds(
                        speeds, // 機器人相對速度
                        gyro.getRotation2d() // 機器人當前的旋轉角度
                    ) 
                    : speeds // 如果不是場地相對速度，直接使用提供的速度
            ), isOpenLoop) // 設定向量模組的狀態，並指定是否使用開環控制
        );
    }

    /**
     * 設定所有向量模組的狀態。
     * 
     * @param states 一個包含目標狀態的 SwerveModuleState 陣列。
     * @param isOpenLoop 是否使用開環控制。
     */
    public void setState(SwerveModuleState[] states, boolean isOpenLoop){
        IntStream.range(0, 4).forEachOrdered(i -> modules.get(i).setState(states[i], RobotBase.isSimulation() || isOpenLoop));
        log("Drivetrain/TargetStates", states);
        log("Drivetrain/TargetSpeeds", Constants.kinematics.toChassisSpeeds(states));
    }

    @Override
    public void periodic(){
        PoseEstimator.update(gyro.getRotation2d(), getPositions());
        modules.stream().forEach(SwerveMod::logData);
        logData();
    }

    public void logData(){
        log("Drivetrain/CurrentStates", getStates());
        log("Drivetrain/CurrentSpeeds", Constants.kinematics.toChassisSpeeds(getStates()));
    }

    public static Drivetrain getInstance(){
        drivetrain = drivetrain != null ? drivetrain : new Drivetrain();
        return drivetrain;
    }
}
