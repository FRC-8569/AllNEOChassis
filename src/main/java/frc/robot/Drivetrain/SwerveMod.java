package frc.robot.Drivetrain;

import static edu.wpi.first.units.Units.Rotations;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkAbsoluteEncoderSim;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.sim.SparkRelativeEncoderSim;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class SwerveMod extends DogLog {
    public SparkMax DriveMotor, SteerMotor; //開的跟轉的馬達
    public RelativeEncoder DriveEncoder;
    public AbsoluteEncoder SteerEncoder; //轉的編碼器
    public SparkClosedLoopController DrivePID, SteerPID; //硬體PID的呼叫器

    public SparkMaxSim DriveState, SteerState;
    public SparkRelativeEncoderSim driveEncoderState;
    public SparkAbsoluteEncoderSim SteerEncoderState;

    public DCMotorSim DriveSim, SteerSim;

    private SparkMaxConfig DriveConfig, SteerConfig; //設定
    private int moduleID;

    /**
     * 建立一個向量模塊
     * @param ModuleID 模塊的ID(DataLogging用的)
     * @param DriveID  開的馬達的ID
     * @param SteerID  轉的馬達的ID
     * @param offset   編碼器的校正量, 詳情請看<a href="https://docs.revrobotics.com/ion-build/motion/maxplanetary-system">這邊</a> ,不過只要找那一片校正板就好了
     * @param DriveInverted 開的馬達有沒有反轉, 如果有出現左右相反的情況在調整
     */
    public SwerveMod(int ModuleID, int DriveID, int SteerID, Angle offset, boolean DriveInverted){
        DriveMotor = new SparkMax(DriveID, MotorType.kBrushless); //應該不會用有刷馬達吧...
        SteerMotor = new SparkMax(SteerID, MotorType.kBrushless);
        DriveEncoder = DriveMotor.getEncoder();
        SteerEncoder = SteerMotor.getAbsoluteEncoder(); //這邊是抓外接的sensor
        DrivePID = DriveMotor.getClosedLoopController();
        SteerPID = SteerMotor.getClosedLoopController();
        this.moduleID = ModuleID;

        DriveConfig = new SparkMaxConfig();
        SteerConfig = new SparkMaxConfig();

        DriveConfig
            .idleMode(IdleMode.kBrake)
            .inverted(false)
            .voltageCompensation(12)
            .smartCurrentLimit(Constants.SlipCurrent);
        DriveConfig.encoder
            .positionConversionFactor(1/Constants.DriveGearRatio*Constants.WheelCirc) //把rotation轉換成meter
            .velocityConversionFactor(1/Constants.DriveGearRatio*Constants.WheelCirc/60); //把rot/s轉換成m/s
        DriveConfig.closedLoop
            .pidf(0, 0, 0, 1.0/473) //473是馬達的kV,可以在REV官網上翻
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        DriveConfig.closedLoop.maxMotion
            .maxVelocity(1).maxAcceleration(4);

        SteerConfig
            .idleMode(IdleMode.kBrake)
            .inverted(true)
            .voltageCompensation(12)
            .smartCurrentLimit(60);
        SteerConfig.closedLoop
            .pidf(0, 0, 0, 1.0/473)
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
        SteerConfig.absoluteEncoder
            .inverted(true)
            .zeroOffset(offset.in(Rotations));

        
        DriveMotor.configure(DriveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        SteerMotor.configure(SteerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
        
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(
            DriveEncoder.getVelocity(),
            Rotation2d.fromRotations(SteerEncoder.getPosition()) //因為這邊是直接抓外接的sesnor所以不用加其他的處理
        );
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(
            DriveEncoder.getPosition(),
            Rotation2d.fromRotations(SteerEncoder.getPosition())
        );
    }

    public void setState(SwerveModuleState state, boolean isOpenLoop){
        state.optimize(getState().angle); //優化角度, 讓他轉最少的角度到目標位置
            if(!isOpenLoop)DrivePID.setReference(state.speedMetersPerSecond, ControlType.kVelocity);
            else DriveMotor.set((state.speedMetersPerSecond/Constants.constraints.maxVelocityMPS())*(5676*(1/Constants.DriveGearRatio*Constants.WheelCirc))); //沒錯後面那坨就是馬達的最大速度
            SteerPID.setReference(state.angle.getRotations(), ControlType.kPosition);
        if(RobotBase.isSimulation()) DriveState.setAppliedOutput((state.speedMetersPerSecond/Constants.constraints.maxVelocityMPS())*(5676*(1/Constants.DriveGearRatio*Constants.WheelCirc)));
    }

    public void logData(){
        log("Drivetrain/Module/%d/DriveMotorVelocity".formatted(moduleID), DriveEncoder.getVelocity());
        log("Drivetrain/Module/%d/SteerPosition".formatted(moduleID), SteerEncoder.getPosition());
    }


    public double getCurrent(){
        return DriveMotor.getOutputCurrent()+SteerMotor.getOutputCurrent();
    }
}
