// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Swerve;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;


/** Add your docs here. */
public class SwerveModule {

    private final TalonFX m_driveMotor;
    private final TalonFX m_steerMotor;
    private final CANcoder m_steerEncoder;

    private final StatusSignal<Double> m_drivePositionStatusSignal;
    private final StatusSignal<Double> m_driveVelocityStatusSignal;
    
    private final StatusSignal<Double> m_steerPositionStatusSignal;
    private final StatusSignal<Double> m_steerVelocityStatusSignal;
    private final BaseStatusSignal[] m_signals;


    private final MotionMagicVoltage m_angleVoltageSetter = new MotionMagicVoltage(0.0);
    private final VelocityVoltage m_velocityVoltageSetter = new VelocityVoltage(0.0);

    private final SwerveModulePosition m_internalState = new SwerveModulePosition();
    public SwerveModuleState m_targetState = new SwerveModuleState();

    public SwerveModule(int driveMotorId, int steerMotorId, int steerEncoderId,
     boolean DriveMotorInverted, boolean SteerMotorInverted, double CANCoderOffset, String canbusName){

        //Motors and Absolute Encoder
        m_driveMotor = new TalonFX(driveMotorId, canbusName);
        m_steerMotor = new TalonFX(steerMotorId, canbusName);
        m_steerEncoder = new CANcoder(steerEncoderId, canbusName);

        //Drive Motor Configs
        TalonFXConfiguration driveConfigs = new TalonFXConfiguration();
        driveConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        driveConfigs.Slot0 = new Slot0Configs();
        driveConfigs.TorqueCurrent.PeakForwardTorqueCurrent = Constants.ModuleConstants.kSlipCurrent;
        driveConfigs.TorqueCurrent.PeakReverseTorqueCurrent = -Constants.ModuleConstants.kSlipCurrent;

        driveConfigs.CurrentLimits.StatorCurrentLimit = Constants.ModuleConstants.kSlipCurrent;
        driveConfigs.CurrentLimits.StatorCurrentLimitEnable = true;


        driveConfigs.MotorOutput.Inverted = DriveMotorInverted ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;
        
        m_driveMotor.getConfigurator().apply(driveConfigs);

        //Steer Motor Configs
        TalonFXConfiguration steerConfigs = new TalonFXConfiguration();
        steerConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        steerConfigs.Slot0 = new Slot0Configs();

        steerConfigs.Feedback.RotorToSensorRatio = Constants.ModuleConstants.kSteerMotorGearRatio;

        steerConfigs.MotionMagic.MotionMagicCruiseVelocity = 100.0 / Constants.ModuleConstants.kSteerMotorGearRatio;
        steerConfigs.MotionMagic.MotionMagicAcceleration = steerConfigs.MotionMagic.MotionMagicCruiseVelocity / 0.100;
        steerConfigs.MotionMagic.MotionMagicExpo_kV = 0.12 * Constants.ModuleConstants.kSteerMotorGearRatio;
        steerConfigs.MotionMagic.MotionMagicExpo_kA = 0.1;

        steerConfigs.ClosedLoopGeneral.ContinuousWrap = true;

        steerConfigs.MotorOutput.Inverted = SteerMotorInverted
        ? InvertedValue.Clockwise_Positive
        : InvertedValue.CounterClockwise_Positive;

        m_steerMotor.getConfigurator().apply(steerConfigs);

        //Cancoder Configs
        CANcoderConfiguration cancoderConfigs = new CANcoderConfiguration();
        cancoderConfigs.MagnetSensor.MagnetOffset = CANCoderOffset; 
        m_steerEncoder.getConfigurator().apply(cancoderConfigs);

        //Inputs
        m_drivePositionStatusSignal = m_driveMotor.getPosition().clone();
        m_driveVelocityStatusSignal = m_driveMotor.getVelocity().clone();
        m_steerPositionStatusSignal = m_steerMotor.getPosition().clone();
        m_steerVelocityStatusSignal = m_steerMotor.getVelocity().clone();

        m_signals = new BaseStatusSignal[4];
        m_signals[0] = m_drivePositionStatusSignal;
        m_signals[1] = m_driveVelocityStatusSignal;
        m_signals[2] = m_steerPositionStatusSignal;
        m_signals[3] = m_steerVelocityStatusSignal;

        // Control Modes
        m_angleVoltageSetter.UpdateFreqHz = 0;
        m_velocityVoltageSetter.UpdateFreqHz = 0;

        reset();


    }

    public void setDesiredState(SwerveModuleState targetState){
        var optimized = SwerveModuleState.optimize(targetState, m_internalState.angle);
        m_targetState = optimized;

        SmartDashboard.putString("State " + m_driveMotor.getDeviceID(), m_targetState.toString());

        double angleToRot = m_internalState.angle.getRotations();
        double velocityToSet = m_targetState.speedMetersPerSecond * Constants.ModuleConstants.kDriveMotorGearRatio / Math.PI * Constants.ModuleConstants.kWheelDiameter;

        m_driveMotor.setControl(m_velocityVoltageSetter.withVelocity(velocityToSet));
        m_steerMotor.setControl(m_angleVoltageSetter.withPosition(angleToRot));
    }

    public SwerveModulePosition getPosition(boolean refresh){
        if(refresh){
            BaseStatusSignal.refreshAll(m_signals);
        }

        double drive_rot = BaseStatusSignal.getLatencyCompensatedValue(m_drivePositionStatusSignal, m_driveVelocityStatusSignal);
        double angle_rot = BaseStatusSignal.getLatencyCompensatedValue(m_steerPositionStatusSignal, m_steerVelocityStatusSignal);
        
        double driveRotToMeter = drive_rot *  Math.PI * Constants.ModuleConstants.kWheelDiameter /  Constants.ModuleConstants.kDriveMotorGearRatio;
        double angleRotToRad = Units.rotationsToRadians(angle_rot);

        m_internalState.distanceMeters = driveRotToMeter;
        m_internalState.angle = new Rotation2d(angleRotToRad);

        return m_internalState;

    }

    public void stopModule(){
        m_driveMotor.set(0);
        m_steerMotor.set(0);
    }


    public SwerveModuleState getCurrentState(){
        return new SwerveModuleState(getDriveVelocity(), Rotation2d.fromRotations(m_steerPositionStatusSignal.getValue()));
    }

    public double getDriveVelocity(){
        return Rot2Meter(m_driveVelocityStatusSignal.getValue());
    }

    public double Rot2Meter(double rotations){
        return (rotations * Math.PI * Constants.ModuleConstants.kWheelDiameter) / Constants.ModuleConstants.kDriveMotorGearRatio;
    }

    public double getAbsoluteEncoderRad(){
        double angle = Units.rotationsToRadians(m_steerEncoder.getAbsolutePosition().getValue());
        return angle;
    }

    public void reset(){
        m_driveMotor.setPosition(0);
    }
}
