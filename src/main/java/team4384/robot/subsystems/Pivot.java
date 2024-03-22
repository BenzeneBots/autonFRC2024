package team4384.robot.subsystems;


import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import com.ctre.phoenix6.StatusCode;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;

import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team4384.robot.constants.RobotMap;


public class Pivot extends SubsystemBase {

    private final TalonFXConfiguration mLeaderConfig = new TalonFXConfiguration();
    private final TalonFXConfiguration mFollowerConfig = new TalonFXConfiguration();

    private TalonFX mLeaderMotor;
    private TalonFX mFollowerMotor;

    private PositionVoltage pos_requests;
    private double mPivotRPM;
    private double mRpsRight;
    private double mRpsLeft;

    //Pivot Mechanical details
    //TODO: Update Gear Ratio
    private double mPivotRotPerRadian = 1; //mGearRatio / (2 * Math.PI);
    private final double PIVOT_ALLOWED_TOLERANCE = 0.2;

    //Pivot min and maximum angle. Pivot operation is between 2 positions. These position to be aligned
    // as per the pivot mount on the robot
    private static double minAngleInRad = 0.0;
    private static double maxAngleInRad = 360;

    private double mRightMotorPosInDeg;
    private double mLightMotorPosInDeg;
    private double mRightMotorPosInRotation;
    private double mLeftMotorPosInRotation;

    private final VelocityVoltage mVelocityRequest = new VelocityVoltage(0).withSlot(0);
    private final PositionVoltage mPositionVoltageRequest = new PositionVoltage(0).withSlot(0);

    private RelativeEncoder mEncoder;
    private final VoltageOut mVoltageRequest = new VoltageOut(0);

    /* Keep a brake request so we can disable the motor */
    private final NeutralOut m_brake = new NeutralOut();


    public Pivot() {
        mLeaderMotor = new TalonFX(RobotMap.Pivot.DeviceID.turnMotorLeader);
        mFollowerMotor = new TalonFX(RobotMap.Pivot.DeviceID.turnMotorFollower);

        mFollowerMotor.setControl(new Follower(mLeaderMotor.getDeviceID(), true));

        this.updateMotorConfig(mLeaderConfig, 0);
        saveMotorConfig(mLeaderMotor, mLeaderConfig);

        this.updateMotorConfig(mFollowerConfig, 0);
        saveMotorConfig(mFollowerMotor, mFollowerConfig);        
    }


    public void raise() {
        // No PID Tuning yet
        mLeaderMotor.set(RobotMap.Pivot.Speeds.pivotSpeed);
    }

    public void lower() {
        // No PID Tuning yet
        mLeaderMotor.set(RobotMap.Pivot.Speeds.pivotSpeed * -1);
    }
    public void stop() {
        mLeaderMotor.stopMotor();
        mFollowerMotor.stopMotor();
    }

    public void setDsrdPosInDeg(double rightMotorDeg, double leftMotorDeg) {
        //double rightMotorPosInRads = Units.degreesToRadians(rightMotorDeg);
        //double leftMotorPosInRads = Units.degreesToRadians(leftMotorDeg);

        mRightMotorPosInDeg = MathUtil.clamp(rightMotorDeg, minAngleInRad, maxAngleInRad);
        mLightMotorPosInDeg = MathUtil.clamp(leftMotorDeg, minAngleInRad, maxAngleInRad);
        
        mRightMotorPosInRotation = mRightMotorPosInDeg/360.0;
        mLeftMotorPosInRotation = mLightMotorPosInDeg/360.0;
        mLeaderMotor.setControl(mPositionVoltageRequest.withPosition(mRightMotorPosInRotation));
        mFollowerMotor.setControl(mPositionVoltageRequest.withPosition(mLeftMotorPosInRotation));

    }

    public double[] getCurrentMotorPosition() {
        double currentRightMotorPosition;
        double currentLeftMotorPosition;

        currentRightMotorPosition = mLeaderMotor.getPosition().getValueAsDouble();
        currentLeftMotorPosition = mLeaderMotor.getPosition().getValueAsDouble();

        return new double[] {currentRightMotorPosition, currentLeftMotorPosition};
    }

    public double[] getCurrentMotorPositionInDeg() {
        double currentRightMotorPosition;
        double currentLeftMotorPosition;

        currentRightMotorPosition = mLeaderMotor.getPosition().getValueAsDouble();
        currentLeftMotorPosition = mFollowerMotor.getPosition().getValueAsDouble();

        return new double[] {currentRightMotorPosition, currentLeftMotorPosition};
    }

    public boolean isDesiredPositionReached() {
        //Note: We are checking only for Right motor. What if there is mis-alignemnt between right and left motors?
        double setPositionReached = Math.abs(mRightMotorPosInRotation-this.getCurrentMotorPosition()[0]);
        //double TestsetPositionReached = Math.abs(mRightMotorPosInDeg-(mPivotMotors[0].getPosition().getValueAsDouble()));
        boolean isPositionReached = setPositionReached < PIVOT_ALLOWED_TOLERANCE;
        //isPositionReached = TestsetPositionReached < 0.2;
        return isPositionReached;
    }

    public void resetPosition() {
        mLeaderMotor.setPosition(minAngleInRad);
        mFollowerMotor.setPosition(minAngleInRad);
    }

    public void setRPMOutput(double lRpmRight, double lRpmLeft) {
        setPivotDirection();
        mPivotRPM = lRpmRight;
        mRpsRight = lRpmRight/60.0;
        mRpsLeft = lRpmLeft/60.0; 
        //TODO: Determing do we need feed forward for the motor in velocity control
        mLeaderMotor.setControl(mVelocityRequest.withVelocity(mRpsRight).withFeedForward(0));
        mFollowerMotor.setControl(mVelocityRequest.withVelocity(mRpsLeft).withFeedForward(0));
      }

    @Override
    public void periodic() {

        // Use Values for Tuning Preset Positions
        // Planning on implementing limit switch
        SmartDashboard.putNumber("pivot", mLeaderMotor.getPosition().getValue());
        SmartDashboard.putNumber("Pivot Current", mLeaderMotor.getSupplyCurrent().getValue());
        SmartDashboard.putNumber("Pivot Current 1", mFollowerMotor.getSupplyCurrent().getValue());

        //mPivotMotors[0].setControl(mVoltageRequest.withOutput(.5));
        //mPivotMotors[1].setControl(mVoltageRequest.withOutput(.5));
        //API for 'get encoders'
    }

    private void updateMotorConfig(TalonFXConfiguration lConfigMotor, int slotNum) {
        //TODO: ADJUST THESE VALUES
        lConfigMotor.MotorOutput.NeutralMode = NeutralModeValue.Coast;        
        lConfigMotor.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.5;  
        lConfigMotor.Feedback.SensorToMechanismRatio = 1;          
        lConfigMotor.TorqueCurrent.PeakForwardTorqueCurrent = 30;
        lConfigMotor.TorqueCurrent.PeakReverseTorqueCurrent = -30;   

        if (slotNum == 0) {
            lConfigMotor.Slot0.kP = 4.8;
            lConfigMotor.Slot0.kI = 0;
            lConfigMotor.Slot0.kD = 0.1;
            lConfigMotor.Slot0.kS = 0.25;
            lConfigMotor.Slot0.kV = 0.12; 
        } else if (slotNum ==1 ) {
            lConfigMotor.Slot1.kP = 1;
            lConfigMotor.Slot1.kI = 0.1;
            lConfigMotor.Slot1.kD = 0.0; 
        }    
    }

    private void saveMotorConfig(TalonFX lMotorTalonFX,TalonFXConfiguration lConfigMotor) {
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
        status = lMotorTalonFX.getConfigurator().apply(lConfigMotor);
        if (status.isOK()) break;
        }
        if(!status.isOK()) {
        System.out.println("Could not apply configurations: " + status.toString());
        }
    }

    private void setPivotDirection() {
        mLeaderMotor.setInverted(false);
        mFollowerMotor.setInverted(false);
    }
}

