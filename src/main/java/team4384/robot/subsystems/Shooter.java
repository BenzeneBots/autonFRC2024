package team4384.robot.subsystems;


import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team4384.robot.constants.RobotMap;


import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.CAN;

import edu.wpi.first.math.util.Units;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;

import com.ctre.phoenix6.signals.NeutralModeValue;


public class Shooter extends SubsystemBase {
    private TalonFX mLeftIndexerMotor;
    private TalonFX mRightIndexerMotor;
    private TalonFX mLeftShooterMotor;
    private TalonFX mRightShooterMotor;

    private TalonFXConfiguration mLeftShooterMotorConfig = new TalonFXConfiguration();
    private TalonFXConfiguration mRightShooterMotorConfig = new TalonFXConfiguration();

    private static final double SHOOTER_NOTE_INTAKE_THRESH_CURRENT = 15.0;

    private double mShooterRPM;
    private double mDesiredDC;
    private double mLeftMotorOutputCurrent;
    private double mLeftFiltereMotorOutputCurrent;
    private double mLeftMotorSupplyCurrent;
    private double mLeftMotorVelocity;

    private double mRightFiltereMotorOutputCurrent;
    private double mRightMotorOutputCurrent;
    private double mRightMotorVelocity;
    
    private double mRpsRight;
    private double mRpsLeft;
    private boolean done;


    private final VoltageOut mVoltageRequest = new VoltageOut(0);
    private final VelocityVoltage mVelocityRequest = new VelocityVoltage(0).withSlot(0);
    private final DutyCycleOut mDutyCycleRequest = new DutyCycleOut(0);
    private PositionVoltage mManualIntakeRequest = new PositionVoltage(0).withSlot(1);
    private final double mGearRatio = 1;

    LinearFilter currentFilter = LinearFilter.movingAverage(5);

    public Shooter() {
        mLeftIndexerMotor = new TalonFX(RobotMap.Pivot.DeviceID.leftIndexer);
        mRightIndexerMotor = new TalonFX(RobotMap.Pivot.DeviceID.rightIndexer);

        mLeftShooterMotor = new TalonFX(RobotMap.Pivot.DeviceID.leftShooter);
        mRightShooterMotor = new TalonFX(RobotMap.Pivot.DeviceID.rightShooter);

        mLeftIndexerMotor.setInverted(false);
        mRightIndexerMotor.setInverted(true);
        mRightShooterMotor.setInverted(false);
        mLeftShooterMotor.setInverted(true);

//        this.updateMotorConfig(mLeftShooterMotorConfig, 0);
//        saveMotorConfig(mLeftShooterMotor, mLeftShooterMotorConfig);
//        this.updateMotorConfig(mRightShooterMotorConfig, 0);
//        saveMotorConfig(mRightShooterMotor, mRightShooterMotorConfig);
    }


    public void shoot() {
        mLeftShooterMotor.set(RobotMap.Pivot.Speeds.shootSpeed);
        mRightShooterMotor.set(RobotMap.Pivot.Speeds.shootSpeed);
        done = true;
    }

    public void back() {
        if(!done && mLeftIndexerMotor.getSupplyCurrent().getValue() < 2.5) {
            mLeftShooterMotor.set(RobotMap.Pivot.Speeds.backSpeed);
            mRightShooterMotor.set(RobotMap.Pivot.Speeds.backSpeed);
            mLeftIndexerMotor.set(RobotMap.Pivot.Speeds.backSpeed);
            mRightIndexerMotor.set(RobotMap.Pivot.Speeds.backSpeed);
        } else if(mLeftShooterMotor.getSupplyCurrent().getValue() > 2.5) {
            done = true;
        }
    }

    public void index() {
        mLeftIndexerMotor.set(RobotMap.Pivot.Speeds.indexSpeed);
        mRightIndexerMotor.set(RobotMap.Pivot.Speeds.indexSpeed);
    }


    public void stopShoot() {
        mLeftIndexerMotor.stopMotor();
        mLeftShooterMotor.stopMotor();
        mRightShooterMotor.stopMotor();
        mRightIndexerMotor.stopMotor();
    }

    
    public void setVoltageOutput(double voltageOut) {
        setForwardShooterDir();
        mRightShooterMotor.setControl(mVoltageRequest.withOutput(voltageOut));
        mLeftShooterMotor.setControl(mVoltageRequest.withOutput(voltageOut));
      }

    /**
    * Control the shooter motor by varying voltage duty cycle or PWM.
    * To induce the spin to Notes, apply varying duty cycle to the motors.
    * Use Offset +ve or -ve value to increase or decrease the duty cycle.
    *
    * @param  desiredOut  value of the duty cycle applied to the motor
    * @param  desiredOffset value of the offset factor applied to the left motor.
    * @return      None

    */

    public void setPercentOutput(double desiredOut, double desiredOffset) {
        setForwardShooterDir();

        mDesiredDC = desiredOut;
        mRightShooterMotor.setControl(mDutyCycleRequest.withOutput(desiredOut));
        if (desiredOffset > 0) 
        {
         double desiredOffsetDC = desiredOut * (1+desiredOffset);
         mLeftShooterMotor.setControl(mDutyCycleRequest.withOutput(desiredOffsetDC));
        }
        else {
            mLeftShooterMotor.setControl(mDutyCycleRequest.withOutput(desiredOut));
        } 
      }

    /**
    * Control the shooter motor by varying voltage duty cycle or PWM.
    * Use for the manual Note intake. Set the direction of the motor to reverse direction 
    * of the shooter function. Threshold is required to ensure the motor rotates just enough 
    * to intake the Note without loosing it.
    *

    */

    public void setLowSpeedPercentOutput(double desiredDC) {
      setForwardShooterDir(); 
      mDesiredDC = desiredDC;
      mLeftShooterMotor.setControl(mDutyCycleRequest.withOutput(mDesiredDC));
      mRightShooterMotor.setControl(mDutyCycleRequest.withOutput(mDesiredDC));
    }


    public double[] getPercentOutput() {
        return new double[] {mRightShooterMotor.getDutyCycle().getValueAsDouble(), 
                             mLeftShooterMotor.getDutyCycle().getValueAsDouble()};
    }


    public void setRPMOutput(double lRpmRight, double lRpmLeft) {
        setForwardShooterDir();
        mShooterRPM = lRpmRight;
        mRpsRight = lRpmRight/60.0;
        mRpsLeft = lRpmLeft/60.0; 
        //TODO: Determing do we need feed forward for the motor in velocity control
        mRightShooterMotor.setControl(mVelocityRequest.withVelocity(mRpsRight).withFeedForward(0));
        mLeftShooterMotor.setControl(mVelocityRequest.withVelocity(mRpsLeft).withFeedForward(0));
      }

    
    public void setRPMManualIntake(double intakDegrees) {        
        setReverseIntakeDir();
        mRightShooterMotor.setControl(mManualIntakeRequest.withPosition(Units.degreesToRadians(intakDegrees)));
        mLeftShooterMotor.setControl(mManualIntakeRequest.withPosition(Units.degreesToRadians(intakDegrees)));
    }

      
    public double getFilteredCurrent() {
        return mLeftFiltereMotorOutputCurrent;
    }

    public boolean getNoteIntakeCurrentStatus() {
        return (mLeftFiltereMotorOutputCurrent > SHOOTER_NOTE_INTAKE_THRESH_CURRENT);
    }

    public double[] getMotorVelocity() {
        return new double[] {mLeftMotorVelocity,
                            mRightMotorVelocity};

    }

    @Override
    public void periodic() {
        mLeftMotorOutputCurrent = mRightShooterMotor.getStatorCurrent().getValueAsDouble();
        mRightMotorOutputCurrent = mLeftShooterMotor.getStatorCurrent().getValueAsDouble();
        
        mLeftMotorSupplyCurrent = mRightShooterMotor.getSupplyCurrent().getValueAsDouble();

        mLeftMotorVelocity = mRightShooterMotor.getVelocity().getValueAsDouble();
        mRightMotorVelocity = mLeftShooterMotor.getVelocity().getValueAsDouble();

        mLeftFiltereMotorOutputCurrent = currentFilter.calculate(mLeftMotorOutputCurrent);
        mRightFiltereMotorOutputCurrent = currentFilter.calculate(mRightMotorOutputCurrent);


        double cur1 = mLeftIndexerMotor.getSupplyCurrent().getValue();
        double cur2 = mRightIndexerMotor.getSupplyCurrent().getValue();
        // Used for Current Limit Stopping Motor
        SmartDashboard.putNumber("left current", mLeftIndexerMotor.getSupplyCurrent().getValue());
        SmartDashboard.putNumber("right current", mRightIndexerMotor.getSupplyCurrent().getValue());

        boolean in = cur1 > 2;
        SmartDashboard.putBoolean("In", in);
    }

    private void updateMotorConfig(TalonFXConfiguration lConfigMotor, int slotNum) {
        lConfigMotor.MotorOutput.NeutralMode = NeutralModeValue.Coast;        
        lConfigMotor.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.5;  
        lConfigMotor.Feedback.SensorToMechanismRatio = 1;          
        lConfigMotor.TorqueCurrent.PeakForwardTorqueCurrent = 30;
        lConfigMotor.TorqueCurrent.PeakReverseTorqueCurrent = -30;

        if (slotNum == 0) {
            lConfigMotor.Slot0.kP = 0.11;
            lConfigMotor.Slot0.kI = 0.5;
            lConfigMotor.Slot0.kD = 0.0001;
            lConfigMotor.Slot0.kS = 0.05; // Add 0.05 V output to overcome static friction
            //TODO: Change this to Krakon KV
            lConfigMotor.Slot0.kV = 0.12; // Falcon 500 is a 500kV motor, 500rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / Rotation per second;
        } else if (slotNum ==1 ) {
            lConfigMotor.Slot1.kP = 24; //An error of 0.5 rotations results in 12 V output
            lConfigMotor.Slot1.kI = 0.0; // no output for integrated error
            lConfigMotor.Slot1.kD = 0.1; // A velocity of 1 rps results in 0.1 V output
        }    
    }

    private void saveMotorConfig(TalonFX lMotorTalonFX,TalonFXConfiguration lConfigMotor) {
            /* Retry config apply up to 5 times, report if failure */
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
        status = lMotorTalonFX.getConfigurator().apply(lConfigMotor);
        if (status.isOK()) break;
        }
        if(!status.isOK()) {
        System.out.println("Could not apply configs, error code: " + status.toString());
        }
    }

    private void setForwardShooterDir() {
        mRightShooterMotor.setInverted(true);
        mLeftShooterMotor.setInverted(false);
    }

    private void setReverseIntakeDir() {
        mRightShooterMotor.setInverted(true);
        mLeftShooterMotor.setInverted(false);
    }    

}
