package team4384.robot.subsystems;



import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.*;

import team4384.robot.constants.RobotMap;

public class Intake extends SubsystemBase {

    private CANSparkMax turnMotorLeader;
    private CANSparkMax turnMotorFollower;
    private TalonFX spinner;

enum IntakeState {
  OFF,
  RUNNING,
  STUCK
}

private IntakeState currentIntakeState = IntakeState.OFF;

public Intake() {
        //ToDo - Add Motor configurations
  turnMotorLeader = new CANSparkMax(RobotMap.Intake.turnMotorLeader, MotorType.kBrushless);
  turnMotorFollower = new CANSparkMax(RobotMap.Intake.turnMotorFollower, MotorType.kBrushless);
  spinner = new TalonFX(RobotMap.Intake.spinner);
  turnMotorFollower.follow(turnMotorLeader, true);
}

public void raise() {
    turnMotorLeader.set(0.1);
    fixPos();
}

    public void lower() {
        turnMotorLeader.set(-0.1);
        fixPos();
    }

    public void intakeWheelForward(double speed) {
        //Todo - Set Motor Speed here
        //Sends the note into the robot
        turnMotorLeader.setInverted(false);
        turnMotorLeader.set(speed);
        turnMotorFollower.setInverted(false);
        turnMotorFollower.set(speed);
      }

    public void intakeWheelReverse(double speed) {
        //Todo - Set Motor Speed here
        //Sends the note out of the robot 
        turnMotorLeader.setInverted(true);
        turnMotorLeader.set(speed);
        turnMotorFollower.setInverted(true);
        turnMotorFollower.set(speed);
      }
    
    public void intakePivot(double intakePivotRotation) {
      //Todo - Set Motor Speed and angle here
      //Sends the intake toward the robot if rotation is positive, and away if negetive, probably
      //intakeM1Kracken.setControl(krackenConfig1.withPosition(intakePivotRotation));
      //intakeM2Kracken.setControl(krackenConfig1.withPosition(intakePivotRotation));

    }

    public void feed() {
        spinner.set(0.65);
    }


    public void backfeed() {
        spinner.set(-0.65);
    }

    public double getSpeed() {
      return turnMotorLeader.get();
    }

    public void halt() {
        spinner.stopMotor();
    }

    public void fixPos() {
        turnMotorLeader.getPIDController().setReference(turnMotorLeader.getEncoder().getPosition(), CANSparkBase.ControlType.kPosition);
        turnMotorFollower.getPIDController().setReference(turnMotorFollower.getEncoder().getPosition(), CANSparkBase.ControlType.kPosition);
    }
}
