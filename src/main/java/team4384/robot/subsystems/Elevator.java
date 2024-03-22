package team4384.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team4384.robot.constants.RobotMap;
import edu.wpi.first.math.util.Units;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class Elevator extends SubsystemBase {
    private CANSparkMax leftEngage;
    private CANSparkMax rightEngage;

    public void Elevator() {
        leftEngage = new CANSparkMax(RobotMap.Elevator.Device_ID.leftEngageID, CANSparkLowLevel.MotorType.kBrushless);
        rightEngage = new CANSparkMax(RobotMap.Elevator.Device_ID.rightEngageID, CANSparkLowLevel.MotorType.kBrushless);

        rightEngage.follow(leftEngage, true);
    }

    public void engage() {
        leftEngage.getPIDController().setReference(RobotMap.Elevator.Presets.engaged, CANSparkBase.ControlType.kPosition);
    }

    public void disengage() {
        leftEngage.getPIDController().setReference(RobotMap.Elevator.Presets.home, CANSparkBase.ControlType.kPosition);
    }
}