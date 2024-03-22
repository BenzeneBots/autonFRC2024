package team4384.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.DoubleArrayPublisher;

import team4384.robot.subsystems.Shooter;

public class CmdDCSetPoint extends Command {

    private static final  String ntInstanceName = "/ShooterControls";
    private static final double SHOOTER_NOTE_LOADED_CURRENT = 10.0;
    Shooter mShooter;
    double mVoltageRequest;
    double mPercentOutput;

    DoubleSubscriber testPercentOutput;
    DoubleSubscriber testDCOffset;
    //DoubleSubscriber testRPMLeft;
    //DoubleSubscriber testRPMRight;
    DoublePublisher testPubPercentOutput;
    DoubleArrayPublisher ShooterMotorDC;


    public CmdDCSetPoint(Shooter shooter, double desiredDC) {
        mShooter = shooter;
        mPercentOutput = desiredDC;

        NetworkTable shooterNtTab =
    NetworkTableInstance.getDefault().getTable(ntInstanceName);

    ShooterMotorDC = shooterNtTab.getDoubleArrayTopic("ShooterCommandedDC").publish();


    shooterNtTab.getDoubleTopic("SetDutCycle").publish().set(0);
    shooterNtTab.getDoubleTopic("SetDCOffset").publish().set(0);
    //shooterNtTab.getDoubleTopic("SetRPMLeft").publish().set(0);
    //shooterNtTab.getDoubleTopic("SetRPMRight").publish().set(0);

    testPercentOutput = shooterNtTab.getDoubleTopic("SetDutCycle").subscribe(0);
    testDCOffset = shooterNtTab.getDoubleTopic("SetDCOffset").subscribe(0);
    //testRPMLeft = shooterNtTab.getDoubleTopic("SetRPMLeft").subscribe(0);
    //testRPMRight = shooterNtTab.getDoubleTopic("SetRPMRight").subscribe(0);
    //testPubPercentOutput = shooterNtTab.getDoubleTopic("MotorDutyCycle").publish();

        addRequirements(mShooter);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        mShooter.setPercentOutput(testPercentOutput.get(0),testDCOffset.get(0));
        //mShooter.setRPMOutput(testRPMLeft.get(0), testRPMRight.get(0));
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        mShooter.setPercentOutput(0.0, testDCOffset.get(0));
        //testPubPercentOutput.set(mShooter.getPercentOutput());
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        //ToDo - Can we detect when to end shooter command automatically? Determine conditions when shooter can stop
        //ToDo - Without a finish logic, this command will run forever .         
        if (mShooter.getFilteredCurrent() > SHOOTER_NOTE_LOADED_CURRENT) {
            return false;
        }
        else
        {
            return false;
        }
        
    }    

}