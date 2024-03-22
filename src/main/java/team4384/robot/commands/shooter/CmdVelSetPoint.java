package team4384.robot.commands.shooter;

import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import team4384.robot.subsystems.Pivot;
import team4384.robot.subsystems.Shooter;

public class CmdVelSetPoint extends Command{

    private static final  String ntInstanceName = "/ShooterControls";
    Shooter mShooter;
    Pivot mPivot;
    double mTargetRPMRight;
    double mTargetRPMLeft;
    DoubleSubscriber testRPMLeft;
    DoubleSubscriber testRPMRight;
    DoubleSubscriber testRPMLeftPivot;
    DoubleSubscriber testRPMRightPivot;
    DoubleArrayPublisher ShooterMotorSpeed;

    public CmdVelSetPoint(Shooter shooter, double lTargetRPMRight, double lTargetRPMLeft) {
        this.mTargetRPMRight = lTargetRPMRight;
        this.mTargetRPMLeft = lTargetRPMLeft;
        this.mShooter = shooter;
        addRequirements(mShooter);

        NetworkTable shooterNtTab = NetworkTableInstance.getDefault().getTable(ntInstanceName);

        ShooterMotorSpeed = shooterNtTab.getDoubleArrayTopic("ShooterCommandedSpeed").publish();
        shooterNtTab.getDoubleTopic("SetRPMLeft").publish().set(0);
        shooterNtTab.getDoubleTopic("SetRPMRight").publish().set(0);

        testRPMLeft = shooterNtTab.getDoubleTopic("SetRPMLeft").subscribe(0);
        testRPMRight = shooterNtTab.getDoubleTopic("SetRPMRight").subscribe(0);
        //testRPMRightPivot = shooterNtTab.getDoubleTopic("SetRPMPivotRight").subscribe(0);
        //testRPMLeftPivot = shooterNtTab.getDoubleTopic("SetRPMPivotLeft").subscribe(0);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}


    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        mShooter.setRPMOutput(testRPMLeft.get(0), testRPMRight.get(0));
       // mPivot.setRPMOutput(testRPMRightPivot.get(0), testRPMLeftPivot.get(0));
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        mShooter.setRPMOutput(0, 0);
        //testPubPercentOutput.set(mShooter.getPercentOutput());
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() { 
        //ToDo - Can we detect when to end shooter command automatically? Determine conditions when shooter can stop
        //ToDo - Without a finish logic, this command will run forever .         
        return false;
    }

}
