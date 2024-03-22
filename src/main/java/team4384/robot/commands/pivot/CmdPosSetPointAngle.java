package team4384.robot.commands.pivot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import team4384.robot.subsystems.Pivot;

public class CmdPosSetPointAngle extends Command{
    private static final  String ntInstanceName = "/PivotControl";

    private Pivot mPivot;
    private double mDesiredPosInAngle;
    private double mDesiredPosInRadians;

    DoubleArrayPublisher pivotDesiredAnglePub;
    private DoubleSubscriber testPivotDesiredAngle;
    private DoublePublisher currentPivotAngle;
    
    public CmdPosSetPointAngle(Pivot lPivot, double lDesiredAngle) {

        mPivot = lPivot;
        mDesiredPosInAngle = lDesiredAngle;
        mDesiredPosInRadians = Units.degreesToRadians(mDesiredPosInAngle);
        addRequirements(mPivot);

        NetworkTable pivotNtTab = NetworkTableInstance.getDefault().getTable(ntInstanceName);

        pivotDesiredAnglePub = pivotNtTab.getDoubleArrayTopic("PivotDesiredAngle").publish();
        currentPivotAngle = pivotNtTab.getDoubleTopic("PivotCurrentAngle").publish();

        pivotNtTab.getDoubleTopic("SetPivotAngle").publish().set(0);

        testPivotDesiredAngle = pivotNtTab.getDoubleTopic("SetPivotAngle").subscribe(0);
    }
    
    @Override
    public void initialize() {
        super.initialize();
        //TODO: Desired angle is overwritten by the dashboard.Once correct value is determined removed dashboard entry
        mDesiredPosInAngle = testPivotDesiredAngle.getAsDouble();
        mDesiredPosInRadians= Units.degreesToRadians(mDesiredPosInAngle);
        mPivot.setDsrdPosInDeg(mDesiredPosInAngle, mDesiredPosInAngle);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        currentPivotAngle.set(mPivot.getCurrentMotorPositionInDeg()[0]);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        //Wait till final position is reached, We need a timeout so that we dont have to wait for ever
        //TODO: Add Timer override to exit within a period of time.
        if (mPivot.isDesiredPositionReached())
        {
            mPivot.resetPosition();
            currentPivotAngle.set(mPivot.getCurrentMotorPositionInDeg()[0]);
            return true;
        }
        else {
            return false;
        }
    }

}