package team4384.robot.commands.intake;

import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import team4384.robot.subsystems.Shooter;

public class CmdPosManualIntake extends Command {

    private static final  String ntInstanceName = "/IntakeControls";
    Shooter mShooter;
    double Degrees;
    DoubleArrayPublisher ShooterMotorSpeed;

    DoubleSubscriber testDegrees;

    public CmdPosManualIntake(Shooter shooter, double lDegrees) {
        this.mShooter = shooter;
        this.Degrees = lDegrees;

        addRequirements(mShooter);

        NetworkTable shooterIntakeNtTab = NetworkTableInstance.getDefault().getTable(ntInstanceName);
        ShooterMotorSpeed = shooterIntakeNtTab.getDoubleArrayTopic("ShooterCommandedSpeed").publish();
        shooterIntakeNtTab.getDoubleTopic("SetDegrees").publish().set(0);

        testDegrees = shooterIntakeNtTab.getDoubleTopic("SetDegrees").subscribe(0);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}


    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        this.Degrees = testDegrees.get(0);
        mShooter.setRPMManualIntake(this.Degrees);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        mShooter.setRPMManualIntake(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() { 
        //ToDo - Can we detect when to end shooter command automatically? Determine conditions when shooter can stop
        //ToDo - Without a finish logic, this command will run forever .   
        if (mShooter.getNoteIntakeCurrentStatus()) {
            return true;
        } else {
            return false;
        }
    }

}

