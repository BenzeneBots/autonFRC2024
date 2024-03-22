package team4384.robot.sim;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.math.util.Units;
import team4384.robot.constants.ConstIntake;
import team4384.robot.constants.ConstShooter;
import team4384.robot.constants.RobotMap;
import team4384.robot.subsystems.Intake;

public class SimSuperStructure implements AutoCloseable{
    Intake m_intake;

    private final Mechanism2d m_mech2d =
      new Mechanism2d(RobotMap.drivebaseLength * 2, RobotMap.drivebaseLength * 2);

    private final MechanismRoot2d m_drivebaseRoot2d =
      m_mech2d.getRoot("Drivebase", RobotMap.drivebaseLength * 0.5, RobotMap.drivebaseWidth * 0.5);

      private final MechanismLigament2d m_drivebase2d =
      m_drivebaseRoot2d.append(new MechanismLigament2d("Drivebase", RobotMap.drivebaseLength, 0));

      private final MechanismLigament2d m_intake2d =
      m_drivebaseRoot2d.append(new MechanismLigament2d("Intake", ConstIntake.intakeLength, 0));


    private final IntakeVisualizer mIntakeVisualizer = new IntakeVisualizer("m_intake2d");
    
    private final Color8Bit m_drivebase2d_originalColor;
    private final Color8Bit m_intake2d_originalColor;

    private final ArrayList<VisualizerUtils.MechanismDisplay> m_displays = new ArrayList<>();

    public SimSuperStructure() {
        m_drivebase2d.setColor(new Color8Bit(235, 137, 52));
        m_intake2d.setColor(new Color8Bit(235, 229, 52));

        m_drivebase2d_originalColor = m_drivebase2d.getColor();
        m_intake2d_originalColor = m_intake2d.getColor();

        SmartDashboard.putData("SuperStructure Sim", m_mech2d);
        if (RobotBase.isSimulation()) {
            var intakeDisplay =
            new VisualizerUtils.MechanismDisplay(0.5, 0.5, mIntakeVisualizer.getLigament());
            m_displays.add(intakeDisplay);  

            for (var display : m_displays) display.addSmartDashboardDisplay();
        }

    }

  public void registerIntake(Intake intake) {
    m_intake = intake;
  }

    /* Function to visualize the speed of a particular motor. */
    public void updateMotorColor(
        MechanismLigament2d ligament, double motorSpeed, Color8Bit originalColor) {
      double deltaBrightness = Math.abs(motorSpeed) * 75;
  
      Color8Bit newColor =
          new Color8Bit(
              originalColor.red + (int) deltaBrightness,
              originalColor.green + (int) deltaBrightness,
              originalColor.blue + (int) deltaBrightness);
  
      ligament.setColor(newColor);
    }

    
  public void updateIntake() {
//    mIntakeVisualizer.update(m_intake.getAngle(), m_intake.getSpeed());
    updateMotorColor(m_intake2d, m_intake.getSpeed(), m_intake2d_originalColor);
  }


  public void periodic() {
    if (m_intake != null) updateIntake();
  }
  
    @Override
    public void close() throws Exception {
        for (var display : m_displays) display.close();
    }
    
}
