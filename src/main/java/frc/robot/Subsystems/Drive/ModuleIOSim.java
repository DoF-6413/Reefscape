package frc.robot.Subsystems.Drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.RobotStateConstants;

/** ModuleIO implementation for the simulated mode of the robot */
public class ModuleIOSim implements ModuleIO {
  // Flywheel simulations
  private final DCMotorSim m_driveSim;
  private final DCMotorSim m_turnSim;

  // Motor voltages
  private double m_driveAppliedVolts = 0.0;
  private double m_turnAppliedVolts = 0.0;

  // PID & Feedforward controllers
  private final PIDController m_driveController;
  private SimpleMotorFeedforward m_driveFeedforward;
  private double m_driveSetpointRadPerSec = 0.0;

  /**
   * Constructs a new {@link ModuleIOSim} instance.
   *
   * <p>This creates a new {@link ModuleIO} object that uses the simulated versions of the KrakenX60
   * and NEO motors to run the Drive and Turn of the simulated Module.
   */
  public ModuleIOSim() {
    System.out.println("[Init] Creating ModuleIOSim");

    // Initialize simulated motors
    m_driveSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getKrakenX60(1),
                DriveConstants.DRIVE_MOI_KG_M2,
                DriveConstants.DRIVE_GEAR_RATIO),
            DCMotor.getKrakenX60(1));
    m_turnSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getNEO(1), DriveConstants.TURN_MOI_KG_M2, DriveConstants.TURN_GEAR_RATIO),
            DCMotor.getNEO(1));

    // Initialize PID & Feedforward controllers
    m_driveController =
        new PIDController(
            DriveConstants.DRIVE_KP_SIM, DriveConstants.DRIVE_KI_SIM, DriveConstants.DRIVE_KD_SIM);
    m_driveFeedforward =
        new SimpleMotorFeedforward(DriveConstants.DRIVE_KS_SIM, DriveConstants.DRIVE_KV_SIM);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    // Calculate and apply next output voltage from the PID and Feedforward controller
    m_driveAppliedVolts =
        m_driveController.calculate(inputs.driveVelocityRadPerSec, m_driveSetpointRadPerSec)
            + m_driveFeedforward.calculate(m_driveSetpointRadPerSec);
    this.setDriveVoltage(m_driveAppliedVolts);

    // Update simulated motors
    m_driveSim.update(RobotStateConstants.LOOP_PERIODIC_SEC);
    m_turnSim.update(RobotStateConstants.LOOP_PERIODIC_SEC);

    // Update logged Drive motor inputs from the simulated flywheel system
    inputs.driveIsConnected = true;
    inputs.driveAppliedVoltage = m_driveAppliedVolts;
    inputs.driveCurrentAmps = Math.abs(m_driveSim.getCurrentDrawAmps());
    inputs.drivePositionRad = m_driveSim.getAngularPositionRad();
    inputs.driveVelocityRadPerSec = m_driveSim.getAngularVelocityRadPerSec();

    // Update logged Turn motor inputs from the simulated flywheel system
    inputs.absoluteEncoderIsConnected = true;
    inputs.turnAppliedVoltage = m_turnAppliedVolts;
    inputs.turnCurrentAmps = Math.abs(m_turnSim.getCurrentDrawAmps());
    inputs.turnAbsolutePositionRad =
        Rotation2d.fromRadians(MathUtil.angleModulus(m_turnSim.getAngularPositionRad()));
    inputs.turnVelocityRadPerSec = m_turnSim.getAngularVelocityRadPerSec();

    // Update odometry inputs
    inputs.odometryTimestamps = new double[] {Timer.getFPGATimestamp()};
    inputs.odometryDrivePositionsRad = new double[] {inputs.drivePositionRad};
    inputs.odometryAbsTurnPositions = new Rotation2d[] {inputs.turnAbsolutePositionRad};
  }

  @Override
  public void setDriveVoltage(double volts) {
    m_driveAppliedVolts =
        MathUtil.clamp(volts, -RobotStateConstants.MAX_VOLTAGE, RobotStateConstants.MAX_VOLTAGE);
    m_driveSim.setInputVoltage(m_driveAppliedVolts);
  }

  @Override
  public void setTurnVoltage(double volts) {
    m_turnAppliedVolts =
        MathUtil.clamp(volts, -RobotStateConstants.MAX_VOLTAGE, RobotStateConstants.MAX_VOLTAGE);
    m_turnSim.setInputVoltage(m_turnAppliedVolts);
  }

  @Override
  public void setDriveVelocity(double velocityRadPerSec) {
    m_driveSetpointRadPerSec = velocityRadPerSec;
  }

  @Override
  public void setDrivePID(double kP, double kI, double kD) {
    m_driveController.setPID(kP, kI, kD);
  }

  @Override
  public void setDriveFF(double kS, double kV) {
    m_driveFeedforward.setKs(kS);
    m_driveFeedforward.setKv(kV);
  }
}
