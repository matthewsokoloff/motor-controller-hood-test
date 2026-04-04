package frc.robot;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot
{
  private final XboxController driverXbox = new XboxController(0);

  private final SparkMax hoodMotor = new SparkMax(7, MotorType.kBrushed);
  private final RelativeEncoder hoodEncoder = hoodMotor.getEncoder();

  // Shooter
  private final SparkMax shooterMotor = new SparkMax(10, MotorType.kBrushless);
  private final SparkMax feederMotor  = new SparkMax(6, MotorType.kBrushless);

  private boolean homed = false;
  private boolean homing = false;

  private final Timer homingTimer = new Timer();
  private final Timer overCurrentTimer = new Timer();

  private boolean prevA = false;

  // Tune on the real robot
  private static final double HOMING_POWER = -0.05;
  private static final double MANUAL_POWER = 0.05;
  private static final double CURRENT_THRESHOLD_AMPS = 5.0;
  private static final double MIN_RUN_SECONDS = 0.20;
  private static final double DEBOUNCE_SECONDS = 0.10;
  private static final double HOMING_TIMEOUT_SECONDS = 2.0;

  // Shooter tuning
  private static final double SHOOT_SPEED = 1.0;
  private static final double FEED_SPEED  = 0.6;

  @Override
  public void robotInit()
  {
    // Hood config
    SparkMaxConfig hoodConfig = new SparkMaxConfig();
    hoodConfig.idleMode(IdleMode.kBrake);
    hoodConfig.smartCurrentLimit(10);

    hoodMotor.configure(
        hoodConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    // Shooter motor config
    SparkMaxConfig shooterConfig = new SparkMaxConfig();
    shooterConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(40)
        .inverted(true);   // flip if needed

    shooterMotor.configure(
        shooterConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    // Feeder motor config
    SparkMaxConfig feederConfig = new SparkMaxConfig();
    feederConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(40)
        .inverted(false);  // flip if needed

    feederMotor.configure(
        feederConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  @Override
  public void teleopPeriodic()
  {
    boolean a = driverXbox.getAButton();
    boolean b = driverXbox.getBButton();
    boolean x = driverXbox.getXButton();
    boolean y = driverXbox.getYButton();

    // hood movement

    // A button rising edge -> start homing
    if (a && !prevA)
    {
      startHoming();
    }
    prevA = a;

    // Manual control overrides homing
    if (x)
    {
      homing = false;
      homed = false;
      hoodMotor.set(-MANUAL_POWER);
    }
    else if (y)
    {
      homing = false;
      homed = false;
      hoodMotor.set(MANUAL_POWER);
    }
    else if (!homing)
    {
      hoodMotor.stopMotor();
    }

    if (homing)
    {
      hoodMotor.set(HOMING_POWER);

      boolean eligible = homingTimer.get() > MIN_RUN_SECONDS;
      boolean overCurrent = hoodMotor.getOutputCurrent() >= CURRENT_THRESHOLD_AMPS;

      if (eligible && overCurrent)
      {
        if (!overCurrentTimer.isRunning())
        {
          overCurrentTimer.restart();
        }
      }
      else
      {
        overCurrentTimer.stop();
        overCurrentTimer.reset();
      }

      boolean hitStop = overCurrentTimer.hasElapsed(DEBOUNCE_SECONDS);
      boolean timedOut = homingTimer.hasElapsed(HOMING_TIMEOUT_SECONDS);

      if (hitStop || timedOut)
      {
        hoodMotor.stopMotor();
        homing = false;

        if (hitStop)
        {
          hoodEncoder.setPosition(0.0);
          homed = true;
        }
      }
    }

    // shooter to b, equal to configbindings in yagsl
    // Equivalent to:
    // b().whileTrue(shooterSubsystem.shoot()).onFalse(stopAll())

    if (b)
    {
      shooterMotor.set(SHOOT_SPEED);
      feederMotor.set(FEED_SPEED);
    }
    else
    {
      shooterMotor.stopMotor();
      feederMotor.stopMotor();
    }

    // SmartDB
    SmartDashboard.putNumber("Hood Current", hoodMotor.getOutputCurrent());
    SmartDashboard.putNumber("Hood Position", hoodEncoder.getPosition());
    SmartDashboard.putBoolean("Hood Homing", homing);
    SmartDashboard.putBoolean("Hood Homed", homed);

    SmartDashboard.putBoolean("Shooter B Pressed", b);
    SmartDashboard.putNumber("Shooter Current", shooterMotor.getOutputCurrent());
    SmartDashboard.putNumber("Feeder Current", feederMotor.getOutputCurrent());
  }

  private void startHoming()
  {
    homed = false;
    homing = true;
    homingTimer.restart();
    overCurrentTimer.stop();
    overCurrentTimer.reset();
  }
}
