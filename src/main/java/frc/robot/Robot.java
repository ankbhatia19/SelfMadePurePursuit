/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.*;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  Notifier purePursuitRunner;
  Drivetrain drive;

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
    drive = new Drivetrain();

    PurePursuitHandler.addPoint(new Point(0, 0));
    PurePursuitHandler.addPoint(new Point(2, 0));
    PurePursuitHandler.addPoint(new Point(4,4));

    purePursuitRunner = new Notifier(() ->
    {
      Odometry robotOdo = drive.getOdo();
      Point robotPosition = new Point((float) robotOdo.getX(), (float) robotOdo.getY());
      float robotAngle = (float) robotOdo.getTheta();
      Point lookaheadPoint = PurePursuitHandler.getLookaheadPoint(robotPosition);
      double curvature = PurePursuitHandler.calculateCurvature(robotAngle, robotPosition, lookaheadPoint);
      double[] velocities = PurePursuitHandler.getTargetVelocities(curvature, (float) Constants.maxVelocity, (float) Constants.wheelBaseWidth);
      drive.setFPS(velocities[0] , velocities[1]);
      System.out.println(velocities[0] + ", " + velocities[1]);
    });

  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    SmartDashboard.putString("Odometry", drive.getOdo().toString());
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to
   * the switch structure below with additional strings. If using the
   * SendableChooser make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    //System.out.println("Auto selected: " + m_autoSelected);
    purePursuitRunner.startPeriodic(0.01);
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {

    if (PurePursuitHandler.isFinished()){
      drive.setSpeeds(0, 0);
      purePursuitRunner.stop();
      purePursuitRunner.close();
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
    drive.setFPS(2, 2);
  }
}
