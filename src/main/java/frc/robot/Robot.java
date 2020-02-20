package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.WaitCommand;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.networktables.*;

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

  private Spark m_Left0 = new Spark(1);

  private Spark m_Right0 = new Spark(2);

  private SpeedControllerGroup m_LeftMotors = new SpeedControllerGroup(m_Left0);
  private SpeedControllerGroup m_RightMotors = new SpeedControllerGroup(m_Right0);
  private DifferentialDrive m_Drive = new DifferentialDrive(m_LeftMotors,m_RightMotors);

  private XboxController m_Controller = new XboxController(0);

  private boolean m_LimelightHasValidTarget = false;
  private double m_LimelightDriveCommand = 0.0;
  private double m_LimelightDriveCommandFar = 0.0;
  private double m_LimelightSteerCommand = 0.0;
  private double m_LimelightHeadingCommand = 0.0;
  private double m_LimelightSkewCommand = 0.0;

  private double DESIRED_HEADING;
  private double DESIRED_TARGET_AREA;

  private double tv;
  private double tx;
  private double ty;
  private double ta;
  private double ts;

  /*


    First - goto Skew 0
    Second - Store tx, ty, ta
    Third - calculate something - changes based on which step four
    Four - act on said calculation
      a) Rotate and Drive in a straight path to the correct area
      b) Correct x, then go straight - Do first, use NAVX to turn 90*, Step Three - TX + NAVX Angle, drive a distance (tx * Constant)
      c) Piecewise curve go over then stright |_ _| |_ _|

      Push a button, skew 0, find tx, calculate drive tx, rotate 90*, drive tx, rotate back 90*


  */

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
   
    
        m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
        m_chooser.addOption("My Auto", kCustomAuto);
        SmartDashboard.putData("Auto choices", m_chooser);
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
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {

      

        Update_Limelight_Tracking();

        System.out.println(ta);

        double left = m_Controller.getY(Hand.kRight);
        double right = m_Controller.getY(Hand.kLeft);
        boolean Y = m_Controller.getYButton();
        boolean autoFar = false;
        boolean A = m_Controller.getAButton();
        boolean X = m_Controller.getXButton();
        boolean S = m_Controller.getStartButton();
 

        left *= 0.70;
        right *= 0.70;


        if (Y)
        {
          if (m_LimelightHasValidTarget)
          {
  
                if(Math.abs(ty) > 0){
                  
                  m_Drive.arcadeDrive(-m_LimelightDriveCommand,0.0);
                }
          }
          else
          {
                m_Drive.arcadeDrive(0.0,0.0);
          }
        }
        else if (X){


          if (m_LimelightHasValidTarget)
          {
         

             if (Math.abs(tx) > 0){
              m_Drive.arcadeDrive(0.0, -m_LimelightSteerCommand);
             }

          
            }
          else
          {
                m_Drive.arcadeDrive(0.0,0.0);
          }

        }
        else if (A){

          if (m_LimelightHasValidTarget)
          {

            m_Drive.arcadeDrive(-m_LimelightHeadingCommand, 0.0);

          }


        }
        else if (S){

          if (m_LimelightHasValidTarget)
          {

            m_Drive.arcadeDrive(0.0, m_LimelightSkewCommand);

          }

        }
        else if (autoFar){

          if (m_LimelightHasValidTarget)
          {
            
                m_Drive.arcadeDrive(m_LimelightDriveCommandFar,-m_LimelightSteerCommand);
          }
          else
          {
                m_Drive.arcadeDrive(0.0,0.0);
          }

        }

        
        else
        {
          m_Drive.tankDrive(left,right);
        }
  
}

  @Override
  public void testPeriodic() {
  }

  /**
   * This function implements a simple method of generating driving and steering commands
   * based on the tracking data from a limelight camera.
   */
  public void Update_Limelight_Tracking()
  {


        // These numbers must be tuned for your Robot!  Be careful!
        final double STEER_K = 0.1;                    // how hard to turn toward the target
        final double SKEW_K = 0.04;
        final double DRIVE_K = 0.26;
                  // how hard to drive fwd toward the target
        DESIRED_TARGET_AREA = 0.0;        // Area of the target when the robot reaches the wall
        final double DESIRED_TARGET_AREA_FAR = -32.0;
        DESIRED_HEADING = 3.8;
        final double MAX_DRIVE = 0.7;                   // Simple speed limit so we don't drive too fast

        // double KpAim = -0.05f;
        // double KpDistance = -0.05f;
        // double min_aim_command = 0.075f;
        // double steering_adjust = 0.0f;
          

         tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
         tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
         ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
         ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);
         ts = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ts").getDouble(0);
        




        // double heading_error = -tx;
        // double distance_error = -ty;

        // if (tx > 1.0)
        // {
        //         steering_adjust = KpAim*heading_error - min_aim_command;
        // }
        // else if (tx < 1.0)
        // {
        //         steering_adjust = KpAim*heading_error + min_aim_command;
        // }

        // double distance_adjust = KpDistance * distance_error;

        // left_command += steering_adjust + distance_adjust;
        // right_command -= steering_adjust + distance_adjust;

        if (tv < 1.0)
        {
          m_LimelightHasValidTarget = false;
          m_LimelightDriveCommand = 0.0;
          m_LimelightSteerCommand = 0.0;
          return;
        }

        m_LimelightHasValidTarget = true;

        // Start with proportional steering
        double steer_cmd = tx * STEER_K;
        m_LimelightSteerCommand = steer_cmd;

        double skew_cmd = ts * SKEW_K;
        m_LimelightSkewCommand = skew_cmd;

     

        // double farsteer_cmd = tx * STEER_K;
        // m_LimelightSteerCommandFar = -farsteer_cmd;

        // try to drive forward until the target area reaches our desired area
        double drive_cmd = (DESIRED_TARGET_AREA - ty) * DRIVE_K;

        double fardrive_cmd = (DESIRED_TARGET_AREA_FAR - ty) * DRIVE_K;
        double heading_cmd =  ta * DRIVE_K;


        // if(ty < DESIRED_TARGET_AREA_FAR){

        //   double fardrive_cmd = (DESIRED_TARGET_AREA_FAR + ty) * DRIVE_K;
        //   if (fardrive_cmd > MAX_DRIVE)
        // {
        //   fardrive_cmd = MAX_DRIVE;
        // }
        //   m_LimelightDriveCommandFar = fardrive_cmd;

        // }else if (ty > DESIRED_TARGET_AREA_FAR){
        //   double fardrive_cmd = (DESIRED_TARGET_AREA_FAR - ty) * DRIVE_K;
        //   if (fardrive_cmd > MAX_DRIVE)
        // {
        //   fardrive_cmd = MAX_DRIVE;
        // }
        //   m_LimelightDriveCommandFar = fardrive_cmd;
        // }

        // don't let the robot drive too fast into the goal
        if (drive_cmd > MAX_DRIVE)
        {
          drive_cmd = MAX_DRIVE;
        }
        m_LimelightDriveCommand = drive_cmd;
        
        if (heading_cmd > MAX_DRIVE){

          heading_cmd = MAX_DRIVE;
        }     
        m_LimelightHeadingCommand = heading_cmd;     




        if (-fardrive_cmd > MAX_DRIVE)
        {
          fardrive_cmd = -MAX_DRIVE;
        }
          m_LimelightDriveCommandFar = -fardrive_cmd;
  }
}