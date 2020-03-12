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

import com.ctre.phoenix.motorcontrol.ControlMode; 
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;


/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  //autonomous
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private static final String kMoveShootAuto = "move then shoot";
  private static final String kShootMoveAuto = "shoot then move";
  private static final String kMoveOffLine = "move off line";
  private static final String kTargetShootAuto = "target then shoot auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  //CAN IDs
  private static final int leftBackID = 1;
  private static final int leftFrontID = 2;
  private static final int rightBackID = 3;
  private static final int rightFrontID = 4;
  private static final int topShooterID = 5;
  private static final int bottomShooterID = 6;
  private static final int beltID = 9;
  private static final int intakeID = 10;
  private static final int lWinchID = 8;
  private static final int rWinchID = 7;
  private static final int intakeMoveID = 11;
  private static final int colorWheelID = 12;

  //motor speeds
  private double intakeSpeed = -0.3;
  private double beltSlow = -0.5; // -0.56, -0.45
  private double topSpeed = 0.55; //0.55
  private double bottomSpeed = -0.65;
  private double topFastSpeed = 0.75;
  private double bottomFastSpeed = -0.6;
  private double colorWheelSpeed = 0.25;
  private double winchSpeed = -1;
  private double winchSlow = -0.3;

  //ball motors
  private CANSparkMax topShooter;
  private CANSparkMax bottomShooter;
  private WPI_VictorSPX beltUp;
  private WPI_VictorSPX intakeIn;

  //lift motors
  private WPI_VictorSPX lWinch;
  private WPI_VictorSPX rWinch;

  //intake move up/down motor
  private WPI_VictorSPX intakeMove;

  //color wheel motor
  private WPI_VictorSPX colorWheel;

  //drive motors
  private CANSparkMax leftBack;
  private CANSparkMax leftFront;
  private CANSparkMax rightBack;
  private CANSparkMax rightFront;

  //drive
  private DifferentialDrive drive;

  //joysticks
  private Joystick leftJoy; //left drive control
  private Joystick rightJoy; //right drive control
  private Joystick videogame; //ball & lift control -- button mapping below
  /** buttons & sicks for videogame controller -- SAM!!
  * #1: belts backwards
  * #6: shooter fast
  * #7: shooter
  * #8: belts
  * #5 & #3: winch up/down
  * #5 & #3: winch up/down SLOWLY
  * #5 & left stick: lift up/down
  * right stick y-axis: intake in/out
  * right stick x-axis: color wheel
  * left stick y-axis: intake up/down
  */

  //limit switches
  private DigitalInput intakeLimit, beltLimit;

  //timers
  private Timer time;
  private Timer lightsTime;

  //camera values
  private static final int IMG_WIDTH = 320;
  private static final int IMG_HEIGHT = 240;

  //colors
  DriverStation ds = DriverStation.getInstance();
  DriverStation.Alliance alliance;
  byte[] color;
  private boolean colorsOn = false;

  private int ballCount = 0;

  //arduino/LED control
  private SerialPort arduino;


  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    //autonomous
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    m_chooser.addOption("move then shoot", kMoveShootAuto);
    m_chooser.addOption("shoot then move", kShootMoveAuto);
    m_chooser.addOption("move off line", kMoveOffLine);
    m_chooser.addOption("traget then shoot auto", kTargetShootAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    //motor contollers
    topShooter = new CANSparkMax(topShooterID, MotorType.kBrushless);
    bottomShooter = new CANSparkMax(bottomShooterID, MotorType.kBrushless);
    beltUp = new WPI_VictorSPX(beltID);
    intakeIn = new WPI_VictorSPX(intakeID);
    leftBack = new CANSparkMax(leftBackID, MotorType.kBrushless);
    leftFront = new CANSparkMax(leftFrontID, MotorType.kBrushless);
    rightBack = new CANSparkMax(rightBackID, MotorType.kBrushless);
    rightFront = new CANSparkMax(rightFrontID, MotorType.kBrushless);
    lWinch = new WPI_VictorSPX(lWinchID);
    rWinch = new WPI_VictorSPX(rWinchID);
    colorWheel = new WPI_VictorSPX(colorWheelID);
    intakeMove = new WPI_VictorSPX(intakeMoveID);

    //photoelectric sensors
    intakeLimit = new DigitalInput(2);
    beltLimit = new DigitalInput(3);

    //for whatever reason all the motors are inverted
    rightFront.setInverted(true);
    rightBack.setInverted(true);
    leftBack.setInverted(true);
    leftFront.setInverted(true);

    //break not coast: B/C LED --> blue = break mode, pink = coast mode
    rightFront.setIdleMode(IdleMode.kBrake);
    rightBack.setIdleMode(IdleMode.kBrake);
    leftBack.setIdleMode(IdleMode.kBrake);
    leftFront.setIdleMode(IdleMode.kBrake);

    //front motors just copy back motors -- only have to call the back motors in the code
    leftFront.follow(leftBack);
    rightFront.follow(rightBack);
    lWinch.follow(rWinch);

    //drivetrain -- tank drive
    drive = new DifferentialDrive(leftBack, rightBack);

    //joysticks
    leftJoy = new Joystick(1);
    rightJoy = new Joystick(0);
    videogame = new Joystick(2);

    //timer
    time = new Timer();

    //ball camera
    UsbCamera cameraBall = CameraServer.getInstance().startAutomaticCapture(); 
    cameraBall.setResolution(IMG_WIDTH, IMG_HEIGHT);

    //LEDs
    lightsTime = new Timer();
    lightsTime.start();

    //arduino USB port
    try {
      arduino = new SerialPort(9600, SerialPort.Port.kUSB);
      System.out.println("Connected to kUSB");
    } catch (Exception e) {
      System.out.println("failed to connect to kUSB, trying kUSB1");

      try {
        arduino = new SerialPort(9600, SerialPort.Port.kUSB1);
        System.out.println("Connected to kUSB1");
      } catch (Exception e1){
        System.out.println("failed to connect to kUSB1, trying kUSB2");

        try {
          arduino = new SerialPort(9600, SerialPort.Port.kUSB2);
          System.out.println("Connected to kUSB2");
        } catch (Exception e2) {
          System.out.println("failed to connect to all USB ports");
        }
      }
    }

    //alliance color
    alliance = ds.getAlliance();
    if (alliance == DriverStation.Alliance.Red) {
      color = new byte[] {0x14};
    } else {
      color = new byte[] {0x15};
    }
  }

  /**
   * move towards target
   * @param xVal x value of tape
   */
  public void setMotor(double xVal) {
    double turnSpeed = 0.4; //speed

    xVal = xVal - 160;
    if (xVal > 0) {
      turnSpeed *= -1;
    }

    //right moves froward
    //left moves backward
    if (Math.abs(xVal) < 30) { //deadzone
      turnSpeed = 0;
    } /*else {
      turnSpeed = 0.00000019 * Math.pow((xVal), 3);
    }*/

    System.out.println("speed: " + turnSpeed);
    drive.tankDrive(turnSpeed, -turnSpeed);
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
    /*if (lightsTime.get() > 5) {
      System.out.println("Wrote to arduino");
      arduino.write(new byte[] {0x12}, 1);
      lightsTime.reset();
    }*/

    //never actually prints... why? -- did Brandt add print statements to his??
    if (arduino.getBytesReceived() > 0) {
      System.out.print(arduino.readString());
    }

     /*else {
      color = new byte[] {0x11};
    }*/
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
    System.out.println("Auto selected: " + m_autoSelected);

    //autonomous timer
    time.start();

    //set base LED color
    if (alliance == DriverStation.Alliance.Red) {
      color = new byte[] {0x14};
    } else {
      color = new byte[] {0x15};
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    //x-position of tape
    double xPos = NetworkTableInstance.getDefault().getTable("tape").getEntry("tapeTurn").getNumber(160.0).doubleValue();

    //pick auto mode...
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
        case kMoveShootAuto:
        //move off line, THEN shoot three balls
        if (time.get() <5) { //drive off line
          drive.tankDrive(0.25, 0.25);
        } else if (time.get() < 6) { //get shooter up to speed
          topShooter.set(topSpeed);
          bottomShooter.set(bottomSpeed);
          drive.tankDrive(0, 0);
        } else if (time.get() < 10) { //shoot three balls
          topShooter.set(topSpeed);
          bottomShooter.set(bottomSpeed);
          beltUp.set(beltSlow);
        } else { //done!
          topShooter.set(0);
          bottomShooter.set(0);
          beltUp.set(0);
          drive.tankDrive(0, 0);
        }
        break;

      case kShootMoveAuto:
        //shoot three balls, THEN move off line
        if (time.get() <1) { //get shooter up to speed
          topShooter.set(topSpeed);
          bottomShooter.set(bottomSpeed);
        } else if (time.get() < 5) { //shoot three balls
          topShooter.set(topSpeed);
          bottomShooter.set(bottomSpeed);
          beltUp.set(beltSlow);
          drive.tankDrive(0, 0);
        } else if (time.get() < 10) { //drive off line
          topShooter.set(0);
          bottomShooter.set(0);
          beltUp.set(0);
          drive.tankDrive(0.25, 0.25);
        } else { //done!
          topShooter.set(0);
          bottomShooter.set(0);
          beltUp.set(0);
          drive.tankDrive(0, 0);
        }
        break;

      case kTargetShootAuto:
      default:
        //find the target, shoot three balls, move of line
        if (time.get() < 3) { //target
          setMotor(xPos);
        } else if (time.get() < 4){ //get motors up to speed
          topShooter.set(topSpeed);
          bottomShooter.set(bottomSpeed);
        } else if (time.get() < 8){ //shoot the balls
          topShooter.set(topSpeed);
          bottomShooter.set(bottomSpeed);
          beltUp.set(beltSlow);
          drive.tankDrive(0, 0);
        } else if (time.get() < 13) { //move off line
          topShooter.set(0);
          bottomShooter.set(0);
          beltUp.set(0);
          drive.tankDrive(0.25, 0.25);
        } else { //done!!
          topShooter.set(0);
          bottomShooter.set(0);
          beltUp.set(0);
          drive.tankDrive(0, 0);
        }
        break;

      case kMoveOffLine:
      //default: 
        //drive backward off line
        if (time.get() < 5) {
          drive.tankDrive(0.25, 0.25); //absolutely no idea why it's backwards -- the motors have already been inverted in robotInit
        } else {
          drive.tankDrive(0, 0);
        }
        break;

      /*case kDefaultAuto:
      default:
        // Put default auto code here
        break;*/
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    colorsOn = false;

    //x-position of tape
    double xPos = NetworkTableInstance.getDefault().getTable("tape").getEntry("tapeTurn").getNumber(160.0).doubleValue();
    System.out.println("xPos: " + (xPos-160)); //minus 160 so center is at (0,0)

    //motor speeds
    double leftSpeed = leftJoy.getY();
    double rightSpeed = rightJoy.getY();

    //quadratic rather than linear control
    leftSpeed = (leftSpeed * Math.abs(leftSpeed))/1.3;
    rightSpeed = (rightSpeed * Math.abs(rightSpeed))/1.3;

    //tank drive
    drive.tankDrive(leftSpeed, rightSpeed);

    //buttons
    Boolean ShooterRun = videogame.getRawButton(7);
    Boolean beltRun = videogame.getRawButton(8);
    Boolean beltBackwards = videogame.getRawButton(1);
    Boolean liftRun = videogame.getRawButton(5);
    Boolean shooterFast = videogame.getRawButton(6);
    Boolean colorWheelRun = videogame.getRawButton(2);
    Boolean winchRun = videogame.getRawButton(3);
    Boolean moveTowardTape = leftJoy.getRawButton(2);
    Boolean winchRunSlow = videogame.getRawButton(1);

    //TARGET: *LEFT JOYSTICK* button #2
    if (moveTowardTape) {
      setMotor(xPos); 
    } else {
      drive.tankDrive(leftSpeed, rightSpeed);
    }

    //BELTS: button #8
    if (beltRun) { //two-way belts
      beltUp.setInverted(false);
      beltUp.set(ControlMode.PercentOutput, beltSlow);
      arduino.write(new byte[] {0x13}, 1);
      colorsOn = true;
    } else if (beltBackwards && liftRun != true) {
    //BELTS BACKWARDS: button #1
    } else if (beltBackwards) {
      beltUp.setInverted(true);
      beltUp.set(ControlMode.PercentOutput, beltSlow);
      arduino.write(new byte[] {0x13}, 1);
      colorsOn = true;
    //new automatic belt system thingy w/limit switches
    } else if (intakeLimit.get() || beltLimit.get()) {
      if (ballCount < 2) {
        beltUp.setInverted(false);
        beltUp.set(ControlMode.PercentOutput, -0.1);
        intakeIn.set(ControlMode.PercentOutput, 0.1); 
        ballCount ++;
      } else {
        beltUp.setInverted(false);
        beltUp.set(ControlMode.PercentOutput, beltSlow); 
        intakeIn.set(ControlMode.PercentOutput, 0.1);
        arduino.write(new byte[] {0x13}, 1);      
        colorsOn = true;
        ballCount ++;
      }
    } else {
      beltUp.set(0);
      //colorsOn = false;
    }

    //SHOOTER: button #7
    if (ShooterRun) {
      arduino.write(new byte[] {0x12}, 1);
      topShooter.set(topSpeed);
      bottomShooter.set(bottomSpeed);
      colorsOn = true;
      ballCount = 0;
    //SHOOTER FAST: button #6
    } else if (shooterFast){
      arduino.write(new byte[] {0x12}, 1);
      topShooter.set(topFastSpeed);
      bottomShooter.set(bottomFastSpeed);
      colorsOn = true;
      ballCount = 0;
    } else {
      topShooter.set(0);
      bottomShooter.set(0);
    }

    //INTAKE IN/OUT: right stick y-axis
    double speedIn = videogame.getRawAxis(3);
    if (speedIn >= 0.1){
      speedIn = intakeSpeed * -1;
    } else if (speedIn <= -0.1) {
      speedIn = intakeSpeed;
    } else {
      speedIn = 0;
    }
    intakeIn.set(ControlMode.PercentOutput, speedIn);

    //COLOR WHEEL: button #2
    if (colorWheelRun) {
      colorWheel.set(colorWheelSpeed);
    } else {
      colorWheel.set(0);
    }

    //INTAKE UP/DOWN: left stick y-axis
    if (liftRun != true){
      intakeMove.set(ControlMode.PercentOutput, videogame.getY());
    } else {
      intakeMove.set(0);
    }

    //WINCH UP/DOWN: button #3 & button #5
    if (winchRun) {
      if (liftRun) {
        rWinch.set(ControlMode.PercentOutput, winchSpeed);
      } else {
        rWinch.set(0);
      }
    //WINCH UP/DOWN SLOWLY: button #1 & button #5
    } else if (winchRunSlow){
      if (liftRun) {
        rWinch.set(ControlMode.PercentOutput, winchSlow);
      } else {
        rWinch.set(0);
      }
    } else {
      rWinch.set(0);
    }

    //return to base colors
    if (colorsOn == false) {
      arduino.write(color, 1);
    }
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
