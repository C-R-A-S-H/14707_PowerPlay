// package org.firstinspires.ftc.teamcode;

// import org.firstinspires.gtc.teamcode.Management.Resourcemanager;
// import org.firstinspires.gtc.teamcode.sensors.sensors;
// import org.firstinspires.gtc.teamcode.Drive;
// import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
// import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
// import com.qualcomm.robotcore.util.ElapsedTime;
// import com.qualcomm.robotcore.util.Range;
// import java.util.concurrent.TimeUnit;
// import com.qualcomm.robotcore.hardware.Blinker;
// import com.qualcomm.robotcore.hardware.HardwareDevice;
// import com.qualcomm.robotcore.hardware.DcMotor;
// import com.qualcomm.robotcore.hardware.Servo;
// import com.qualcomm.robotcore.hardware.ColorSensor;
// import com.qualcomm.robotcore.hardware.DistanceSensor;
// import com.qualcomm.robotcore.hardware.Gyroscope;
// import com.qualcomm.robotcore.hardware.DigitalChannel;

// @TeleOp

// public class Drive extends LinearOpMode {
//     public HardwareDevice webcam_1;
//     public ElapsedTime runtime = new ElapsedTime();
//     public DcMotor LFD;
//     public DcMotor LBD;
//     public DcMotor RFD;
//     public DcMotor RBD;
  
//     public DcMotor armR;
//     public Servo claw;
//     public Servo claw2;
    
//     private ColorSensor color;
//     private DigitalChannel touch;
//     private Gyroscope imu;
//     private DigitalChannel sensor_digital;
    
//     public final static double CLAW_HOME = 0.1;     // Starting position for claw 
//     public final static double CLAW_MIN_RANGE = 0.15;  // Smallest number value allowed for claw position
//     public final static double CLAW_MAX_RANGE = 0.8;  // Largestnumber value allowed for claw position
    
//     double clawPosition = CLAW_HOME;  // Sets safe position
//     final double CLAW_SPEED = 0.7 ;  // Sets rate to move servo
    
//     public final static double CLAW2_HOME = 0.6;     // Starting position for claw 
//     public final static double CLAW2_MIN_RANGE = 0.10;  // Smallest number value allowed for claw position
//     public final static double CLAW2_MAX_RANGE = 0.7;  // Largestnumber value allowed for claw position
    
//     double claw2Position = CLAW2_HOME;  // Sets safe position
//     final double CLAW2_SPEED = 1 ;  // Sets rate to move servo
    
//     @Override
//     public void runOpMode() {
        
//         touch = hardwareMap.get(DigitalChannel.class, "sensor_digital");
//         color = hardwareMap.get(ColorSensor.class, "color");
//         LFD  = hardwareMap.get(DcMotor.class, "left front drive");
//         LBD  = hardwareMap.get(DcMotor.class, "left back drive");
//         RFD = hardwareMap.get(DcMotor.class, "right front drive");
//         RBD = hardwareMap.get(DcMotor.class, "right back drive");
//         //turret = hardwareMap.get(DcMotor.class, "turret");
//         armR = hardwareMap.get(DcMotor.class, "arm right");
//         claw = hardwareMap.get(Servo.class, "claw");
//         claw2 = hardwareMap.get(Servo.class, "claw2");

//         LFD.setDirection(DcMotor.Direction.REVERSE);
//         LBD.setDirection(DcMotor.Direction.REVERSE);
//         RFD.setDirection(DcMotor.Direction.FORWARD);
//         RBD.setDirection(DcMotor.Direction.FORWARD);
//         touch.setMode(DigitalChannel.Mode.INPUT);
//         //turret.setDirection(DcMotor.Direction.FORWARD);
        
//         LFD.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//         LBD.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//         RFD.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//         RBD.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//         armR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
//         claw.setPosition(CLAW_HOME);
//         claw2.setPosition(CLAW2_HOME);
        
//         telemetry.addData("Status", "Initialized");
//         telemetry.addData(">", "Press Start to scan Servo." );
//         telemetry.update();
        
//         ResetEncoders();
        
//         waitForStart();
//         runtime.reset();

//         // run until the end of the match (driver presses STOP)
//         while (opModeIsActive()) {
//             if (gamepad2.x) {
//                 clawPosition += CLAW_SPEED;
//                 claw2Position -= CLAW2_MIN_RANGE;
//             }
//             else if (gamepad2.a) {
//                 clawPosition -= CLAW_SPEED;
//                 claw2Position += CLAW2_MAX_RANGE;
//             }
            
//             clawPosition = Range.clip(clawPosition, CLAW_MIN_RANGE, CLAW_MAX_RANGE);
//             claw.setPosition(clawPosition);
            
//             claw2Position = Range.clip(claw2Position, CLAW2_MIN_RANGE, CLAW2_MAX_RANGE);
//             claw2.setPosition(claw2Position);
            
//             // Display the current value
            
//             // if (gamepad2.left_bumper) {
//             //     ArmRunUsingEncoders();
//             //     Arm(180);
//             //     ArmPower(1);
//             //     ArmRunToPosition();
//             //     // HoldPos(0.05);
//             //     TelemetryL();
//             //     sleep(10);
//             // // } else if (gamepad2.left_bumper = false) {
//             // //     // double armPower;
//             // // armR.setPower(armPower);
//             // //      //ArmStopUsingEncoders();
//             // // HoldPos(0);
//             // // armPower = (1 * gamepad2.left_stick_y);
//             // // Touch();
//             // // armPower = (1 * -gamepad2.left_stick_y);
//             // // // HoldPos(0.2);
//             // // Touch();
//             // // Telemetry();
            
//             // }
//             /*telemetry.addData(">", "Press Stop to end test." );
//             telemetry.update();*/

//             idle();
//             StopUsingEncoders();
//             double max;
            
           
            
//             double axial   =  -gamepad1.left_stick_y; // forward and backward
//             double lateral = (0.7 * -gamepad1.right_stick_x); // turning left and right
//             double yaw     =  -gamepad1.left_stick_x; // stafing
//             //double turns = gamepad1.right_stick_x;
//             //double turnss = -gamepad1.right_stick_x;
            
//             // Combine the joystick requests for each axis-motion to determine each wheel's power.
//             // Set up a variable for each drive wheel to save the power level for telemetry.
            
//             double RFPower = axial + lateral + yaw;
//             double LFPower = axial - lateral - yaw;
//             double LBPower = axial - lateral + yaw;
//             double RBPower = axial + lateral - yaw;
            
            
//             LFD.setPower(LFPower);
//             RFD.setPower(RFPower);
//             LBD.setPower(LBPower);
//             RBD.setPower(RBPower);
            
            
//             //RFPower = Range.clip(turns, -0.5, 0.5) ; 
//             //RBPower = Range.clip(turnss, -0.5, 0.5) ; 
//             //LFPower = Range.clip(turnss, 0.5, -0.5) ;
//             //LBPower = Range.clip(turns, 0.5, -0.5) ;
            
//             // Normalize the values so no wheel power exceeds 100%
//             // This ensures that the robot maintains the desired motion.
//             /*max = Math.max(Math.abs(LFPower), Math.abs(RFPower));
//             max = Math.max(max, Math.abs(LFPower));
//             max = Math.max(max, Math.abs(RFPower));
//             max = Math.max(max, Math.abs(LBPower));
//             max = Math.max(max, Math.abs(RBPower));

//             if (max > 1.0) {
//                 LFPower = max;
//                 RFPower = max;
//                 LBPower = max;
//                 RBPower = max;
//             }*/
            
//             // This is test code:
//             //
//             // Uncomment the following code to test your motor directions.
//             // Each button should make the corresponding motor run FORWARD.
//             //   1) First get all the motors to take to correct positions on the robot
//             //      by adjusting your Robot Configuration if necessary.
//             //   2) Then make sure they run in the correct direction by modifying the
//             //      the setDirection() calls above.
//             // Once the correct motors move in the correct direction re-comment this code.
            
//             /*
//             leftFrontPower  = gamepad1.x ? 1.0 : 0.0;  // X gamepad
//             leftBackPower   = gamepad1.a ? 1.0 : 0.0;  // A gamepad
//             rightFrontPower = gamepad1.y ? 1.0 : 0.0;  // Y gamepad
//             rightBackPower  = gamepad1.b ? 1.0 : 0.0;  // B gamepad
//             */
//             //double turretPower;
//             double armPower;
            
//             //double drive = gamepad2.right_trigger;
//             //double turn  =  -gamepad2.left_stick_y;
//             //turretPower    = Range.clip(drive, -1.0, 1.0) ;
//             //armPower   = Range.clip(turn, -1, 1);
//             //armPower   = Range.clip(drive, -0.7, 0.7);
            
//             // if (gamepad2.left_bumper == true) {
//             //     ArmRunUsingEncoders();
//             //     Arm(180);
//             //     ArmPower(1);
//             //     ArmRunToPosition();
//             //     // HoldPos(0.05);
//             //     TelemetryN();
//             //     sleep(10);
//             // } else if (gamepad2.left_bumper == false) {
                
//             // } /*else if (gamepad2.left_bumper) {
//             //     RunUsingEncoders();
//             //     HoldPos(0);
//             //     Arm(2900, 1);
//             //     RunToPosition();
//             //     Power(0);
//             //     TelemetryL();
//             //     HoldPos(0.1);
//             //     TelemetryL();
//             // } else if (gamepad2.b) {
//             //     RunUsingEncoders();
//             //     HoldPos(0);
//             //     Arm(0, 1);
//             //     RunToPosition();
//             //     Power(0);
//             //     TelemetryL();
//             //     HoldPos(0.1);
//             //     TelemetryL();
//             // }*/
            
//             ArmStopUsingEncoders();
//             HoldPos(0);
//             armPower = (1 * gamepad2.left_stick_y);
//             Touch();
//             armPower = (1 * -gamepad2.left_stick_y);
//             HoldPos(0.2);
//             Touch();
//             //Telemetry();
            
            
//             armR.setPower(armPower);
            
            
//             /*LFPosition && RFPosition && LBPosition && RBPosition = (1 * -gamepad2.left_stick_y);
            
//             LFPosition && RFPosition && LBPosition && RBPosition = (1 * -gamepad2.right_stick_x);*/
            
            
//             /*LFD.setTargetPosition(LFPosition);
//             RFD.setTargetPosition(RFPosition);
//             LBD.setTargetPosition(LBPosition);
//             RBD.setTargetPosition(RBPosition);
//             arm.setTargetPosition(armPosition);*/
            
            

//             //double time;
//             //Telemetry();
//             // Show the elapsed game time and wheel power.
            
            
//                 telemetry.addData("LF Encoder:", LFD.getCurrentPosition());
//                 telemetry.addData("RF Encoder:", RFD.getCurrentPosition());
//                 telemetry.addData("LB Encoder:", LBD.getCurrentPosition());
//                 telemetry.addData("RB Encoder:", RBD.getCurrentPosition());
//                 telemetry.addData("Arm Encoder", armR.getCurrentPosition());
//                 /*telemetry.addData("Left/Right", "%4.2f",  "%4.2f", LFPower, RFPower);
//                 telemetry.addData("Left/Right", "%4.2f",  "%4.2f", LBPower, RBPower);*/
//                 telemetry.addData("LF Power ", "%.2f", LFPower);
//                 telemetry.addData("RF Power", "%.2f", RFPower);
//                 telemetry.addData("LB Power", "%.2f", LBPower);
//                 telemetry.addData("RB Power", "%.2f", RBPower);
//                 telemetry.addData("Arm Power", "%.2f", armPower);
//                 telemetry.addData("Status", "Run Time: " + runtime.time(TimeUnit.MINUTES));
//                 //telemetry.addData("Color", "%1.0f", color.argb());
//                 //telemetry.addData("color", color.getrgba);
//                 telemetry.update();
//                 /*telemetry.addData("LF-Encoder/Power", "%4.2f", LFD.getCurrentPosition(), LFPower);
//                 telemetry.addData("RF-Encoder/Power", "%4.2f", RFD.getCurrentPosition(), RFPower);
//                 telemetry.addData("LB-Encoder/Power", "%4.2f", LBD.getCurrentPosition(), LBPower);
//                 telemetry.addData("RB-Encoder/Power", "%4.2f", RBD.getCurrentPosition(), RBPower);
//                 telemetry.addData("Arm-Encoder/Power", "%4.2f", armR.getCurrentPosition(), armPower);
//                 telemetry.addData("Status", "Run Time: " + runtime.toString());
//                 telemetry.update();*/
//                 /*telemetry.addData("Front left/Right", "%4.2f, %4.2f", LFPower, RFPower);
//                 telemetry.addData("Back  left/Right", "%4.2f, %4.2f", LBPower, RBPower);
//                 telemetry.addData("Motors", "arm (%.2f)", armPower);
//                 telemetry.update();*/
//         }
//     }
// }
