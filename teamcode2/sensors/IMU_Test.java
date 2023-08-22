/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package sensors;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.ColorSensor;

import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Locale;

/**
 * {@link SensorBNO055IMU} gives a short demo on how to use the BNO055 Inertial Motion Unit (IMU) from AdaFruit.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 *
 * @see <a href="http://www.adafruit.com/products/2472">Adafruit IMU</a>
 */
@Autonomous(name = "Sensor: BNO055 IMU", group = "Sensor")

public class IMU_Test extends LinearOpMode
    {
    //----------------------------------------------------------------------------------------------
    // State
    //----------------------------------------------------------------------------------------------

    // The IMU sensor object
    public BNO055IMU imu;
    public BNO055IMU.Parameters imuParameters;

    // State used for updating telemetry
    public Orientation angles;
    public Acceleration gravity;
    
    public ElapsedTime Elapsed_Time;

    //----------------------------------------------------------------------------------------------
    // Main logic
    //----------------------------------------------------------------------------------------------
    
    private ColorSensor color;
    private Blinker expansion_Hub_1;
    private Blinker expansion_Hub_2;
    private HardwareDevice webcam_1;
    private DcMotor arm;
    private Servo claw2;
    private Servo claw;
    private DistanceSensor distance;
    private DcMotor LBD;
    private DcMotor LFD;
    private DcMotor RBD;
    private DcMotor RFD;
    private DcMotor turret;
   

    // todo: write your code here
    public final static double CLAW_HOME = 0.1;     // Starting position for claw 
    public final static double CLAW_MIN_RANGE = 0.15;  // Smallest number value allowed for claw position
    public final static double CLAW_MAX_RANGE = 0.8;  // Largestnumber value allowed for claw position
    
    double clawPosition = CLAW_HOME;  // Sets safe position
    final double CLAW_SPEED = 0.4 ;  // Sets rate to move servo
    
    public final static double CLAW2_HOME = 0.5;     // Starting position for claw 
    public final static double CLAW2_MIN_RANGE = 0.10;  // Smallest number value allowed for claw position
    public final static double CLAW2_MAX_RANGE = 0.7;  // Largestnumber value allowed for claw position
    
    double claw2Position = CLAW2_HOME;  // Sets safe position
    final double CLAW2_SPEED = 0.5 ;  // Sets rate to move servo
    
    float Yaw_Angle;
    
    public void RunToPosition() {
        RFD.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RBD.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LFD.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LBD.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void ResetDriveEncoders() {
        LFD.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LBD.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RFD.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RBD.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void ResetArmEncoder() {
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void ResetEncoders() {
        LFD.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LBD.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RFD.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RBD.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void Power(double Power) {
        LFD.setPower(Power);
        LBD.setPower(Power);
        RFD.setPower(Power);
        RBD.setPower(Power);
    }
    public void ArmPower(double Power) {
        arm.setPower(Power);
    }
   
    public void Arm(int Pos5) {
        arm.setTargetPosition(Pos5);
        
    }
    public void ArmRuntoPsoition() {
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void StopUsingEncoders() {
        LFD.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LBD.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RFD.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RBD.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void RunUsingEncoders() {
        LFD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LBD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RFD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RBD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void MDrive(int Pos1, int Pos2, int Pos3, int Pos4) {
        LFD.setTargetPosition(Pos1);
        RFD.setTargetPosition(Pos2);
        LBD.setTargetPosition(Pos3);
        RBD.setTargetPosition(Pos4);
    }
    // public void MDriveCurrent(int p1, int p2, int p3, int p4) {
    //     LFD.getTargetPosition(p1);
    //     RFD.getTargetPosition(p2);
    //     LBD.getTargetPosition(p3);
    //     RBD.getTargetPosition(p4);
    // }
    public void Drive(int Pos1, int Pos2) {
        LFD.setTargetPosition(Pos1);
        RFD.setTargetPosition(Pos2);
        LBD.setTargetPosition(Pos1);
        RBD.setTargetPosition(Pos2);
    }
    public void Velocity(double Power) {
        ((DcMotorEx) LFD).setVelocity(Power);
        ((DcMotorEx) LBD).setVelocity(Power);
        ((DcMotorEx) RFD).setVelocity(Power);
        ((DcMotorEx) RBD).setVelocity(Power);
        ((DcMotorEx) arm).setVelocity(Power);
    }
    public void TelemetryL() {
        while (arm.isBusy()) {
            telemetry.addData("Stauts", "Low Junction");
            telemetry.update();
        }
    }
    public void TelemetryM() {
        while (arm.isBusy()) {
            telemetry.addData("Stauts", "Medium Junction");
            telemetry.update();
        }
    }
    public void TelemetryHi() {
        while (arm.isBusy()) {
            telemetry.addData("Stauts", "High Junction");
            telemetry.update();
        }
    }
    public void TelemetryH() {
        while (arm.isBusy()) {
            telemetry.addData("Stauts", "Home");
            telemetry.update();
        }
    }
    public void Telemetry() {
        while (opModeIsActive() && LFD.isBusy()) {   //leftMotor.getCurrentPosition() < leftMotor.getTargetPosition())
            telemetry.addData("encoder-fwd-left", LFD.getCurrentPosition() + "  busy=" + LFD.isBusy());
            telemetry.addData("encoder-fwd-right", RFD.getCurrentPosition() + "  busy=" + RFD.isBusy());
            telemetry.addData("encoder-bwd-left", LBD.getCurrentPosition() + "  busy=" + LBD.isBusy());
            telemetry.addData("encoder-bwd-right", RBD.getCurrentPosition() + "  busy=" + RBD.isBusy());
            telemetry.addData("encoder-arm", arm.getCurrentPosition() + "  busy=" + arm.isBusy());
            telemetry.update();
            //idle();
        }
    }
    public void Telemetry2() {
        while (opModeIsActive() && LFD.isBusy()) {   //leftMotor.getCurrentPosition() < leftMotor.getTargetPosition())
            telemetry.addData("encoder-fwd-left", LFD.getCurrentPosition() + "  busy=" + LFD.isBusy());
            telemetry.addData("encoder-fwd-right", RFD.getCurrentPosition() + "  busy=" + RFD.isBusy());
            telemetry.addData("encoder-bwd-left", LBD.getCurrentPosition() + "  busy=" + LBD.isBusy());
            telemetry.addData("encoder-bwd-right", RBD.getCurrentPosition() + "  busy=" + RBD.isBusy());
            telemetry.addData("encoder-arm", arm.getCurrentPosition() + "  busy=" + arm.isBusy());
            telemetry.update();
            //idle();
        }
    }
    public void ColorRed() {
            while (opModeIsActive()) {
                telemetry.addData("Red", color.red());
                telemetry.addData("Green", color.green());
                telemetry.addData("Blue", color.blue());
                
              
                
                telemetry.update();
                
                if (color.red() > color.green() && color.red() > color.blue()) {
                    ResetDriveEncoders();
                    MDrive(300, 0, 0, 300);
                    RunToPosition();
                    Power(0.6);
                    Telemetry2();
                    sleep(1000);
                }
            }
            
    }
    public void ColorGreen() {
            while (opModeIsActive()) {
                telemetry.addData("Red", color.red());
                telemetry.addData("Green", color.green());
                telemetry.addData("Blue", color.blue());
                
               
                
                telemetry.update();
                
                if (color.green() > color.red() && color.green() > color.blue()) {
                    ResetDriveEncoders();
                    MDrive(300, 300, 0, 0);
                    RunToPosition();
                    Power(0.6);
                    Telemetry2();
                    sleep(1000000);
                }
            }
            
    }
    public void ColorBlue() {
            while (opModeIsActive()) {
                telemetry.addData("Red", color.red());
                telemetry.addData("Green", color.green());
                telemetry.addData("Blue", color.blue());
                
                
                
                telemetry.update();
                
                if (color.blue() > color.green() && color.blue() > color.red()) {
                    ResetDriveEncoders();
                    MDrive(0, 0, 300, 300);
                    RunToPosition();
                    Power(0.6);
                    Telemetry2();
                    sleep(1000);
                }
            }
           
    }
    
    
    public void Color() {
            while (opModeIsActive()) {
                //color sensor telemetry
                telemetry.addData("Red", color.red());
                telemetry.addData("Green", color.green());
                telemetry.addData("Blue", color.blue());
                telemetry.update();
                
                // 
                if (color.red() > color.green() && color.red() > color.blue()) {
                    ResetDriveEncoders();
                    MDrive(0, 0, 0, 0);
                    RunToPosition();
                    Velocity(1);
                    Power(0.3);
                    Telemetry2();
                    
                    sleep(200);
                    
                    LFD.setPower(0);
                    RFD.setPower(0);
                    LBD.setPower(0);
                    RBD.setPower(0);
                    ResetDriveEncoders();
                    
                    sleep(200);
                    
                    ResetDriveEncoders();
                    MDrive(744, 744, 744, 744);
                    RunToPosition();
                    Velocity(1);
                    Power(0.3);
                    Telemetry2();
                    
                    sleep(400);
                    
                    LFD.setPower(0);
                    RFD.setPower(0);
                    LBD.setPower(0);
                    RBD.setPower(0);
                    ResetDriveEncoders();
                    
                    sleep(200);
                    
                    ResetDriveEncoders();
                    MDrive(-744, -744, -744, -744);
                    RunToPosition();
                    Velocity(1);
                    Power(0.3);
                    Telemetry2();
                    
                    sleep(400);
                    
                    LFD.setPower(0);
                    RFD.setPower(0);
                    LBD.setPower(0);
                    RBD.setPower(0);
                    ResetDriveEncoders();
                    
                    sleep(200);
                    
                    ResetDriveEncoders();
                    MDrive(388, -388, 388, -388);
                    RunToPosition();
                    Velocity(1);
                    Power(0.3);
                    Telemetry2();
                    
                    sleep(400);
                    
                    LFD.setPower(0);
                    RFD.setPower(0);
                    LBD.setPower(0);
                    RBD.setPower(0);
                    ResetDriveEncoders();
                    
                    sleep(200);
                    
                    MDrive(255, 255, 255, 255);/*  1116   1 inch = 62 */
                    RunToPosition();
                    Velocity(1);
                    Power(0.3); 
                    Telemetry2();
                    
                    sleep(1000);
                    
                    LFD.setPower(0);
                    RFD.setPower(0);
                    LBD.setPower(0);
                    RBD.setPower(0);
                    ResetDriveEncoders();
                    
                    Arm(600);
                    Velocity(0.5);
                    ArmPower(-0.6);
                    ArmRuntoPsoition();
                    TelemetryM();
                    
                    sleep(100);
                    
                    claw.setPosition(CLAW_HOME);
                    claw2.setPosition(CLAW2_HOME);
                    
                    sleep(1500);
                    
                    Arm(600);
                    Velocity(0.5);
                    ArmPower(0.6);
                    ArmRuntoPsoition();
                    TelemetryM();
                    
                    sleep(400);
                    
                    MDrive(-1244, 1244, -1244, 1244);/*  1116   1 inch = 62 */
                    RunToPosition();
                    Velocity(1);
                    Power(0.3); 
                    Telemetry2();
                    
                    sleep(1000);
                    
                    LFD.setPower(0);
                    RFD.setPower(0);
                    LBD.setPower(0);
                    RBD.setPower(0);
                    ResetDriveEncoders();
                    
                    sleep(700);
                    
                    MDrive(-1390, -1390, -1390, -1390);/*  1116   1inch = 62 */
                    RunToPosition();
                    Velocity(1);
                    Power(0.3);
                    Telemetry2();
                    
                    //sleep(3000);
                    
                    LFD.setPower(0);
                    RFD.setPower(0);
                    LBD.setPower(0);
                    RBD.setPower(0);
                    ResetDriveEncoders();
                    
                    
                    
                    sleep(10000000);
                } else if (color.green() > color.red() && color.green() > color.blue()) {
                    ResetDriveEncoders();
                    MDrive(0, 0, 0, 0);
                    RunToPosition();
                    Velocity(1);
                    Power(0.3);
                    Telemetry2();
                    
                    sleep(200);
                    
                    LFD.setPower(0);
                    RFD.setPower(0);
                    LBD.setPower(0);
                    RBD.setPower(0);
                    ResetDriveEncoders();
                    
                    sleep(200);
                    
                    ResetDriveEncoders();
                    MDrive(744, 744, 744, 744);
                    RunToPosition();
                    Velocity(1);
                    Power(0.3);
                    Telemetry2();
                    
                    sleep(400);
                    
                    LFD.setPower(0);
                    RFD.setPower(0);
                    LBD.setPower(0);
                    RBD.setPower(0);
                    ResetDriveEncoders();
                    
                    sleep(200);
                    
                    ResetDriveEncoders();
                    MDrive(-744, -744, -744, -744);
                    RunToPosition();
                    Velocity(1);
                    Power(0.3);
                    Telemetry2();
                    
                    sleep(400);
                    
                    LFD.setPower(0);
                    RFD.setPower(0);
                    LBD.setPower(0);
                    RBD.setPower(0);
                    ResetDriveEncoders();
                    
                    sleep(200);
                    
                    ResetDriveEncoders();
                    MDrive(388, -388, 388, -388);
                    RunToPosition();
                    Velocity(1);
                    Power(0.3);
                    Telemetry2();
                    
                    sleep(400);
                    
                    LFD.setPower(0);
                    RFD.setPower(0);
                    LBD.setPower(0);
                    RBD.setPower(0);
                    ResetDriveEncoders();
                    
                    sleep(200);
                    
                    MDrive(255, 255, 255, 255);/*  1116   1 inch = 62 */
                    RunToPosition();
                    Velocity(1);
                    Power(0.3); 
                    Telemetry2();
                    
                    sleep(1000);
                    
                    LFD.setPower(0);
                    RFD.setPower(0);
                    LBD.setPower(0);
                    RBD.setPower(0);
                    ResetDriveEncoders();
                    
                    Arm(900);
                    Velocity(0.5);
                    ArmPower(-0.6);
                    ArmRuntoPsoition();
                    TelemetryM();
                    
                    sleep(100);
                    
                    claw.setPosition(CLAW_HOME);
                    claw2.setPosition(CLAW2_HOME);
                    
                    sleep(1500);
                    
                    Arm(600);
                    Velocity(0.5);
                    ArmPower(0.6);
                    ArmRuntoPsoition();
                    TelemetryM();
                    
                    sleep(400);
                    
                    sleep(400);
                    
                    MDrive(-1244, 1244, -1244, 1244);/*  1116   1 inch = 62 */
                    RunToPosition();
                    Velocity(1);
                    Power(0.3); 
                    Telemetry2();
                    
                    sleep(1000);
                    
                    LFD.setPower(0);
                    RFD.setPower(0);
                    LBD.setPower(0);
                    RBD.setPower(0);
                    ResetDriveEncoders();
                    
                    sleep(700);
                    
                    MDrive(1400, 1400, 1400, 1400);/*  1116   1inch = 62 */
                    RunToPosition();
                    Velocity(1);
                    Power(0.3);
                    Telemetry2();
                    
                    //sleep(3000);
                    
                    LFD.setPower(0);
                    RFD.setPower(0);
                    LBD.setPower(0);
                    RBD.setPower(0);
                    ResetDriveEncoders();
                    
                    MDrive(944, -944, 944, -944);/*  1116   1inch = 62 */
                    RunToPosition();
                    Velocity(1);
                    Power(0.3);
                    Telemetry2();
                    
                    //sleep(3000);
                    
                    LFD.setPower(0);
                    RFD.setPower(0);
                    LBD.setPower(0);
                    RBD.setPower(0);
                    ResetDriveEncoders();
                    
                    sleep(1000000);
                } else if (color.blue() > color.green() && color.blue() > color.red()) {
                    ResetDriveEncoders();
                    MDrive(0, 0, 0, 0);
                    RunToPosition();
                    Velocity(1);
                    Power(0.3);
                    Telemetry2();
                    
                    sleep(200);
                    
                    LFD.setPower(0);
                    RFD.setPower(0);
                    LBD.setPower(0);
                    RBD.setPower(0);
                    ResetDriveEncoders();
                    
                    sleep(200);
                    
                    ResetDriveEncoders();
                    MDrive(744, 744, 744, 744);
                    RunToPosition();
                    Velocity(1);
                    Power(0.3);
                    Telemetry2();
                    
                    sleep(400);
                    
                    LFD.setPower(0);
                    RFD.setPower(0);
                    LBD.setPower(0);
                    RBD.setPower(0);
                    ResetDriveEncoders();
                    
                    sleep(200);
                    
                    ResetDriveEncoders();
                    MDrive(-744, -744, -744, -744);
                    RunToPosition();
                    Velocity(1);
                    Power(0.3);
                    Telemetry2();
                    
                    sleep(400);
                    
                    LFD.setPower(0);
                    RFD.setPower(0);
                    LBD.setPower(0);
                    RBD.setPower(0);
                    ResetDriveEncoders();
                    
                    sleep(200);
                    
                    ResetDriveEncoders();
                    MDrive(388, -388, 388, -388);
                    RunToPosition();
                    Velocity(1);
                    Power(0.3);
                    Telemetry2();
                    
                    sleep(400);
                    
                    LFD.setPower(0);
                    RFD.setPower(0);
                    LBD.setPower(0);
                    RBD.setPower(0);
                    ResetDriveEncoders();
                    
                    sleep(200);
                    
                    MDrive(255, 255, 255, 255);/*  1116   1 inch = 62 */
                    RunToPosition();
                    Velocity(1);
                    Power(0.3); 
                    Telemetry2();
                    
                    sleep(1000);
                    
                    LFD.setPower(0);
                    RFD.setPower(0);
                    LBD.setPower(0);
                    RBD.setPower(0);
                    ResetDriveEncoders();
                    
                    Arm(600);
                    Velocity(0.5);
                    ArmPower(-0.6);
                    ArmRuntoPsoition();
                    TelemetryM();
                    
                    sleep(100);
                    
                    claw.setPosition(CLAW_HOME);
                    claw2.setPosition(CLAW2_HOME);
                    
                    sleep(1500);
                    
                    Arm(600);
                    Velocity(0.5);
                    ArmPower(0.6);
                    ArmRuntoPsoition();
                    TelemetryM();
                    
                    sleep(400);
                    
                    
                    MDrive(-255, -255, -255, -255);/*  1116   1 inch = 62 */
                    RunToPosition();
                    Velocity(1);
                    Power(0.3); 
                    Telemetry2();
                    
                    sleep(1000);
                    
                    LFD.setPower(0);
                    RFD.setPower(0);
                    LBD.setPower(0);
                    RBD.setPower(0);
                    ResetDriveEncoders();
                    
                    sleep(300);
                    
                    MDrive(-388, 388, -388, 388);/*  1116   1 inch = 62 */
                    RunToPosition();
                    Velocity(1);
                    Power(0.3); 
                    Telemetry2();
                    
                    sleep(1000);
                    
                    LFD.setPower(0);
                    RFD.setPower(0);
                    LBD.setPower(0);
                    RBD.setPower(0);
                    ResetDriveEncoders();
                    
                    sleep(1000000);
                }
            }
    }
    @Override public void runOpMode() {

        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        color = hardwareMap.get(ColorSensor.class, "color");
        LFD  = hardwareMap.get(DcMotor.class, "left front drive");
        LBD  = hardwareMap.get(DcMotor.class, "left back drive");
        RFD = hardwareMap.get(DcMotor.class, "right front drive");
        RBD = hardwareMap.get(DcMotor.class, "right back drive");
        //turret = hardwareMap.get(DcMotor.class, "turret");
        arm = hardwareMap.get(DcMotor.class, "arm right");
        claw = hardwareMap.get(Servo.class, "claw");
        claw2 = hardwareMap.get(Servo.class, "claw2");

      LFD.setDirection(DcMotor.Direction.REVERSE);
        LBD.setDirection(DcMotor.Direction.REVERSE);
        RFD.setDirection(DcMotor.Direction.FORWARD);
        RBD.setDirection(DcMotor.Direction.FORWARD);
        //turret.setDirection(DcMotor.Direction.FORWARD);
        //arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LFD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LBD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RFD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RBD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        
        ResetEncoders();
        RunUsingEncoders();

        // Set up our telemetry dashboard
        composeTelemetry();
        
        // Create new IMU Parameters object.
        imuParameters = new BNO055IMU.Parameters();
        // Use degrees as angle unit.
        imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        // Express acceleration as m/s^2.
        imuParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        // Disable logging.
        imuParameters.loggingEnabled = false;
        // Initialize IMU.
        imu.initialize(imuParameters);
        // Prompt user to press start buton.
        telemetry.addData("IMU Example", "Press start to continue...");
        telemetry.update();

        // Wait until we're told to go
        waitForStart();

        // Start the logging of measured acceleration
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        
        Power(0);
        
        int Red; //= color.red();
        int Green; //= color.green();
        int Blue; //= color.blue();
        
        int color;
        
        // Loop and update the dashboard
        // Create a timer object with millisecond
    // resolution and save in ElapsedTime Variable
    Elapsed_Time = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    // Initialize motor power variables to 20%
    LFD.setPower(.2);
    LBD.setPower(.2);
    RFD.setPower(.2);
    RBD.setPower(.2);
    // Set motor powers to the variable values
    // LFD.setPower(LFPower);
    // LBD.setPower(LBPower);
    // RFD.setPower(RFPower);
    // RBD.setPower(RBPower);
    
    double LFPower;
    double LBPower;
    double RFPower;
    double RBPower;
    // Move robot forward for 30 second or until stop
    // is Pressed on Driver Station
    while (!(Elapsed_Time.milliseconds() >= 2000 || isStopRequested())) {
      // Save Gyro's yaw angle
    //   X_Angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).firstAngle;
    //   Y_Angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).secondAngle;
      Yaw_Angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
      // Report yaw orientation to Driver Station
      telemetry.addData("Yaw Value", Yaw_Angle);
      // If the robot is moving straight ahead the
      // yaw value will be close to zero. If it's not, we
      // need to adjust the motor powers to adjust heading.
      // If robot yaws right or left by 5 or more,
      // adjust motor power variables to compensation
      if (Yaw_Angle < -1) {
        // Turn left
        LFPower = 0.18;
        LBPower = 0.18;
        RFPower = 0.22;
        RBPower = 0.22;
      } else if (Yaw_Angle < 1) {
        // Turn right
        LFPower = 0.22;
        LBPower = 0.22;
        RFPower = 0.18;
        RBPower = 0.18;
      } else {
        // Continue forward
        LFPower = 0.2;
        LBPower = 0.2;
        RFPower = 0.2;
        RBPower = 0.2;
      }
      // Report the new power levels to the Driver Station
      telemetry.addData("LFD Power", LFPower);
      telemetry.addData("LBD Power", LBPower);
      telemetry.addData("RFD Power", RFPower);
      telemetry.addData("RBD Power", RBPower);
      // Update the motors to the new power levels
      LFD.setPower(LFPower);
      LBD.setPower(LBPower);
      RFD.setPower(RFPower);
      RBD.setPower(RBPower);
      telemetry.update();
      // Wait 1/10 second before checking again
      sleep(100);
    }
    LFPower = 0.2;
    LBPower = 0.2;
    RFPower = -0.2;
    RBPower = -0.2;
    while (!(Yaw_Angle <= -90 || isStopRequested())) {
      Yaw_Angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
      telemetry.addData("Yaw Angle", Yaw_Angle);
      telemetry.update();
    }
    // We're done. Turn off the motors
    Power(0);
        while (opModeIsActive()) {
        //     loop(
        //         if (!MDriveCurrent(1000, 1000, 1000, 1000)) {
        //             MDrive(1000, 1000, 1000, 1000);/* 1333 1116   1 inch = 62 */
        //             RunToPosition();
        //             while (RunToPosition()) {
        //                 angles.set
        //                 if (angles.firstAngle < 0.3) {
        //                     LFD.LynxSetMotorConstantPowerCommand(0.7);
        //                     LBD.LynxSetMotorConstantPowerCommand(0.7);
        //                     RFD.LynxSetMotorConstantPowerCommand(0.5);
        //                     RBD.LynxSetMotorConstantPowerCommand(0.5);
        //                 } else if (angles.firstAngle > -0.3) {
        //                     LFD.LynxSetMotorConstantPowerCommand(0.5);
        //                     LBD.LynxSetMotorConstantPowerCommand(0.5);
        //                     RFD.LynxSetMotorConstantPowerCommand(0.7);
        //                     RBD.LynxSetMotorConstantPowerCommand(0.7);
        //                 }
        //                 Velocity(1);
        //                 Power(0.4);
                        
        //             }
        //         }
        //     MDrive(1000, 1000, 1000, 1000);/* 1333 1116   1 inch = 62 */
        //     RunToPosition();
        //     Velocity(1);
        //     Power(0.4);
            
            telemetry.update();
        }
    }

    //----------------------------------------------------------------------------------------------
    // Telemetry Configuration
    //----------------------------------------------------------------------------------------------

    void composeTelemetry() {

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() { @Override public void run()
                {
                // Acquiring the angles is relatively expensive; we don't want
                // to do that in each of the three items that need that info, as that's
                // three times the necessary expense.
                angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                gravity  = imu.getGravity();
                }
            });

        telemetry.addLine()
            .addData("status", new Func<String>() {
                @Override public String value() {
                    return imu.getSystemStatus().toShortString();
                    }
                })
            .addData("calib", new Func<String>() {
                @Override public String value() {
                    return imu.getCalibrationStatus().toString();
                    }
                });

        telemetry.addLine()
            .addData("heading", new Func<String>() {
                @Override public String value() {
                    return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                })
            .addData("roll", new Func<String>() {
                @Override public String value() {
                    return formatAngle(angles.angleUnit, angles.secondAngle);
                    }
                })
            .addData("pitch", new Func<String>() {
                @Override public String value() {
                    return formatAngle(angles.angleUnit, angles.thirdAngle);
                    }
                });

        telemetry.addLine()
            .addData("grvty", new Func<String>() {
                @Override public String value() {
                    return gravity.toString();
                    }
                })
            .addData("mag", new Func<String>() {
                @Override public String value() {
                    return String.format(Locale.getDefault(), "%.3f",
                            Math.sqrt(gravity.xAccel*gravity.xAccel
                                    + gravity.yAccel*gravity.yAccel
                                    + gravity.zAccel*gravity.zAccel));
                    }
                });
    
    }

    //----------------------------------------------------------------------------------------------
    // Formatting
    //----------------------------------------------------------------------------------------------

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
}
