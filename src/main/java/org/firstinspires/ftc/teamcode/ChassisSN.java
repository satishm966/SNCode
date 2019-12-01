package org.firstinspires.ftc.teamcode;
import android.app.Activity;
import android.graphics.Color;
import android.view.View;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import java.util.Locale;


public class ChassisSN {

    static final double     COUNTS_PER_MOTOR_REV    = 383.6*2  ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 3.937 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.5;
    static final double     TURN_SPEED              = 0.4;
    static final double     servoPosition           = 0.0;
    LinearOpMode OPMODE                             =  null;
    HardwarePushbotSN       robot;

    public void init ( LinearOpMode op, HardwarePushbotSN r)
    {
        OPMODE = op;
        robot = r;

        robot.CH_M_BR.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.CH_M_FR.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.CH_M_BL.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void Forward(double distance){
        OPMODE.telemetry.addLine().addData("[SN CHASSIS] ", "Moving FWD with Speed: %f, Distance: %f", DRIVE_SPEED, distance);
        encoderDrive(DRIVE_SPEED,  distance,   distance, distance, distance);
    }
    public void Backward(double distance){
        OPMODE.telemetry.addLine().addData("[SN CHASSIS]", "Moving Backward with Speed: %f, Distance: %f", DRIVE_SPEED, distance);
        encoderDrive(DRIVE_SPEED,  -distance,   -distance, -distance, -distance);
    }
    public void StrafeL(double distance){
        OPMODE.telemetry.addLine().addData("[SN CHASSIS]", "StrafeL with Speed: %f, Distance: %f", DRIVE_SPEED, distance);
        encoderDrive(DRIVE_SPEED,  -distance,   distance, distance, -distance);
    }
    public void StrafeR(double distance){
        OPMODE.telemetry.addLine().addData("[SN CHASSIS]", "StrafeR with Speed: %f, Distance: %f", DRIVE_SPEED, distance);

        encoderDrive(DRIVE_SPEED,  distance,   -distance, -distance, +distance);
    }
    public void TurnL(double distance){
        OPMODE.telemetry.addLine().addData("[SN CHASSIS]", "TurnL with Speed: %f, Distance: %f", DRIVE_SPEED, distance);

        encoderDrive(TURN_SPEED,  distance,   -distance, distance, -distance);
    }
    public void TurnR(double distance){
        OPMODE.telemetry.addLine().addData("[SN CHASSIS]", "TurnR with Speed: %f, Distance: %f", DRIVE_SPEED, distance);

        encoderDrive(TURN_SPEED,  -distance,   distance, -distance, -distance);
    }
    public void encoderDrive(double speed,
                             double FL, double FR, double BL, double BR) {
        int newLeftTarget;
        int newLeftTarget1;
        int newRightTarget;
        int newRightTarget1;
        // Ensure that the opmode is still active
        if (OPMODE.opModeIsActive()) {
            // Determine new target position, and pass to motor controlle
            newLeftTarget = robot.CH_M_FL.getCurrentPosition() + (int) (FL * COUNTS_PER_INCH);
            newLeftTarget1 = robot.CH_M_BL.getCurrentPosition() + (int) (BL * COUNTS_PER_INCH);
            newRightTarget = robot.CH_M_FR.getCurrentPosition() + (int) (FR * COUNTS_PER_INCH);
            newRightTarget1 = robot.CH_M_BR.getCurrentPosition() + (int) (BR * COUNTS_PER_INCH);
            robot.CH_M_FL.setTargetPosition(newLeftTarget);
            robot.CH_M_FR.setTargetPosition(newRightTarget);
            robot.CH_M_BL.setTargetPosition(newLeftTarget1);
            robot.CH_M_BR.setTargetPosition(newRightTarget1);
            // Turn On RUN_TO_POSITION
            robot.CH_M_FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.CH_M_FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.CH_M_BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.CH_M_BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            // reset the timeout time and start motion.
//            runtime.reset();
            robot.CH_M_FR.setPower(Math.abs(speed));
            robot.CH_M_FL.setPower(Math.abs(speed));
            robot.CH_M_BR.setPower(Math.abs(speed));
            robot.CH_M_BL.setPower(Math.abs(speed));
            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (OPMODE.opModeIsActive() &&
                    (robot.CH_M_FL.isBusy() && robot.CH_M_FR.isBusy())) {
                // Display it for the driver.
//                telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
 //               telemetry.addData("Path2", "Running at %7d :%7d", robot.CH_M_FL.getCurrentPosition(), robot.CH_M_FR.getCurrentPosition());
 //               telemetry.update();
            }
            // Stop all motion;
            robot.CH_M_FL.setPower(0);
            robot.CH_M_FR.setPower(0);
            robot.CH_M_BL.setPower(0);
            robot.CH_M_BR.setPower(0);
        }
    }
}