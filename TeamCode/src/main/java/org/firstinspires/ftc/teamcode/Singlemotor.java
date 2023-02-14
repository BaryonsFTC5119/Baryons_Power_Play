
package org.firstinspires.ftc.teamcode;
import static android.os.SystemClock.sleep;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.io.FileWriter;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name = "Singlemotor", group = "Iterative OpMode")
public class Singlemotor extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor lr = null; //used to be leftDrive
    private DcMotor rr = null;
    private RealRobot robot;
    private Controller controller;
    private Controller controller2;
    private boolean slowMode = false;
    Servo claw, upDown, spinner, openSesame;
    String state = "end";

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        robot = new RealRobot(hardwareMap, telemetry);
        controller = new Controller(gamepad1);
        controller2 = new Controller(gamepad2);
        upDown = hardwareMap.servo.get("updown");
        claw = hardwareMap.servo.get("claw");
        spinner = hardwareMap.servo.get("spinner");
        openSesame = hardwareMap.servo.get("OS");
        robot.ltrolley.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        controller.update();
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
        robot.resetHeading();
        robot.ltrolley.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.ltrolley.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        controller.update();
        controller2.update();
        robot.loop();

        final double x = Math.pow(controller.left_stick_x*1, 3.0);
        final double y = Math.pow(controller.left_stick_y*1, 3.0);

        //top Y, left X, bottom A, right B

        if(controller2.Y()) {
            robot.lr.setPower(y);
            robot.lf.setPower(0);
            robot.rr.setPower(0);
            robot.rf.setPower(0);
        }
        else if(controller2.X()) {
            robot.lf.setPower(y);
            robot.lr.setPower(0);
            robot.rr.setPower(0);
            robot.rf.setPower(0);
        }
        else if(controller2.A()) {
            robot.rr.setPower(y);
            robot.lf.setPower(0);
            robot.lr.setPower(0);
            robot.rf.setPower(0);
        }
        else if(controller2.B()) {
            robot.rf.setPower(y);
            robot.lf.setPower(0);
            robot.rr.setPower(0);
            robot.lr.setPower(0);
        }

        if(controller2.rightBumperOnce()) {
            claw.setPosition(claw.getPosition() +0.005);
        } else if(controller2.leftBumperOnce()) {
            claw.setPosition(claw.getPosition() - 0.005);
        }

        if(controller2.dpadLeftOnce()) {
            spinner.setPosition(spinner.getPosition() + 0.005);
        } else if(controller2.dpadRightOnce()) {
            spinner.setPosition(spinner.getPosition() - 0.005);
        }

        if(controller2.dpadUpOnce()) {
            upDown.setPosition(upDown.getPosition() + 0.005);
        } else if(controller2.dpadLeftOnce()) {
            upDown.setPosition(upDown.getPosition() - 0.005);
        }

        telemetry.addData("1 Left Joystick Y", controller.left_stick_y);
        telemetry.addData("1 Left Joystick X", controller.left_stick_x);
        telemetry.addData("2 Left Joystick Y", controller2.left_stick_y);
        telemetry.addData("2 Left Joystick X", controller2.left_stick_x);
        telemetry.addData("LF Position", robot.lf.getCurrentPosition());
        telemetry.addData("RF Position", robot.rf.getCurrentPosition());
        telemetry.addData("LR Position", robot.lr.getCurrentPosition());
        telemetry.addData("RR Position", robot.rr.getCurrentPosition());
        telemetry.addData("Spinner position: ", spinner.getPosition());
        telemetry.addData("upDown position: ", upDown.getPosition());
        telemetry.addData("claw wants to go to:  ", claw.getPosition());
        telemetry.addData("Open sesame wants to go to: ", openSesame.getPosition());
        telemetry.addData("LTrolley position: ", robot.ltrolley.getCurrentPosition());
        telemetry.addData("Status ", state);
        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {

    }


}