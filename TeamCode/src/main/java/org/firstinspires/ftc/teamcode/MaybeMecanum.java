
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

@TeleOp(name = "MaybeMecanum", group = "Iterative OpMode")
public class MaybeMecanum extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime elapsed = new ElapsedTime();
    private DcMotor lr = null;
    private DcMotor rr = null;
    private RealRobot robot;
    private Controller controller;
    private Controller controller2;
    private boolean arcadeMode = false;
    private boolean slowMode = false;
    //private boolean clawGrabPos = true;
    private boolean headingReset = false;
    public static double governor = 0.7;
    int start;
    Servo claw, upDown, spinner, openSesame;
    String state = "end";
    //private ElapsedTime time = new Ela

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
        //claw.setPosition(0.55);
        //robot.ltrolley.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        controller.update();
        /*if (controller.AOnce()) {
            //arcadeMode = !arcadeMode;
        }*/
        if (controller.XOnce()) {
            slowMode = !slowMode;
        }
        telemetry.addData("Arcade Mode (a)", arcadeMode ? "YES" : "no.");
        telemetry.addData("Slow Mode (s)", slowMode ? "YES" : "no.");
        telemetry.update();
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        elapsed.reset();
        robot.resetHeading();
        //robot.ltrolley.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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

        telemetry.addData("Arcade Mode (a)", arcadeMode ? "YES" : "no.");
        telemetry.addData("Slow Mode (s)", slowMode ? "YES" : "no.");
        telemetry.addData("Heading", robot.getHeadingDegrees());

        final double x = Math.pow(controller.left_stick_x * 1, 3.0);
        final double y = Math.pow(controller.left_stick_y * 1, 3.0);

        final double rotation = -Math.pow(controller.right_stick_x * 1, 3.0) / 1.5;
        final double direction = -(Math.atan2(x, y) + (arcadeMode ? robot.getHeading() : 0.0));
        final double speed = Math.min(1.0, Math.sqrt(x * x + y * y));

        double lf = (slowMode ? governor * .6 : governor) * speed * Math.sin(direction + Math.PI / 4.0) + (slowMode ? .2 : 1) * rotation;
        double rf = (slowMode ? governor * .6 : governor) * speed * Math.cos(direction + Math.PI / 4.0) - (slowMode ? .2 : 1) * rotation;
        double lr = (slowMode ? governor * .6 : governor) * speed * Math.cos(direction + Math.PI / 4.0) + (slowMode ? .2 : 1) * rotation;
        double rr = (slowMode ? governor * .6 : governor) * speed * Math.sin(direction + Math.PI / 4.0) - (slowMode ? .2 : 1) * rotation;

        /**
         * Controller button position reference:
         * Top (Triangle): Y
         * Left (Square): X
         * Bottom (X): A
         * Right (Circle): B
         */

        // Toggles between slowmode and not
        if (controller.XOnce()) {
            slowMode = !slowMode;
        }

        // Starts State Machine
        if (controller.dpadLeftOnce()) {
            state = "lift";
            start = robot.ltrolley.getCurrentPosition();
        }

        // Rotates to heading 0
        if (controller.BOnce()) {
            if (!headingReset)
                robot.resetHeading();
            else
                robot.rotateTo(180, 0.4);
        }

        //Toggles Claw Spinner position
        if (controller.leftBumperOnce()) {
            robot.spinnerToggle();
            /*if (clawGrabPos) {
                spinner.setPosition(0.95); // Claw Spinner Upright
                clawGrabPos = false;
            } else {
                spinner.setPosition(0.15); // Claw Spinner Flipped
                clawGrabPos = true;
            }*/
        }

        // Lowers Trolley Arm
        if (controller.YOnce()) {
            /*if (upDown.getPosition() < 1)
                upDown.setPosition(upDown.getPosition() + 0.05);*/
            robot.upDownCycle(1);
        }

        // Raises Trolley Arm
        if (controller.AOnce()) {
            /*if (upDown.getPosition() > 0.375)
                upDown.setPosition(upDown.getPosition() - 0.05);*/
            robot.upDownCycle(-1);
        }

        if (controller.rightBumperOnce()) {
            robot.clawToggle();
        }

        // Moves Trolley down
        if (controller.dpadDown()) {
            robot.ltrolley.setPower(-0.8);
            state = "end";
        }

        // Moves Trolley up
        if (controller.dpadUp()) {
            robot.ltrolley.setPower(0.8);
            state = "end";
        }

        // Stops Trolley
        if (!controller.dpadDown() && !controller.dpadUp()) {
            robot.ltrolley.setPower(0);
        }

        // Opens Opener
        if (controller.left_trigger > 0.2) {
            openSesame.setPosition(openSesame.getPosition() + 0.1);
        }

        // Closes Opener
        if (controller.right_trigger > 0.2) {
            openSesame.setPosition(openSesame.getPosition() - 0.1);
        }

        // State Machine
    //start+2900 at top
        if(state.equals("lift")){
            elapsed.reset();
            robot.clawClose();
            //int start = robot.ltrolley.getCurrentPosition();
            robot.upDownMed();
            robot.trolleyMed();
            robot.spinnerFlipped();
            //robot.trolleyHalf();
            /*robot.ltrolley.setTargetPosition(start+3050); //4600
            robot.ltrolley.setPower(0.7);
            robot.ltrolley.setMode(DcMotor.RunMode.RUN_TO_POSITION);*/
            //sleep(1500);
            while(robot.ltrolley.isBusy());
            elapsed.reset();
            robot.trolleyHigh();
            while(elapsed.milliseconds()<500);
            robot.upDownHigh();
            //upDown.setPosition(1.0);
            /*if(clawGrabPos) {
                spinner.setPosition(0.95);
                clawGrabPos = false;
            } else {
                spinner.setPosition(0.20);
                clawGrabPos = true;
            }*/
            while(robot.ltrolley.isBusy() || elapsed.milliseconds()<500);
            //sleep(1500); // 1000
            state = "drop";
        }

        if(state.equals("drop")){
            robot.clawOpen();
            elapsed.reset();
            while(elapsed.milliseconds()<250);
            //sleep(250);
            state = "return";
        }

        if(state.equals("return")){
            elapsed.reset();
            robot.upDownMed();
            while(elapsed.milliseconds()<250);
            robot.trolleyLow();
            /*robot.ltrolley.setTargetPosition(robot.ltrolley.getCurrentPosition()-3050); // 4600
            robot.ltrolley.setPower(-0.7);
            robot.ltrolley.setMode(DcMotor.RunMode.RUN_TO_POSITION);*/
            robot.spinnerUpright();
            /*
            upDown.setPosition(0.375);
            if(clawGrabPos) {
                spinner.setPosition(0.95);
                clawGrabPos = false;
            } else {
                spinner.setPosition(0.15);
                clawGrabPos = true;
            }
            */
            while(robot.ltrolley.isBusy());
            //sleep(1850);
            state = "stop";
        }
        if(state.equals("stop")) {
            robot.ltrolley.setPower(0);
            state = "end";
        }
        /*
        if(state.equals("end")){
            robot.ltrolley.setPower(0);
        }
        */





/**
 * state machine
 * lift: make the trolley go up set target position set motor to run to position, turn power on, start lifting upDown to save time
 * drop claw: upDown should be fully extended at this point, rotate arm with spinner and open claw
 * return: make trolley come down, retract upDown, rotate claw back with spinner
 * stop: fully stops trolley
 * end: do nothing, add state = end statements to other commands to be able to override if machine goes wrong
 */




        robot.setMotors(lf, lr, rf, rr);

        telemetry.addData("LF Position", robot.lf.getCurrentPosition());
        telemetry.addData("RF Position", robot.rf.getCurrentPosition());
        telemetry.addData("LR Position", robot.lr.getCurrentPosition());
        telemetry.addData("RR Position", robot.rr.getCurrentPosition());
        telemetry.addData("1 Left Joystick Y", controller.left_stick_y);
        telemetry.addData("1 Left Joystick X", controller.left_stick_x);
        telemetry.addData("2 Left Joystick Y", controller2.left_stick_y);
        telemetry.addData("2 Left Joystick X", controller2.left_stick_x);
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
