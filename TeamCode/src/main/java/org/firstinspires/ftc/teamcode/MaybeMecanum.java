
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
    private double start;
    //private DcMotor lr = null;
    //private DcMotor rr = null;
    private RealRobot robot;
    private Controller controller;
    private Controller controller2;
    private boolean arcadeMode = false;
    private boolean slowMode = false;
    private boolean headingReset = false;
    public static double governor = 0.7;
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
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        controller.update();
        if (controller.XOnce()) {
            slowMode = !slowMode;
        }
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
        robot.setTrolleyState();
        robot.ltrolley.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER, robot.lf, robot.rf, robot.rr, robot.lr);
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        controller.update();
        controller2.update();
        robot.loop();

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

        // Toggles between slowmode
        if (controller.XOnce()) {
            slowMode = !slowMode;
        }



        // Rotates to heading 0
        if (controller.BOnce()) {
            if (!headingReset)
                robot.resetHeading();
            else
                robot.rotateTo(180, 0.4);
        }

        // Toggles Claw Spinner position
        if (controller.leftBumperOnce()) {
            robot.spinnerToggle();
        }

        // Raises Arm
        if (controller.YOnce()) {
            robot.upDownCycle(1);
        }

        // Lowers Arm
        if (controller.AOnce()) {
            robot.upDownCycle(-1);
        }

        // Toggles Claw
        if (controller.rightBumperOnce()) {
            robot.clawToggle();
        }

        // Moves Trolley up
        if (controller.dpadUpOnce()) {
            robot.trolleyCycle(1);
            //robot.ltrolley.setPower(0.8);
        }

        // Moves Trolley down
        if (controller.dpadDownOnce()) {
            robot.trolleyCycle(-1);
            //robot.ltrolley.setPower(-0.8);
        }

        // Opens Opener
        if (controller.left_trigger > 0.2) {
            openSesame.setPosition(openSesame.getPosition() + 0.1);
        }

        // Closes Opener
        if (controller.right_trigger > 0.2) {
            openSesame.setPosition(openSesame.getPosition() - 0.1);
        }

        // Starts State Machine
        if (controller.dpadLeftOnce()) {
            if(robot.getState().equals("end")) {
                state = "begin";
            } else {
                state = "end";
            }
        }

        if(state.equals("begin")) {
            robot.clawClose();
            robot.upDownMed();
            robot.trolleyHigh();
            state = "lift";
            if(robot.ltrolley.getCurrentPosition()>=robot.TROLLEY_HIGH/2) {
                state = "rotate";
                start = elapsed.milliseconds();
            }
        }
        if(state.equals("rotate")) {
            robot.spinnerFlipped();
            if(elapsed.milliseconds() >= start + 250) {
                state = "raiseToMed";
            }
        }
        if(state.equals("raiseToMed")) {
            robot.trolleyHigh();
            robot.upDownHigh();
            if(!robot.ltrolley.isBusy()) {
                state = "drop";
                start = elapsed.milliseconds();
            }
        }
        if(state.equals("drop")) {
            robot.clawOpen();
            if(elapsed.milliseconds() >= start + 250) {
                state = "return";
                start = elapsed.milliseconds();
            }
        }
        if(state.equals("return")) {
            robot.upDownMed();
            if (elapsed.milliseconds() >= start + 250) {
                state = "resetTrolley";
            }
        }
        if(state.equals("resetTrolley")){
            robot.spinnerUpright();
            robot.trolleyLow();
            if(!robot.ltrolley.isBusy()) {
                state = "stop";
            }
        }
        if(state.equals("stop")) {
            robot.ltrolley.setPower(0);
            state = "end";
        }

        robot.setMotors(lf, lr, rf, rr);

        telemetry.addData("Arcade Mode (a)", arcadeMode ? "YES" : "no.");
        telemetry.addData("Slow Mode (s)", slowMode ? "YES" : "no.");
        telemetry.addData("Heading", robot.getHeadingDegrees());
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
