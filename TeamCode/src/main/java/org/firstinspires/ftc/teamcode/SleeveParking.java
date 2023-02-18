package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import java.util.*;

@Autonomous(name = "Sleeve Parking")
public class SleeveParking extends LinearOpMode {
    RealRobot robot;
    private static final String TFOD_MODEL_ASSET = "/sdcard/FIRST/tflitemodels/PowerPlayV2.tflite";
    // private static final String TFOD_MODEL_FILE  = "/sdcard/FIRST/tflitemodels/CustomTeamModel.tflite";
    Servo lclaw, lchain, openSesame, upDown;

    private static final String[] LABELS = {
            "1", // banana
            "2", // whale
            "3" // pencil
    };

    int coneNum;
    boolean detected = false;

    private static final String VUFORIA_KEY =
            "AQrfl2X/////AAABmXgYPrwW30wdoBAHztdXSDgfzAtx/3aneWzwyCjSRj16HSy" +
                    "36qQ27HjnVGpjTF+XeOKXM5S3kX+HHUnC8HDJBAD44Iw5qDiE9HIDXikjR" +
                    "kp5CJai96FQUIAaHVQs/hXXYMFbVwPY5++U0WOPlSdRMzxvo0+c+Mjs3XVXj" +
                    "ItZ8OKzAtkdGo5eRVMbogXcz6OmpuM0Ts/u7WHD6Ux+Yp9uiIy/pFt/WOIMmIE" +
                    "w7jP8x941HWDbDsrOSZl78yONALbzqE/afXRns4WhmWt+5hLhmKzufU96/sCZbD1T0o" +
                    "NNWN7p6T25lrWpPuvRUBds5ZXbDTEvbgC5RRz9jZHybH0d6M4SvWiMme+Wp26ZAi/Q1fHE";

    private VuforiaLocalizer vuforia;

    private TFObjectDetector tfod;

    @Override
    public void runOpMode() {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();
        initTfod();
        initialize();


        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can increase the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(1.0, 16.0/9.0);
        }

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()&&!detected) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Objects Detected", updatedRecognitions.size());

                        // step through the list of recognitions and display image position/size information for each one
                        // Note: "Image number" refers to the randomized image orientation/number
                        for (Recognition recognition : updatedRecognitions) {
                            double col = (recognition.getLeft() + recognition.getRight()) / 2 ;
                            double row = (recognition.getTop()  + recognition.getBottom()) / 2 ;
                            double width  = Math.abs(recognition.getRight() - recognition.getLeft()) ;
                            double height = Math.abs(recognition.getTop()  - recognition.getBottom()) ;

                            telemetry.addData(""," ");
                            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100 );
                            telemetry.addData("- Position (Row/Col)","%.0f / %.0f", row, col);
                            telemetry.addData("- Size (Width/Height)","%.0f / %.0f", width, height);

                            if(recognition.getLabel().equals("1")){
                                detected = true;
                                coneNum = 1;
                            }

                            if(recognition.getLabel().equals("2")) {
                                detected = true;
                                coneNum = 2;
                            }

                            if(recognition.getLabel().equals("3")){
                                detected = true;
                                coneNum = 3;
                            }
                        }



                        telemetry.update();
                    }
                }
            }
        }

        // move to right place and cycle
        coneStack();

        robot.encoderDrive(0.5, 5.5, 'L');
        robot.encoderDrive(0.5,15.5,'F');
        sleep(500);
        robot.rotateTo(0, 0.5);
        sleep(500);

        //openSesame.setPosition(0.0);
        //drive forward, rotate, state machine
        if(coneNum==1) {
            robot.encoderDrive(0.5, 35, 'R');
            //sleep(500);
            //robot.encoderDrive(0.5,37.5,'B');
        }
        else if(coneNum==3) {
            robot.encoderDrive(0.5, 34.5, 'L');
            //sleep(500);
            //robot.encoderDrive(0.5, 37.5, 'B');
        } //else {
            //robot.encoderDrive(0.5, 40.0, 'B');
        //}

        /*robot.rotate(-185, 0.4);
        robot.ltrolley.setPower(-0.7);
        openSesame.setPosition(1.0);
        //Sleep until robot opens
        sleep(1050);
        robot.ltrolley.setPower(0.0);
        upDown.setPosition(0.375);
        //Close Robot
        sleep(500);
        lclaw.setPosition(0.5);
        openSesame.setPosition(0.0);
        sleep(2000);
        upDown.setPosition(1.0);
        lclaw.setPosition(0.8);*/
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.75f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 300;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);

        // Use loadModelFromAsset() if the TF Model is built in as an asset by Android Studio
        // Use loadModelFromFile() if you have downloaded a custom team model to the Robot Controller's FLASH.
        tfod.loadModelFromFile(TFOD_MODEL_ASSET, LABELS);
        // tfod.loadModelFromFile(TFOD_MODEL_FILE, LABELS);
    }

    private void coneStack() {
        int start = robot.ltrolley.getCurrentPosition();
        robot.encoderDrive(0.5, 70, 'B'); // need to edit
        sleep(500);
        robot.rotateTo(100, 0.5);
        sleep(1500);
        robot.ltrolley.setPower(0.0);
        upDown.setPosition(0.375);
        sleep(500);
        robot.encoderDrive(0.5,15.5,'B'); // need to edit
        robot.encoderDrive(0.5, 5.5, 'R');

        // get measurements and height
        // drop first cone

        //[0.436, 0.7094];
        raiseCone(start, 0.436);
        raiseCone(start, 0.425);
        raiseCone(start, 0.394);
        raiseCone(start, 0.375);
        raiseCone(start, 0.358);
    }

    private void raiseCone(int start, double dist) {
        // drop cone
        robot.ltrolley.setTargetPosition(start + 4600); // change this
        robot.ltrolley.setPower(0.7);
        sleep(1500);
        upDown.setPosition(1.0);
        robot.spinner.setPosition(0.95);

        // go down
        sleep(1000);
        robot.claw.setPosition(0.5);
        sleep(250);

        upDown.setPosition(0.7);
        robot.ltrolley.setTargetPosition(robot.ltrolley.getCurrentPosition()-3000);
        robot.ltrolley.setPower(-0.7);
        upDown.setPosition(dist);

        // grab cone
        robot.spinner.setPosition(0.15);
        sleep(925);
    }

    public void initialize() {
        ArrayList<DcMotor> allMotors = new ArrayList<>();
        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot = new RealRobot(hardwareMap, telemetry);
        allMotors.add(robot.lf);
        allMotors.add(robot.rf);
        allMotors.add(robot.lr);
        allMotors.add(robot.rr);

        //robot.carriage.setPosition(.77);
        //robot.carriage.setDirection(Servo.Direction.FORWARD);

        for (DcMotor dcMotor : allMotors) {
            dcMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        robot.loop();
        lchain = hardwareMap.servo.get("updown");
        lclaw = hardwareMap.servo.get("claw");
        upDown = hardwareMap.servo.get("updown");
        openSesame = hardwareMap.servo.get("OS");
        lclaw.setPosition(0.8);
        //lchain.scaleRange(0,0.8);
        /*robot.ltrolley.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);*/
        robot.resetHeading();



    }
}
