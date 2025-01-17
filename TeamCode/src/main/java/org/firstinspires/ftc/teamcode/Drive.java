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

package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import java.util.List;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
// idk lol
@TeleOp(name="Basic Drive", group="Linear Opmode")
//@Disabled
public class Drive extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private Robot robot;

    @Override
    public void runOpMode() {
        robot = new Robot(hardwareMap);

        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        //robot.initVuforia();
        // robot.initTfod();

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        /**if (robot.tfod != null) {
         robot.tfod.activate();
         // The TensorFlow software will scale the input images from the camera to a lower resolution.
         // This can result in lower detection accuracy at longer distances (> 55cm or 22").
         // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
         // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
         // should be set to the value of the images used to create the TensorFlow Object Detection model
         // (typically 16/9).
         robot.tfod.setZoom(1.5, 16.0/9.0);
         }
         */
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        int lastflposition = robot.fl.getCurrentPosition();
        int lastfrposition = robot.fr.getCurrentPosition();
        int lastblposition = robot.bl.getCurrentPosition();
        int lastbrposition = robot.br.getCurrentPosition();
        int armTargetPos = 0;

        robot.arm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.armb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // run until the end of the match (driver presses STOP)

        while (opModeIsActive()) {

            double x = Math.pow(gamepad1.left_stick_x,1) * Robot.DRIVE_MULTIPLIER;
            double y = Math.pow(gamepad1.left_stick_y,1) * Robot.DRIVE_MULTIPLIER;
            double turn = Math.pow(gamepad1.right_stick_x,1) * Robot.DRIVE_MULTIPLIER;

            double flspeed = (double)(robot.fl.getCurrentPosition() - lastflposition)/runtime.milliseconds();
            double frspeed = (double)(robot.fr.getCurrentPosition() - lastfrposition)/runtime.milliseconds();
            double blspeed = (double)(robot.bl.getCurrentPosition() - lastblposition)/runtime.milliseconds();
            double brspeed = (double)(robot.br.getCurrentPosition() - lastbrposition)/runtime.milliseconds();
            runtime.reset();

            lastflposition = robot.fl.getCurrentPosition();
            lastfrposition = robot.fr.getCurrentPosition();
            lastblposition = robot.bl.getCurrentPosition();
            lastbrposition = robot.br.getCurrentPosition();


            // Tank Mode uses one stick to control each wheel.
            // - This requires no math, but it is hard to drive forward slowly and keep straight.
            // leftPower  = -gamepad1.left_stick_y ;
            // rightPower = -gamepad1.right_stick_y ;

            double triggerMultiplier = 1.0 - (gamepad1.right_trigger * 0.95);
            double grip = (gamepad1.left_trigger * 0.99);

            // Send calculated power to wheels
            robot.mecanumDrive(x * triggerMultiplier, y * triggerMultiplier, turn * triggerMultiplier);

            //robot.testServo.setPosition(grip);

            robot.sc.setPower(((gamepad1.right_bumper ? 1 : 0) - (gamepad1.left_bumper ? 1 : 0))*0.5);

            robot.arm1.setPower(((gamepad1.a ? 1 : 0) - (gamepad1.b ? 1 : 0)) * 0.5);
            robot.armb.setPower(((gamepad1.a ? 1 : 0) - (gamepad1.b ? 1 : 0)) * 0.5);

            robot.arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.armb.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            armTargetPos += ((gamepad1.a ? 1 : 0) - (gamepad1.b ? 1 : 0)) * 5;

            // presets for arm
            /*
            if(gamepad1.a){
                armTargetPos = 0;
            }
            if(gamepad1.b){
                armTargetPos = 104;
            }
            */
            // clamp the arm target position
            if(armTargetPos > 120){
                armTargetPos = 120;
            }
            if(armTargetPos < 0) {
                armTargetPos = 0;
            }

            // move the arm to the target position
            robot.arm1.setTargetPosition(-armTargetPos);
            robot.armb.setTargetPosition(-armTargetPos);

            // add telemetry for debugging
            telemetry.addData("target position", armTargetPos);
            telemetry.addData("letter 1 position", robot.arm1.getCurrentPosition());
            telemetry.addData("number b position", robot.armb.getCurrentPosition());
            telemetry.update();

            // close claw
            if (gamepad1.left_trigger > 0.2){
                robot.lgrabber.setPosition(0.45);
                robot.rgrabber.setPosition(0.55);
            }
            // open claw
            else {
                robot.lgrabber.setPosition(0);
                robot.rgrabber.setPosition(0);
            }

            // Show the elapsed game time and wheel power.

          /*  if (robot.tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = robot.tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());
                    // step through the list of recognitions and display boundary info.
                    int i = 0;
                    for (Recognition recognition : updatedRecognitions) {
                        telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                        telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                recognition.getLeft(), recognition.getTop());
                        telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                recognition.getRight(), recognition.getBottom());
                        i++;
                    }
                    telemetry.update();
                }
            }*/
        }
    }
}