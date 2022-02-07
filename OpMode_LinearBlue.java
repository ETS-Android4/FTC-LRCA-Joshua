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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.ColorSensor;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


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

@Autonomous(name="OurAutonBlue", group="Linear Opmode")
//@Disabled
public class OpMode_LinearBlue extends LinearOpMode {

    // Declare OpMode members.

    private ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() {

        ElapsedTime runtime = new ElapsedTime();
        DcMotor leftDriveBack;
        DcMotor leftDriveFront;
        DcMotor rightDriveBack;
        DcMotor rightDriveFront;
        DcMotor collectionMotor;
        DcMotor carouselMotor;
        DcMotor viperMotor;
        double motSpeed;

        DistanceSensor sensorRangeBack;
        DistanceSensor sensorRangeRight;
        DistanceSensor sensorRangeLeft;

        //ColorSensor colorSensor;

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //colorSensor = hardwareMap.get(ColorSensor.class, "COL");
        //colorSensor.enableLed(true);

        rightDriveFront = hardwareMap.get(DcMotor.class, "FRW");
        leftDriveFront = hardwareMap.get(DcMotor.class, "FLW");
        rightDriveBack = hardwareMap.get(DcMotor.class, "BRW");
        leftDriveBack = hardwareMap.get(DcMotor.class, "BLW");
        collectionMotor = hardwareMap.get(DcMotor.class, "COLLM");
        carouselMotor = hardwareMap.get(DcMotor.class, "CARM");
        viperMotor = hardwareMap.get(DcMotor.class, "VIPM");

        sensorRangeBack = hardwareMap.get(DistanceSensor.class, "SRBACK");
        sensorRangeRight = hardwareMap.get(DistanceSensor.class, "SRRIGHT");
        sensorRangeLeft = hardwareMap.get(DistanceSensor.class, "SRLEFT");


        leftDriveBack.setDirection(DcMotor.Direction.REVERSE);
        leftDriveFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightDriveBack.setDirection(DcMotor.Direction.FORWARD);
        rightDriveFront.setDirection(DcMotorSimple.Direction.FORWARD);
        collectionMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        carouselMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        viperMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        double distBack = sensorRangeBack.getDistance(DistanceUnit.CM);
        double distRight = sensorRangeRight.getDistance(DistanceUnit.CM);
        double distLeft = sensorRangeLeft.getDistance(DistanceUnit.CM);

        boolean duckStage = true;
        boolean parkStage = false;


        boolean rightCorrect = false;
        boolean backCorrect = false;

        boolean orientedCorrect = false;

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {



            rightDriveFront.setPower(1);
            leftDriveFront.setPower(1);
            rightDriveBack.setPower(1);
            leftDriveBack.setPower(1);
            sleep(3000);

            /*while (duckStage) {

                if (distBack > 17.78) {
                    rightDriveFront.setPower(-0.2);
                    leftDriveFront.setPower(-0.2);
                    rightDriveBack.setPower(-0.2);
                    leftDriveBack.setPower(-0.2);

                    carouselMotor.setPower(0);
                    backCorrect = false;
                } if (distBack < 17.78) {
                    rightDriveFront.setPower(0.2);
                    leftDriveFront.setPower(0.2);
                    rightDriveBack.setPower(0.2);
                    leftDriveBack.setPower(0.2);

                    carouselMotor.setPower(0);
                    backCorrect = false;
                } if (distBack == 17.78) {
                    backCorrect = true;
                }

                if (distRight > 17.78) {
                    rightDriveFront.setPower(-0.2);
                    leftDriveFront.setPower(0.2);
                    rightDriveBack.setPower(0.2);
                    leftDriveBack.setPower(-0.2);

                    carouselMotor.setPower(0);
                    rightCorrect = false;
                } if (distRight < 17.78) {
                    rightDriveFront.setPower(0.2);
                    leftDriveFront.setPower(-0.2);
                    rightDriveBack.setPower(-0.2);
                    leftDriveBack.setPower(0.2);

                    carouselMotor.setPower(0);
                    rightCorrect = false;
                } if (distRight == 17.78) {
                    rightCorrect = true;
                }


                if (rightCorrect == true && backCorrect == true) {
                    if (1 == 1) {
                        carouselMotor.setPower(0.5);
                    } if (1 == 1) {
                        duckStage = false;
                        parkStage = true;
                    }
                } else {
                    carouselMotor.setPower(0);
                    duckStage = false;
                }
            }*/

            /*while (parkStage) {
                if (orientedCorrect == false) {
                    if (distRight >= 17.78) {
                        rightDriveFront.setPower(-0.2);
                        leftDriveFront.setPower(0.2);
                        rightDriveBack.setPower(-0.2);
                        leftDriveBack.setPower(0.2);
                    }

                    if (distRight)
                }

                while (beginning <= 20000) {
                    rightDriveFront.setPower(0.4);
                    leftDriveFront.setPower(0.4);
                    rightDriveBack.setPower(0.4);
                    leftDriveBack.setPower(0.4);
                }
                while (beginning > 20000) {
                    rightDriveFront.setPower(1);
                    leftDriveFront.setPower(-1);
                    rightDriveBack.setPower(-1);
                    leftDriveBack.setPower(1);
                }
            }*/


            /*if (colorSensor.red() < 1000) {
                rightDriveFront.setPower(0.1);
                leftDriveFront.setPower(0.1);
                rightDriveBack.setPower(0.1);
                leftDriveBack.setPower(0.1);

                carouselMotor.setPower(0);
            }

            if (colorSensor.red() >= 1000) {
                rightDriveFront.setPower(0.5);
                leftDriveFront.setPower(0.5);
                rightDriveBack.setPower(0.5);
                leftDriveBack.setPower(0.5);

                carouselMotor.setPower(1);
            }*/

                telemetry.addData("rangeBack", String.format("%.01f cm", sensorRangeBack.getDistance(DistanceUnit.CM)));
                telemetry.addData("rangeRight", String.format("%.01f cm", sensorRangeRight.getDistance(DistanceUnit.CM)));
                telemetry.addData("rangeLeft", String.format("%.01f cm", sensorRangeLeft.getDistance(DistanceUnit.CM)));


                telemetry.addData("Status", "Run Time: " + runtime.toString());
                telemetry.addData("Motors", "left (%.2f), right (%.2f)");
                //telemetry.addData("Alpha  ", colorSensor.alpha());
                //telemetry.addData("Red  ", colorSensor.red());
                telemetry.update();
            }

            // colorSensor.enableLed(false);
        }
    }
