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

@Autonomous(name="OurAuton", group="Linear Opmode")
//@Disabled
public class OPAutonomous extends LinearOpMode {

    //Creates object for timer
    ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        //Motors
        DcMotor leftDriveBack;
        DcMotor leftDriveFront;
        DcMotor rightDriveBack;
        DcMotor rightDriveFront;
        DcMotor collectionMotor;
        DcMotor carouselMotor;
        DcMotor viperMotor;

        double motSpeed;


        double motorCorrection = 0.05;

        boolean oriented = false;

        //Distance Sensors
        DistanceSensor sensorRangeRightFront;
        DistanceSensor sensorRangeRightBack;

        //Creating Sensors
        sensorRangeRightFront = hardwareMap.get(DistanceSensor.class, "SRF");
        sensorRangeRightBack = hardwareMap.get(DistanceSensor.class, "SRB");

        double distRightFront = sensorRangeRightFront.getDistance(DistanceUnit.CM);
        double distRightBack = sensorRangeRightBack.getDistance(DistanceUnit.CM);

        boolean stop = false;

        int stopTime = 500;




        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //Creating Motors
        rightDriveFront = hardwareMap.get(DcMotor.class, "FRW");
        leftDriveFront = hardwareMap.get(DcMotor.class, "FLW");
        rightDriveBack = hardwareMap.get(DcMotor.class, "BRW");
        leftDriveBack = hardwareMap.get(DcMotor.class, "BLW");
        collectionMotor = hardwareMap.get(DcMotor.class, "COLLM");
        carouselMotor = hardwareMap.get(DcMotor.class, "CARM");
        viperMotor = hardwareMap.get(DcMotor.class, "VIPM");

        //Direction of motors
        leftDriveBack.setDirection(DcMotor.Direction.REVERSE);
        leftDriveFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightDriveBack.setDirection(DcMotor.Direction.FORWARD);
        rightDriveFront.setDirection(DcMotorSimple.Direction.FORWARD);
        collectionMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        carouselMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        viperMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();





        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            //This was what I was working on last. I was trying to make it regulate its orientation so that it would be parallel to the wall. It just spins constantly

            //if misaligned, rotate counterclockwise
            if (distRightFront - distRightBack >= 1) {
                rightDriveFront.setPower(-motorCorrection);
                leftDriveFront.setPower(motorCorrection);
                rightDriveBack.setPower(-motorCorrection);
                leftDriveBack.setPower(motorCorrection);
                oriented = false;
            }

            //if misaligned, rotate clockwise
            if (distRightFront - distRightBack <= -1) {
                rightDriveFront.setPower(motorCorrection);
                leftDriveFront.setPower(-motorCorrection);
                rightDriveBack.setPower(motorCorrection);
                leftDriveBack.setPower(-motorCorrection);
                oriented = false;
            }

            //if aligned, no change
            if (distRightFront - distRightBack <= 1 && distRightFront - distRightBack >= -1) {
                oriented = true;
            }

            if (stop) {
                rightDriveFront.setPower(0);
                leftDriveFront.setPower(0);
                rightDriveBack.setPower(0);
                leftDriveBack.setPower(0);
                sleep(stopTime);
                stop = false;
            }

            if (oriented) {
                carouselMotor.setPower(1);

                rightDriveFront.setPower(0);
                leftDriveFront.setPower(0);
                rightDriveBack.setPower(0);
                leftDriveBack.setPower(0);
            }




            //Sends info to DH about distance sensors
                telemetry.addData("rangeRightFront", String.format("%.01f cm", distRightFront));
                telemetry.addData("rangeRightBack", String.format("%.01f cm", distRightBack));

                //Sends info to DH about runtime
                telemetry.addData("Status", "Run Time: " + runtime.toString());
                telemetry.update();
            }
        }
    }