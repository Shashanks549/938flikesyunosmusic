#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep

pros::MotorGroup leftMotors({-4, -9, -19}, // left motor group - ports 4, 9, 19
                            pros::MotorGearset::blue); 
pros::MotorGroup rightMotors({15, 7, 10}, // right motor group - ports 15 (reversed), 7 (reversed), 10 (reversed)
                             pros::MotorGearset::blue); 

							 // drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              11.25, // 24 inch track width
                              lemlib::Omniwheel::NEW_325, // using new 3.25" omnis
                              450, // drivetrain rpm is 450
                              8 // horizontal drift is 2. If we had traction wheels, it would have been 8
);
// Inertial Sensor on port 16
pros::Imu imu(16);

// tracking wheels
// horizontal tracking wheel encoder. Rotation sensor, port 20, not reversed
pros::Rotation horizontalEnc(-11);
// vertical tracking wheel encoder. Rotation sensor, port 11, reversed
pros::Rotation verticalEnc(8);
// horizontal tracking wheel. 2.00" diameter, 5.75" offset, back of the robot (negative)
lemlib::TrackingWheel horizontal(&horizontalEnc, lemlib::Omniwheel::NEW_2, 1);
// vertical tracking wheel. 2.00" diameter, 2.5" offset, left of the robot (negative)
lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_2, -0.5);

// sensors for odometry
lemlib::OdomSensors sensors(&vertical, // vertical tracking wheel
                            nullptr, // vertical tracking wheel 2, set to nullptr as we don't have a second one
                            &horizontal, // horizontal tracking wheel
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// lateral motion controller
lemlib::ControllerSettings lateral_controller(5, // proportional gain (kP) 0.5
                                            0, // integral gain (kI)
                                            3, // derivative gain (kD) 15
                                            0, // anti windup
                                            1, // small error range, in inches
                                            100, // small error range timeout, in milliseconds
                                            6, // large error range, in inches
                                            175, // large error range timeout, in milliseconds 200
                                            0 // maximum acceleration (slew)
);

// angular motion controller
lemlib::ControllerSettings angular_controller(3, // proportional gain (kP) DECREASE KP AND KD 4
                                             0, // integral gain (kI)
                                             21, // derivative gain (kD) 25
                                             0, // anti windup
                                             2, // small error range, in degrees
                                             20, // small error range timeout, in milliseconds 50
                                             6, // large error range, in degrees
                                             100, // large error range timeout, in milliseconds 125
                                             0 // maximum acceleration (slew)
);

// create the chassis
lemlib::Chassis chassis(drivetrain, lateral_controller, angular_controller, sensors); //&throttleCurve, &steerCurve

pros::adi::DigitalOut finger (3, LOW);
pros::adi::DigitalOut PTO_1 (2, LOW);
pros::adi::DigitalOut mogomech_1 (1, LOW);
pros::adi::DigitalOut mogomech_2 (1, LOW);
pros::adi::DigitalOut doinger (4, LOW);
pros::Motor intake_1(-20, pros::v5::MotorGears::blue);
pros::Motor intake_2(18, pros::v5::MotorGears::blue); 
pros::MotorGroup intakes({-20, 18});
pros::Imu inertial_sensor (16);
pros::Optical color_sorter_sensor(21);

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	chassis.calibrate(); // calibrate sensors
    verticalEnc.reset_position();
    horizontalEnc.reset_position();
    finger.set_value(false);
    mogomech_1.set_value(true);
    mogomech_2.set_value(true);
    PTO_1.set_value(true);
    // print position to brain screen
    pros::Task screen_task([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            // delay to save resources
            pros::delay(20);
        }
    });
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void moveArm() {
    intakes.move_velocity(-600);
    pros::delay(800); //800
    intakes.move_velocity(0);
}
void autonomous() {
    chassis.setPose(58.971, 15.985, 180);
    pros::Task AllianceStake(moveArm);
    chassis.swingToHeading(135, DriveSide::LEFT, 2500); //134-135
    chassis.waitUntilDone();
    chassis.moveToPoint(57.527, 11.432, 5000, {.forwards=false});
    chassis.waitUntilDone(); 
    finger.set_value(true);
    pros::delay(500);
    PTO_1.set_value(false); 
    chassis.moveToPoint(43.336, 32.709, 5000, {.forwards=false});
    //finger.set_value(false);
    chassis.waitUntilDone();   
    //chassis.turnToHeading(295, 4000);
    chassis.turnToPoint(30.249, 26.878, 5000, {.forwards=false});
    chassis.waitUntilDone(); 
    chassis.moveToPoint(30.249, 26.878, 5000, {.forwards=false}); //-29.722, 26.303
    chassis.waitUntilDone();
    pros::delay(500);
    mogomech_1.set_value(false);
    mogomech_2.set_value(false);
    pros::delay(200);  
    chassis.turnToPoint(28.331, 44.119, 4000); //-25.833, 50.729
    intakes.move_velocity(-600);
    chassis.waitUntilDone();
    chassis.moveToPoint(28.331, 44.119, 5000, {.maxSpeed=90}); 
    chassis.waitUntilDone();
    pros::delay(750); 
    chassis.turnToPoint(12.485, 51.695, 5000);
    chassis.waitUntilDone();
    chassis.moveToPoint(12.485, 51.695, 5000, {.maxSpeed=90});
    chassis.waitUntilDone();
    pros::delay(500);
    chassis.moveToPoint(22.783, 47.452, 5000, {.forwards=false});
    chassis.waitUntilDone();
    /* chassis.moveToPoint(-7.907, 44.505, 5000, {.maxSpeed=90}); 
    chassis.waitUntilDone();  */  
    chassis.turnToPoint(20.286, 6.816, 5000);
    chassis.waitUntilDone();
    chassis.moveToPoint(20.286, 6.816, 5000, {.maxSpeed=40});
    chassis.waitUntilDone(); 
    intakes.move_velocity(0); 
    
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
	pros::Controller master(pros::E_CONTROLLER_MASTER);
	pros::MotorGroup left_mg({1, -2, 3});    // Creates a motor group with forwards ports 1 & 3 and reversed port 2
	pros::MotorGroup right_mg({-4, 5, -6});  // Creates a motor group with forwards port 5 and reversed ports 4 & 6


	while (true) {
		pros::lcd::print(0, "%d %d %d", (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
		                 (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
		                 (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >> 0);  // Prints status of the emulated screen LCDs

		// Arcade control scheme
		int dir = master.get_analog(ANALOG_LEFT_Y);    // Gets amount forward/backward from left joystick
		int turn = master.get_analog(ANALOG_RIGHT_X);  // Gets the turn left/right from right joystick
		left_mg.move(dir - turn);                      // Sets left motor voltage
		right_mg.move(dir + turn);                     // Sets right motor voltage
		pros::delay(20);                               // Run for 20 ms then update
	}
