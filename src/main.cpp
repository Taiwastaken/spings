#include "main.h"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "liblvgl/extra/widgets/list/lv_list.h"
#include "pros/adi.hpp"
#include "pros/imu.hpp"
#include "pros/llemu.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/motors.hpp"
#include "lemlib/api.hpp"
#include "pros/optical.h"
#include "pros/optical.hpp"
#include "pros/rotation.hpp"
#include "pros/rtos.h"
#include "pros/rtos.hpp"
#include <cstdio>
#include <string>
pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::MotorGroup left_motor_group({-16, -17, 18}, pros::MotorGearset::blue);
pros::MotorGroup right_motor_group({13, -14, 15}, pros::MotorGearset::blue);
pros::Motor intake_stage_1(-11, pros::MotorGearset::green);
pros::Motor intake_stage_2(12, pros::MotorGearset::green);
pros::Motor lady_brown(19,pros::MotorGearset::green);
pros::adi::DigitalOut mogo_mech('B');
pros::adi::DigitalOut intake_lift('A');
pros::adi::DigitalOut doinker('H');
pros::Imu imu(10);
pros::Rotation rotation(8);
pros::Optical color_sorter(1);
lemlib::TrackingWheel vertical_tracking_wheel(&rotation, lemlib::Omniwheel::NEW_275, 2);
int lady_brown_pos = 1;
int counter = 0;
bool color_sorter_red_enable=false;
bool color_sorter_blue_enable=false;
bool counter_enable=false;
bool mogo_mech_extended=false;
bool intake_lift_extended=false;
bool doinker_extended=false;
bool s2disable=false;
lemlib::Drivetrain drivetrain(&left_motor_group, 
                              &right_motor_group, 
                              12.5, 
                              lemlib::Omniwheel::NEW_325, 
                              360, 
                              2); 
lemlib::OdomSensors sensors(nullptr, 
                            nullptr, 
                            nullptr, 
                            nullptr, 
                            &imu 
);
lemlib::ControllerSettings lateral_controller(20, 
                                              0, 
                                            2, 
                                              3, 
                                              1, 
                                              100, 
                                              3, 
                                              500, 
                                              30 
);

lemlib::ControllerSettings angular_controller(3, 
                                            0, 
                                              15, 
                                              3, 
                                              1, 
                                              100, 
                                              3, 
                                              500, 
                                              0 );


lemlib::Chassis chassis(drivetrain, 
                        lateral_controller, 
                        angular_controller, 
                        sensors 
);
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(6, "");
	} else {
		pros::lcd::clear_line(2);
	}
}
void initialize() {
    chassis.calibrate();
    chassis.setPose(0, 0, 0); 
    rotation.set_reversed(true);
    lady_brown.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD) ;
	pros::lcd::initialize();
    color_sorter.set_led_pwm(50);
	pros::lcd::register_btn1_cb(on_center_button);
    intake_lift.set_value(false);
    pros::Task screen_task([&]() {
        while (true) {
            pros::lcd::print(0, "X: %f", chassis.getPose().x); 
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); 
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); 
            pros::lcd::print(3 , "Heading: %f", imu.get_heading()); 
            pros::delay(20);
        }   
    });
}
void disabled() {}
void competition_initialize() {

}
void auton_red_far_4ring(){
    lady_brown.move_absolute(500, 200);
    chassis.moveToPoint(0, 20, 1500);
    chassis.turnToHeading(-150, 2000);
    chassis.moveToPoint(8.5, 23, 1500, {false});
    pros::delay(500);
    mogo_mech.set_value(true);
    pros::delay(500);
    intake_stage_1.move_velocity(200);
    intake_stage_2.move_velocity(200);
    chassis.moveToPoint(-9, 24, 2000);
    chassis.moveToPoint(-2, 24, 2000, {false});
    chassis.moveToPoint(-9,  46, 2000);
    chassis.moveToPoint(-9,  36, 1500, {false});
    chassis.turnToHeading(-30, 700);
    chassis.moveToPoint(-20, 44, 1500);
    chassis.moveToPoint(0, 34, 3000, {false});
    chassis.turnToHeading(90, 1000);
    intake_lift.set_value(true);
    chassis.moveToPoint(30, 34, 1500);
    pros::delay(1000);
    intake_lift.set_value(false);
}
void auton_blue_far_4ring(){
    lady_brown.move_absolute(500, 200);
    chassis.moveToPoint(0, 20, 1500);
    chassis.turnToHeading(-150, 2000);
    chassis.moveToPoint(8.5, 23, 1500, {false});
    pros::delay(500);
    mogo_mech.set_value(true);
    pros::delay(500);
    intake_stage_1.move_velocity(200);
    intake_stage_2.move_velocity(200);
    chassis.moveToPoint(-10, 24, 2000);
    chassis.moveToPoint(-2, 24, 2000, {false});
    chassis.moveToPoint(10,  46, 2000);
    chassis.moveToPoint(10,  36, 1500, {false});
    chassis.turnToHeading(30, 700);
    chassis.moveToPoint(20, 46, 1500);
    chassis.moveToPoint(0, 10, 3000, {false});
    chassis.turnToHeading(-90, 1000);
    intake_lift.set_value(true);
    chassis.moveToPoint(-30, 10, 1500);
    pros::delay(1000);
    intake_lift.set_value(false);
}
// chassis.moveToPoint(x, y, timeout, {false});
//USE THIS AUTON FUNTCTION
//raymod is monkey-collin
void auton_red_far_SAWP(){
    intake_lift.set_value(true);
    intake_stage_1.move_velocity(200);
    chassis.moveToPoint(-16, 12, 2000);
    chassis.moveToPoint(3, 11, 3000, {false, 50});
    intake_lift.set_value(false);
    pros::c::delay(2000);
    intake_stage_2.move_velocity(200);
    pros::c::delay(1500);
    intake_stage_2.move_velocity(0);
    lady_brown.move_absolute(0, 200);
    chassis.moveToPoint(0,10.5, 1500);
    chassis.moveToPose(-36, -12, 45,  3000, {false});
    pros::c::delay(3000);
    intake_stage_2.move_velocity(200);
    mogo_mech.set_value(true);
    pros::delay(500);
    intake_stage_1.move_velocity(200);
    intake_stage_2.move_velocity(200);
    chassis.moveToPoint(-30, -35, 3000);
    chassis.moveToPoint(-50, -32, 2000);
    chassis.moveToPoint(-40, -48, 2000, {false});
    chassis.moveToPoint(-50, -48, 2000);
    chassis.moveToPoint(-40, 0, 2000);
    lady_brown.move_absolute(1700, 200);
}
void auton_blue_far_SAWP(){
    chassis.setPose(0, 0, 0);
    intake_lift.set_value(true);
    intake_stage_1.move_velocity(200);
    chassis.moveToPoint(15, 14, 2000);
    chassis.moveToPoint(0.2, 15.5, 3000, {false, 50});
    intake_lift.set_value(false);
    pros::c::delay(2000);
    chassis.turnToHeading(90, 1000);
    intake_stage_2.move_velocity(200);
    pros::c::delay(1500);
    intake_stage_2.move_velocity(0);
    lady_brown.move_absolute(0, 200);
    chassis.moveToPoint(3,12.5, 1500);
    chassis.moveToPose(35 , -18, -45,  3000, {false});
    pros::c::delay(3000);
    intake_stage_2.move_velocity(200);
    mogo_mech.set_value(true);
    pros::delay(500);
    intake_stage_1.move_velocity(200);
    intake_stage_2.move_velocity(200);
    chassis.moveToPoint(31, -33, 2000);
    chassis.moveToPoint(47, -41, 3000);
    chassis.moveToPoint(33, -32, 2000,  {false});
    chassis.moveToPoint(47, -32, 2000);
    chassis.moveToPoint(0, -13, 2000);
    lady_brown.move_absolute(1700, 200);
}
void auton_red_near_2ring(){
    chassis.moveToPoint(0, -14, 10000, {false});
    chassis.turnToHeading(90, 1500);
    chassis.moveToPoint(-3.5, -13.5, 1500);
    intake_stage_2.move_velocity(150);
    pros::delay(1000);
    chassis.moveToPoint(3, -13.5, 1000);
    chassis.moveToPoint(-1000, -11, 1000,{false});
    intake_stage_2.move_velocity(0);
    chassis.moveToPoint(0, -11, 1000);
    chassis.moveToPoint(30, 12, 2000, {false});
    lady_brown.move_absolute(500, 200);
    pros::delay(1700);
    mogo_mech.set_value(true);
    pros::delay(500);
    chassis.moveToPoint(30, 35, 1500);
    chassis.moveToPoint(30, 35, 1500);
    chassis.moveToPoint(20, 28, 2000, {false});
    chassis.moveToPoint(42, 28, 2000);
    chassis.moveToPoint(45, 28, 2000, {false});
    chassis.swingToPoint(45, 38, DriveSide::RIGHT, 1000);
    chassis.moveToPoint(70, -45, 2000, {false});
}

void auton_blue_near_SAWP(){
    chassis.setPose(0, 0, 120);
    chassis.moveToPoint(cos(120)*10, sin(120)*10, 2000, {false});
//     chassis.turnToHeading(-30, 1000);
//     chassis.moveToPoint(7, -28, 1000, {false});
//     pros::delay(1000);
//     mogo_mech.set_value(true);
//     intake_lift.set_value(true);
//     pros::c::delay(1000);
//     chassis.moveToPoint(22, -9, 2000);
//     intake_stage_2.move_velocity(200);
//     intake_stage_1.move_velocity(200);
//     chassis.moveToPoint(15, -8, 1000, {false});
//     intake_lift.set_value(false);
//     pros::c::delay(500);
//     chassis.moveToPoint(-16, -30, 3000);
//     chassis.moveToPoint(-16, -48, 2000, {false});
//     intake_stage_2.move_velocity(0);
//     mogo_mech.set_value(false);
//     pros::delay(2000);
//     mogo_mech.set_value(true);
//     intake_stage_2.move_velocity(200);
//    chassis.moveToPoint(-30, 0, 2000); 


}
void auton_red_near_SAWP(){
    
}
void auton_blue_near_goalrush(){
    
}
void auton_skills(){
    chassis.setPose(0, 0, 0);
    intake_stage_2.move_velocity(200);
    intake_stage_1.move_velocity(200);
    pros::c::delay(2000);
    chassis.moveToPoint(0, 24, 4000, {true, 75});

    chassis.moveToPoint(-28, 16, 2000, {false, 75});
    pros::c::delay(2000);
    mogo_mech.set_value(true);
    pros::c::delay(1000);
    chassis.turnToHeading(0, 1500);
    chassis.moveToPoint(-24, 40, 4000,  {true, 75});
    chassis.turnToHeading(-90, 1500);
    chassis.moveToPoint(-50, 60, 4000,  {true, 75});
    chassis.turnToHeading(180, 1500);
    chassis.moveToPoint(-50, -5, 4000,  {true, 50});
    chassis.moveToPoint(-50, 16, 4000,  {false, 75});
    chassis.moveToPoint(-64, 10, 4000,  {true, 75});
    chassis.moveToPoint(-50, 16, 4000,  {false, 75});
    chassis.moveToPoint(-64, 0, 2000, {false, 75});
    pros::delay(2000);
    intake_stage_2.move_velocity(0);
    mogo_mech.set_value(false);
    chassis.moveToPoint(-52, 8, 4000, {true, 75});

    chassis.moveToPoint(28, 12, 5000, {false, 75});
    pros::c::delay(5000);
    intake_stage_2.move_velocity(200);
    mogo_mech.set_value(true);
    pros::c::delay(1000);
    chassis.turnToHeading(0, 1500);
    chassis.moveToPoint(16, 40, 4000,  {true, 75});
    chassis.turnToHeading(90, 1500);
    chassis.moveToPoint(44,60, 4000,  {true, 75});
    chassis.turnToHeading(180, 1500);
    chassis.moveToPoint(44, -3, 4000,  {true, 50});
    chassis.moveToPoint(44, 16, 4000,  {false, 75});
    chassis.moveToPoint(56, 16, 4000,  {true, 75});
    chassis.moveToPoint(44, 16, 4000,  {false, 75});
    chassis.moveToPoint(52, 0, 2000, {false, 75});
    pros::delay(2000);
    mogo_mech.set_value(false);
    intake_stage_2.move_velocity(0);

    chassis.moveToPoint(44, 16, 4000, {true, 75});
    chassis.moveToPoint(44, 115, 4000);
    intake_stage_2.move_velocity(200);
    chassis.moveToPoint(-10, 115, 4000, {false, 75});
    pros::c::delay(3000);
    mogo_mech.set_value(true);
    chassis.moveToPoint(20, 80, 4000, {true, 75});
    chassis.moveToPoint(56, 115, 4000, {false, 75});
    chassis.moveToPoint(-10, 115, 4000, {false, 75});
    chassis.moveToPoint(-64, 120, 4000);
    intake_stage_2.move_velocity(0);
    mogo_mech.set_value(false);




    

}
void autonomous() {
    auton_red_far_SAWP();
}
void pneumaticsControl(){
            if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)){
                mogo_mech_extended = ! mogo_mech_extended;
                mogo_mech.set_value(mogo_mech_extended);
            }
            if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)){
                intake_lift_extended = ! intake_lift_extended;
                intake_lift.set_value(intake_lift_extended);
            }
            if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)){
                doinker_extended = ! doinker_extended;
                doinker.set_value(doinker_extended);
            }
            
}
void lady_brown_control(){
	if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)&& lady_brown_pos>0){
        if(master.get_digital(pros::E_CONTROLLER_DIGITAL_B)){
            lady_brown.set_zero_position(lady_brown.get_position()+10);
        }else{
            lady_brown_pos--;
        }
	}
	if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)&& lady_brown_pos<4){
        if(master.get_digital(pros::E_CONTROLLER_DIGITAL_B)){
            lady_brown.set_zero_position(lady_brown.get_position()-10);
        }else{
            lady_brown_pos++;
        }
	}
	switch(lady_brown_pos){
		case 1:
            lady_brown.move_absolute(0, -100);
			break;
		case 2:
		    lady_brown.move_absolute(260, 200);
			break;
		case 3:
		    lady_brown.move_absolute(1450, 200);
			break;
        case 4:
            lady_brown.move_velocity(lady_brown.get_position()<2000?master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)?200:0:0);
	}
}
void color_sort(){
    if(color_sorter_blue_enable&&color_sorter.get_hue()>100){
        s2disable=true;
    }else{
        s2disable=false;
    }
    
}
void opcontrol() {
	while (true) {
		pros::lcd::print(0, "X: %f", chassis.getPose().x);
        pros::lcd::print(1, "Y: %f", chassis.getPose().y);
        pros::lcd::print(2, "Theta: %f", chassis.getPose().theta);
        pros::lcd::print(6, "counter %d", counter);
        pros::lcd::print(7,"Hue: %f", color_sorter.get_hue());
		int leftY = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = (master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X));
		intake_stage_1.move_velocity(master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)?200:master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)?-200:0);
        if(!s2disable){
            intake_stage_2.move_velocity(master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)?200:master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)?-200:0);
        }else{
           
        }
		pneumaticsControl();
		lady_brown_control();
        color_sort();
		chassis.arcade(leftY, rightX); 
		pros::delay(20);
	}
}