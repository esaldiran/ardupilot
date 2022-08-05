#include <AP_HAL/AP_HAL.h>

#include "AC_CustomControl.h"

#if CUSTOMCONTROL_ENABLED

#include "AC_CustomControl_Backend.h"

// table of user settable parameters
const AP_Param::GroupInfo AC_CustomControl::var_info[] = {
    // @Param: CONT_TYPE
    // @DisplayName: Attitude control type
    // @Description: Attitude control type to be used
    // @Values: 0:None
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO_FLAGS("CONT_TYPE", 1, AC_CustomControl, _controller_type, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: CUST_CNT_MSK
    // @DisplayName: Custom Controller bitmask
    // @Description: Custom Controller bitmask to chose which axis to run
    // @Bitmask: 0:Roll, 1:Pitch, 2:Yaw
    // @User: Advanced
    AP_GROUPINFO("AXIS_MASK", 2, AC_CustomControl, _custom_controller_mask, 0),

    AP_GROUPEND
};

const struct AP_Param::GroupInfo *AC_CustomControl::_backend_var_info[CUSTOMCONTROL_MAX_TYPES];

AC_CustomControl::AC_CustomControl(AP_AHRS_View*& ahrs, AC_AttitudeControl_Multi*& atti_control, AP_MotorsMulticopter*& motors, float dt) :
    _dt(dt),
    _ahrs(ahrs),
    _atti_control(atti_control),
    _motors(motors)
{
    AP_Param::setup_object_defaults(this, var_info);    
}

void AC_CustomControl::init(void)
{
    switch (CustomControlType(_controller_type))
    {
        case CustomControlType::CONT_NONE:
            break;
        default:
            return;
    }

    if (_backend[get_type()] && _backend_var_info[get_type()]) {
        AP_Param::load_object_from_eeprom(_backend[get_type()], _backend_var_info[get_type()]);
    }
}

// run custom controller if it is activated by RC switch and appropriate type is selected
void AC_CustomControl::update(void)
{
    if (is_safe_to_run()) {
        Vector3f motor_out_rpy;

        motor_out_rpy = _backend[get_type()]->update();   

        motor_set(motor_out_rpy);
    }
}

// choose which axis to apply custom controller output
void AC_CustomControl::motor_set(Vector3f rpy) {
    if (_custom_controller_mask & (uint8_t)CustomControlOption::ROLL) {
        _motors->set_roll(rpy.x);
    }
    if (_custom_controller_mask & (uint8_t)CustomControlOption::PITCH) {
        _motors->set_pitch(rpy.y);
    }
    if (_custom_controller_mask & (uint8_t)CustomControlOption::YAW) {
        _motors->set_yaw(rpy.z);
    }
}

// move main controller's target to current states, reset filters, 
// and move integrator to motor output
// to allow smooth transition to the primary controller
void AC_CustomControl::reset_main_atti_controller(void) 
{
    // reset attitude and rate target, if feedforward is enabled
    if (_atti_control->get_bf_feedforward()) {
        _atti_control->relax_attitude_controllers();
    }

    Vector3f gyro_latest = _ahrs->get_gyro_latest();

    _atti_control->get_rate_roll_pid().set_integrator(_atti_control->rate_bf_targets().x - gyro_latest.x, _motors->get_roll());
    _atti_control->get_rate_pitch_pid().set_integrator(_atti_control->rate_bf_targets().y - gyro_latest.y, _motors->get_pitch());
    _atti_control->get_rate_yaw_pid().set_integrator(_atti_control->rate_bf_targets().z - gyro_latest.z, _motors->get_yaw());
}

void AC_CustomControl::set_custom_controller(bool enabled)
{
    _custom_controller_active = false;

    // don't allow accidental main controller reset without active custom controller
    if (_controller_type == 0) {
        gcs().send_text(MAV_SEVERITY_INFO, "Custom controller is not enabled");        
        return;
    }

    // controller type is out of range
    if (_controller_type > CUSTOMCONTROL_MAX_TYPES) {
        gcs().send_text(MAV_SEVERITY_INFO, "Custom controller type is out of range");        
        return;
    }

    // backend type is not configured or changed mid-flight
    if (_backend[get_type()] == nullptr) {
        gcs().send_text(MAV_SEVERITY_INFO, "Reboot to enable selected custom controller");
        return;
    }

    // reset main controller
    if (!enabled) {
        gcs().send_text(MAV_SEVERITY_INFO, "Custom controller is OFF");
        // don't reset if the empty backend is selected
        if (_controller_type > 1) {
            reset_main_atti_controller();
        }
    }

    if (enabled && _controller_type > 0) {
        // reset custom controller filter, integrator etc.
        _backend[get_type()]->reset();
        gcs().send_text(MAV_SEVERITY_INFO, "Custom controller is ON");
    } 

    _custom_controller_active = enabled;
}

// check that RC switch is on, backend is not changed mid flight and controller type is selected
bool AC_CustomControl::is_safe_to_run(void) {
    if (_custom_controller_active == true && (_controller_type > 0) 
        && (_controller_type <= CUSTOMCONTROL_MAX_TYPES) && _backend[get_type()] != nullptr )    
    {
        return true;
    }

    return false;
}

#endif