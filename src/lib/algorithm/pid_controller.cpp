#include "edu_robot/algorithm/pid_controller.hpp"

namespace eduart {
namespace robot {
namespace algorithm {

void Pid::reset()
{
  _e_integral = 0.0;
  _e_prev = 0.0;
  _set_point_prev = 0.0;
	_previous_feedback = 0.0;
}

double Pid::process(const double set_point, const double feedback, const double dt)
{
	const double filtered_feedback = (1.0 - parameter.input_filter_weight) * _previous_feedback 
	                               + feedback * parameter.input_filter_weight;
	const double e = set_point - filtered_feedback;

	_e_integral += e * dt;

	double fy = parameter.kp * e // KP
            + parameter.ki * _e_integral // KI
            + parameter.kd * (e - _e_prev) / dt; // KD

	if (fy > parameter.limit) {
		fy = parameter.limit;
	
		if(parameter.use_anti_windup == true) {
			_e_integral -= e * dt;
		}
	}
	else if(fy < -parameter.limit) {
		fy = -parameter.limit;

		if(parameter.use_anti_windup == true) {
			_e_integral -= e * dt;
		}
	}

	_e_prev = e;
  _set_point_prev = set_point;
	_previous_feedback = filtered_feedback;
	return fy;  
}

} // end namespace algorithm
} // end namespace robot
} // end namespace eduart
