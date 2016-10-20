#include "Controller.h"
#include "StringOP.h"


CController::CController()
{
	params.resize(10);
}


CController::~CController()
{
}

CController::CController(const CController &M)
{
	name = M.name;
	params = M.params;
	type = M.type;
	value = M.value;
	interval = M.interval;
	min_val = M.min_val;
	max_val = M.max_val;
}

CController& CController::operator=(const CController &M)
{
	params = M.params;
	name = M.name;
	type = M.type;
	value = M.value;
	interval = M.interval;
	min_val = M.min_val;
	max_val = M.max_val;
	return *this;

}

double CController::calc_value(double t, int experiment_id)
{
	value += params[0] * (get_P(t, experiment_id) + params[1] * get_I(t, experiment_id) + params[2] * get_D(t, experiment_id));
	value = min(value, max_val);
	value = max(value, min_val);
	append(t, value);
	return value;
}


void CController::append(double t, double C)
{
	for (int i = output.n - 1; i >= 0; i--)
	{
		if (output.t[i] == t) output.C[i] = C;
		return;
	}
	output.append(t, C);
	return;

}

double CController::get_P(double t, int experiment_id)
{
	if (tolower(type) == "pid")
		return (Sensor->output[experiment_id].interpol(t) - params[3]);

}
double CController::get_I(double t, int experiment_id)
{
	if (tolower(type) == "pid")
		return (Sensor->output[experiment_id].integrate(t) - params[3]*(t-Sensor->output[experiment_id].t[0]));

}
double CController::get_D(double t, int experiment_id)
{
	if (tolower(type) == "pid")
		return (Sensor->output[experiment_id].slope(t));

}

void CController::set_val(string S, double val)
{
	if (tolower(S) == "kp") params[0] = val;
	if (tolower(S) == "ki") params[1] = val;
	if (tolower(S) == "kd") params[2] = val;
	if (tolower(S) == "set_point") params[3] = val;

}

void CController::set_Ziegler_Nichols_params(CVector ultimate_params)
{	
	/* refrence: en.wikipedia.org/wiki/Ziegler%E2%80%93Nichols_method */
	double Ku = ultimate_params[0];
	double Tu = ultimate_params[1];
	double dt_gain = interval;

	switch (type_ID)
	{
		case 0:  // P
		{
			params[0] = 0.5*Ku;
			params[1] = 0;
			params[2] = 0;
		}
		case 1:  // PI
		{
			params[0] = 0.45*Ku;
			params[1] = 1.2*dt_gain/Tu;
			params[2] = 0;
		}
		case 2:  // PD
		{
			params[0] = 0.8*Ku;
			params[1] = 0;
			params[2] = Tu*dt_gain/8;
		}
		case 3:  // PID
		{
			params[0] = 0.6*Ku;
			params[1] = 2*dt_gain/Tu;
			params[2] = Tu*dt_gain/8;
		}
		case 4:  // PID_NoOvershoot
		{
			params[0] = 0.2*Ku;
			params[1] = 2*dt_gain/Tu;
			params[2] = Tu*dt_gain/3;
		}
		default:
		{
			params[0] = 0.5*Ku;
			params[1] = 0;
			params[2] = 0;
		}
	}
}

void CController::get_type_ID() {
	if (tolower(type) == "p")
		type_ID = 0;
	else if (tolower(type) == "pi")
		type_ID = 1;
	else if (tolower(type) == "pd")
		type_ID = 2;
	else if (tolower(type) == "pid")
		type_ID = 3;
	else if (tolower(type) == "pid_noovershoot")
		type_ID = 4;
}