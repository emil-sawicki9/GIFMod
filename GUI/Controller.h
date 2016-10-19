#pragma once
#include "Sensor.h"

enum controller_type {P=0, PI=1, PD=2, PID=3, PID_NoOvershoot=4};

class CController
{
public:
	CController();
	~CController();
	CController(const CController &M);
	CController& CController::operator=(const CController &BB);
	string name; 
	CBTC output;
	string type;  //P,PI,PD,PID,... 	
	CSensor *Sensor;
	double calc_value(double t, int experiment_id);
	void append(double t, double C); 
	vector<double> params; // for PID controller params[0] = k_p, params[1] = k_i, params[2] = k_d; params[3] = set point
	double get_P(double t, int experiment_id);
	double get_I(double t, int experiment_id);
	double get_D(double t, int experiment_id);
	void CController::set_val(string S, double val);
	double value;
	double interval;
	double min_val = 0;
	double max_val = 1e12;
	//JA:
	int type_ID;
	void get_type_ID();
	int tunning_method; //0=manual, 1=Ziegler–Nichols
	CVector ultimate_params = CVector(2); //For Ziegler–Nichols tunning: ultimate_params(0) = Ku, ultimate_params(1) = Tu
	void set_Ziegler_Nichols_params(CVector ultimate_params);
};

