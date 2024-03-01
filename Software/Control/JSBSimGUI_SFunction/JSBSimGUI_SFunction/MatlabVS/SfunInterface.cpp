#include "StdAfx.h"
#include <Windows.h>
#include "mex.h"
#include "mclcppclass.h"
#include "matrix.h"
#include <iostream>
#include <string>
#include <vector>
#include "JSBSimInterface.h"

using namespace std;
double delta_t;
void *j[];


void SetJSBSimDeltaT(double dt)
{
	delta_t = dt;
	mexPrintf("\t SFunInterface delta_t %f\n",delta_t);
	JSBSim::FGFDMExec FDMExec;
	j[0] = (void *) new JSBSimInterface(&FDMExec, delta_t);
}


// Simple external interface functions called from JSBSim_SFunction to bridge SFunction and JSBSimInterface


bool SfunInterfaceOpen(string prop)
{
	JSBSimInterface *ji = (JSBSimInterface *) j[0];
	if(!ji->Open(prop))
		return 0;
	else 
		return 1;
}

void SfunInterfaceInit(const mxArray *prhs)
{
	JSBSimInterface *ji = (JSBSimInterface *) j[0];
	ji->Init(prhs);
	return;
}
/*
void SfunInterfaceGetEngNum(real_T *engnum)
{
	JI.GetEngNum(engnum);
	return;
}

void SfunInterfaceGetEngType(string* type_eng)
{
	JI.GetEngType(type_eng);
	return;
}
*/
void SfunInterfaceRun(double *inputs, double *states, double *controls, double *propulsion, double *outputs)
{
	JSBSimInterface *ji = (JSBSimInterface *) j[0];
	ji->UpdateStates(inputs, states, controls, propulsion, outputs);
	return;
}
void SfunInterfaceReset(void)
{
	JSBSimInterface *ji = (JSBSimInterface *) j[0];
	ji->ResetToInitialCondition();
	return;
}
void PrintCatalog(void)
{
	JSBSimInterface *ji = (JSBSimInterface *) j[0];
	ji->PrintCatalog();
	return;
}
