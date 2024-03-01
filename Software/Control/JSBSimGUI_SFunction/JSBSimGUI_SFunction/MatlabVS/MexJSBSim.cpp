// MexJSBSim.cpp : Defines the entry point for the console application.
// by A. Bryant Nichols Jr. 2007


#include "stdafx.h"
#include <Windows.h>
#include "mex.h"
#include "mclcppclass.h"
#include "matrix.h"
#include <iostream>
#include <string>
#include <vector>
#include <FGFDMExec.h>
#include <models/FGPropagate.h>
#include <models/FGAuxiliary.h>
#include <models/FGFCS.h>

#include "JSBSimInterface.h"

using namespace std;

// function prototypes
bool StartJSBSim(string aircraftname);
bool QueryJSBSim(string prop);

int iCase=0;
double dist1=0;
double dist2=0;
double *data1;
TCHAR szDummy[_MAX_PATH];

// this object should be persistent in memory until
// the MEX-function is cleared by Matlab
JSBSim::FGFDMExec FDMExec;
JSBSimInterface JI(&FDMExec);

void helpOptions() {
	mexPrintf("function usage:\n"                                           );
	mexPrintf("result = MexJSBSim( string_directive [, string, value] )\n"  );
	mexPrintf("\n"                                                          );
	mexPrintf("Examples:\n"                                                 );
	mexPrintf("    res = MexJSBSim('help');\n"                              );
	mexPrintf("			returns 1 (always)\n"                               );
	mexPrintf("    res = MexJSBSim('open','c172r');\n"                      );
	mexPrintf("			returns 1 if success, 0 otherwise\n"                );
	mexPrintf("    res = MexJSBSim('get','fcs/elevator-cmd-norm')\n"        );
	mexPrintf("			returns the value of the property,\n"               );
	mexPrintf("			or the string 'Property not found'\n"               );
	mexPrintf("    res = MexJSBSim('set','fcs/elevator-cmd-norm',-0.5)\n"   );
	mexPrintf("			returns 1 if success, 0 otherwise\n"                );
}

// the gataway function
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{

	string aircraftName = "";
	string option = "";

	/*
	if (myobj==NULL) // this is the first call to MEX-function
	{ 
		mexPrintf("First call to MEX-file\n");
		myobj = (void *)FDMExec;
		mexMakeMemoryPersistent(myobj);
		// register a mexAtExit function to free allocated memory 
		// in the event this MEX-file is cleared
		mexAtExit(exitFcn);
	}
	*/

	if (nrhs>0)
	{
		char buf[128];
		mwSize buflen;
		buflen = mxGetNumberOfElements(prhs[0]) + 1;
		mxGetString(prhs[0], buf, buflen);

		plhs[0] = mxCreateDoubleMatrix(1, 1, mxREAL);

		if (nrhs==1)
		{
			option = string(buf);

			if ( option == "help")
			{
				helpOptions();
				*mxGetPr(plhs[0]) = 1;
			}
			else
			{
				if ( option == "catalog")
				{
					JI.PrintCatalog();
					*mxGetPr(plhs[0]) = 1;
				}
				else
				{
					mexPrintf("Uncorrect call to this function.\n\n");
					helpOptions();
					*mxGetPr(plhs[0]) = 0;
				}
			}
		}
		if (nrhs>1)
		{
			option = string(buf);

			if ( option == "open" )
			{
				if ( !JI.Open(prhs[1]) ) // load a/c in JSBSim
					mexPrintf("JSBSim could not be started.\n");
				else
					*mxGetPr(plhs[0]) = 1;

			}
			if ( option == "get" )
			{
				
				// TO DO: if ( !JI.Get(prhs[1]) )
				double value;
				if ( !JI.GetPropertyValue(prhs[1],value) )
				{
					mexPrintf("Check property name.\n");
					*mxGetPr(plhs[0]) = 0;
					return;
				}
				else
					*mxGetPr(plhs[0]) = value;
			}
			if ( option == "set" )
			{
				if (nrhs>2)
				{
					if ( !JI.SetPropertyValue(prhs[1],prhs[2]) )
					{
						mexPrintf("Property could not be set.\n");
						*mxGetPr(plhs[0]) = 0;
					}
					else
						*mxGetPr(plhs[0]) = 1;
				}
				else
					mexPrintf("ERROR: uncorrect use of 'set' option.\n");
			}
			if ( option == "init" )
			{
					if ( !JI.Init(prhs[1]) )
					{
						mexPrintf("Initialization failed.\n");
						*mxGetPr(plhs[0]) = 0;
					}
					else
						*mxGetPr(plhs[0]) = 1;
			}
		} // end of nrhs>1
	} // end of nrhs>0
	else
		helpOptions();

	return;

}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

/*
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
bool StartJSBSim(string ac_name)
{
    string RootDir = "JSBSim/";

	// mexEvalString("plot(sin(0:.1:pi))");

	printf("INSIDE StartJSBSim.\n");
	mexPrintf("Aircraft name: %s\n",ac_name.c_str());
	
	// JSBSim stuff

	mexPrintf("Setting up JSBSim ...\n",ac_name.c_str());

    FDMExec.SetAircraftPath(RootDir + "aircraft");
    FDMExec.SetEnginePath(RootDir + "engine");
    FDMExec.SetSystemsPath(RootDir + "systems");

	mexPrintf("\tloading aircraft...\n");

    if ( ! FDMExec.LoadModel( RootDir + "aircraft",
                              RootDir + "engine",
                              RootDir + "systems",
                              ac_name)) 
	{
		mexPrintf("ERROR: JSBSim could not be started\n\n");
		// delete FDMExec;
		return 0;
    }
	
	// Print AC name
	mexPrintf("\tModel %s loaded.\n", FDMExec.GetModelName().c_str() );

	string str_elevator_ctrl = FDMExec.QueryPropertyCatalog("fcs/elevator-cmd-norm");
	if ( str_elevator_ctrl.length()>0 )
	{
		mexPrintf("\tGot %s\n", str_elevator_ctrl.c_str() );
		FDMExec.GetFCS()->SetDeCmd(-0.25);
		FDMExec.GetFCS()->Run();
		mexPrintf("\t\tSet elevator pos to %f (deg)\n", FDMExec.GetFCS()->GetDePos()*180.0/M_PI );

		FDMExec.GetFCS()->SetDeCmd(-1.00);
		FDMExec.GetFCS()->Run();
		mexPrintf("\t\tSet elevator pos to %f (deg)\n", FDMExec.GetFCS()->GetDePos()*180.0/M_PI );
	}

	// Do initialize
	FDMExec.GetIC()->SetAltitudeFtIC(1000.0);
	FDMExec.GetIC()->SetMachIC(0.1);
	FDMExec.GetIC()->SetAlphaDegIC(1.0);
	FDMExec.GetIC()->SetBetaDegIC(0.0);
	FDMExec.GetIC()->SetFlightPathAngleDegIC(0.0);
	FDMExec.GetIC()->SetPhiDegIC(0.0);
	FDMExec.GetIC()->SetPsiDegIC(40.0);
	FDMExec.RunIC(); //loop JSBSim once w/o integrating
	mexPrintf("\tInitialization done.\n");

	//FDMExec.Run();

	// Calculate state derivatives
	FDMExec.GetPropagate()->CalculatePQRdot();      // Angular rate derivative
	FDMExec.GetPropagate()->CalculateUVWdot();      // Translational rate derivative
	FDMExec.GetPropagate()->CalculateQuatdot();     // Angular orientation derivative
	FDMExec.GetPropagate()->CalculateLocationdot(); // Translational position derivative

	// FDMExec.GetAuxiliary()->Run();
	JSBSim::FGColumnVector3 euler_rates = FDMExec.GetAuxiliary()->GetEulerRates();

	mexPrintf("\tState derivatives calculated.\n");

	mexPrintf("\tV true %f (ft/s)\n",FDMExec.GetAuxiliary()->GetVt());
	mexPrintf("\tU_dot %f (ft/s/s)\n",FDMExec.GetPropagate()->GetUVWdot(1));
	mexPrintf("\tV_dot %f (ft/s/s)\n",FDMExec.GetPropagate()->GetUVWdot(2));
	mexPrintf("\tW_dot %f (ft/s/s)\n",FDMExec.GetPropagate()->GetUVWdot(3));
	mexPrintf("\tP_dot %f (ft/s/s)\n",FDMExec.GetPropagate()->GetPQRdot(1));
	mexPrintf("\tQ_dot %f (ft/s/s)\n",FDMExec.GetPropagate()->GetPQRdot(2));
	mexPrintf("\tR_dot %f (ft/s/s)\n",FDMExec.GetPropagate()->GetPQRdot(3));
	mexPrintf("\tTheta_dot %f (ft/s/s)\n",euler_rates(2));

	//mexPrintf("Closing JSBSim.\n",ac_name.c_str());

	return 1;
}
*/
