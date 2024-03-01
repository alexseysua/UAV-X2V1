Before using this SFunction, you must first either download the JSBSim Flight Dynamic Model project or create a JSBSim folder and populate it with an aircraft folder and an engine folder. Of course, these two folders would need to have any engine and aircraft models that you wish to run. 

To download JSBSim, go to http://sourceforge.net/projects/jsbsim/files/ and download the JSBSim-1.0rc2.tar.gz. Take the unzipped JSBSim-1.0 folder (within the JSBSim-1.0.rc2 folder,) rename it to JSBSim and place it into the JSBSim_SFunction project folder. This will allow the SFunction to access the aircraft data folder and parse the aC model files.

If you do not want to download all of JSBSim, the simply use the empty JSBSim folder that has within it an aircraft folder that you can populate with any aircraft folders containing xml files you wish to run, and an engine folder that should have any engine files needed by your aircraft file. The empty folders have already been created for you. 

*** PLEASE NOTE- If you do not have either the Visual Studio 2008 Pro or Express editions, you may want to install either one. Some users have reported some problems running the compiled MEX file, but that by installing VS8 the SFunction ran fine. 

To start-

Open the JSBSimGUI.fig file and either type the name of your simulink model or leave "jsbsimgui_test" just as it appears in the textbox. Press "Load Model" to open the Simulink model. Press the "Help" button for further information on how to use the GUI.

You can run any JSBSim aircraft model as long as it resides in the JSBSim aircraft data folder and it has an engine file in the engine folder.

NOTE: Currently the SFunction is based on the JSBSim Release Candidate 1.0 rc2, not the newer CVS versions. Sorry if that is a major problem for anyone. Once the CVS changes are in a release version, the SFunction will be updated to work with the latest release version. 

2/25/10
Brian Mills

