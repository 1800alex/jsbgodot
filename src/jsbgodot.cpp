/*****************************************************************************
*   File name: jsbgodot.cpp
*   File description: 
*****************************************************************************/
#define JSBGODOT_CPP_SRC

#include "jsbgodot.hpp"
#include "FGFDMExec.h"
#include <InputEvent.hpp>
#include <unistd.h>
#include "models/FGFCS.h"
#include "models/FGAircraft.h"
#include "initialization/FGInitialCondition.h"
#include "models/FGPropulsion.h"
#include <models/FGAuxiliary.h>
#include <stdlib.h>
#include <initialization/FGTrim.h>
/**
 * Simple script to connect JSBSim to a Godot 3D Spatial body
 *
 * On JSBSim: http://jsbsim.sourceforge.net/JSBSimReferenceManual.pdf
 *      https://github.com/JSBSim-Team/jsbsim/issues/396
 *
 * On Godot: https://docs.godotengine.org/en/stable/tutorials/plugins/gdnative/gdnative-cpp-example.html
 */

#define METER_TO_FEET 3.28084f
#define FEET_TO_METER 0.3048f

using namespace godot;

void JSBGodot::_register_methods()
{
	register_method("_process", &JSBGodot::_process);
	register_method("_physics_process", &JSBGodot::_physics_process);
	register_method("_input", &JSBGodot::_input);
	register_property<JSBGodot, double>("input_pitch", &JSBGodot::input_pitch, 0.0);
	register_property<JSBGodot, double>("input_roll", &JSBGodot::input_roll, 0.0);
	register_property<JSBGodot, double>("input_rudder", &JSBGodot::input_rudder, 0.0);
	register_property<JSBGodot, double>("input_throttle", &JSBGodot::input_throttle, 1.0);
}

JSBGodot::JSBGodot()
{
}

JSBGodot::~JSBGodot()
{
	delete FDMExec;
}

void JSBGodot::_init()
{
	//setenv("JSBSIM_DEBUG", "2", 1);
	FDMExec = new JSBSim::FGFDMExec();
	char cwd[1024];
	char *thing = getcwd(cwd, 1024);
	printf("cwd: %s\n", thing);
	FDMExec->SetRootDir(SGPath("./submodules/jsbgodot/jsbsim"));
	FDMExec->SetAircraftPath(SGPath("aircraft"));
	FDMExec->SetEnginePath(SGPath("engine"));
	FDMExec->SetSystemsPath(SGPath("systems"));
	do_scripted = false;
	if(do_scripted)
	{
		FDMExec->LoadScript(SGPath("scripts/Short_S23_4.xml"));
		FDMExec->RunIC();
	}
	else
	{
		FDMExec->LoadModel(SGPath("aircraft"),
			SGPath("engine"),
			SGPath("systems"),
			"f16");
		//        initialise();
		FDMExec->GetIC()->Load(SGPath("reset00.xml"));

		std::shared_ptr<JSBSim::FGInitialCondition> ic = FDMExec->GetIC();
		ic->SetLatitudeDegIC(0.0);
		ic->SetLongitudeDegIC(0.0);
		ic->SetAltitudeASLFtIC(100 * METER_TO_FEET);
		ic->SetMachIC(0.3);
		copy_inputs_to_JSBSim();

		FDMExec->RunIC();
		//        FDMExec->DoTrim(JSBSim::tFull);
	}
	//    std::shared_ptr<JSBSim::FGAircraft> ac = FDMExec->GetAircraft();
	//    std::shared_ptr<JSBSim::FGPropulsion> prop = FDMExec->GetPropulsion();
	//    std::shared_ptr<JSBSim::FGEngine> engine = prop->GetEngine(-1);
}

void JSBGodot::initialise()
{
	std::shared_ptr<JSBSim::FGInitialCondition> ic = FDMExec->GetIC();

	start_pos = JSBGodot::_convert_location_from_godot(to_global(get_translation()));
	start_rot = get_rotation();

	ic->SetLatitudeDegIC(start_pos.x);
	ic->SetLongitudeDegIC(start_pos.z);
	ic->SetAltitudeASLFtIC(start_pos.y * METER_TO_FEET);
	ic->SetPhiDegIC(start_rot.z);
	ic->SetThetaDegIC(start_rot.x); //bank/roll
	ic->SetThetaDegIC(start_rot.x); //pitch
	ic->SetPsiDegIC(start_rot.y);	//heading
	ic->SetMachIC(0.3);
}

void JSBGodot::copy_inputs_to_JSBSim()
{
	if(do_scripted)
	{
		return;
	}
	//    return;
	std::shared_ptr<JSBSim::FGFCS> FCS = FDMExec->GetFCS();

	FCS->SetDeCmd(input_pitch);
	FCS->SetDaCmd(input_roll);
	FCS->SetDrCmd(input_rudder);
	FCS->SetThrottleCmd(-1, input_throttle);
}

void JSBGodot::copy_outputs_from_JSBSim()
{
	std::shared_ptr<JSBSim::FGPropagate> Propagate = FDMExec->GetPropagate();
	std::shared_ptr<JSBSim::FGAuxiliary> Auxiliary = FDMExec->GetAuxiliary();
	//    std::shared_ptr<JSBSim::FGAtmosphere> Atmosphere = FDMExec->GetAtmosphere();
	//    std::shared_ptr<JSBSim::FGAccelerations> Accelerations = FDMExec->GetAccelerations();

	JSBSim::FGLocation location = Propagate->GetLocation();
	double altitude = Propagate->GetAltitudeASLmeters();
	double latitude = location.GetLatitudeDeg();
	double longitude = location.GetLongitudeDeg();

	double bank = Propagate->GetEuler(JSBSim::FGForce::ePhi);
	double pitch = Propagate->GetEuler(JSBSim::FGForce::eTht);
	double heading = Propagate->GetEuler(JSBSim::FGForce::ePsi);

	Vector3 newRot = Vector3(pitch, JSBGodot::_scale_degrees(heading), bank);
	set_rotation(newRot);
	// printf("ROT: %.02f,%.02f,%.02f\n", newRot.x, newRot.y, newRot.z);

	Vector3 newPos = JSBGodot::_convert_location_to_godot(latitude, longitude, altitude);
	set_translation(newPos);
	// printf("OLD: %f,%f,%f\n", newPos.x, newPos.y, newPos.z);
	// printf("LLA: %f,%f,%f\n", latitude, longitude, altitude);
	printf("Altitude: %.01f  Ground Speed: %.01f\n", altitude, Auxiliary->GetVground());
}

void JSBGodot::_input(const Ref<InputEvent> event)
{
}

void JSBGodot::_physics_process(const real_t delta)
{
	copy_inputs_to_JSBSim();
	FDMExec->Run();
	copy_outputs_from_JSBSim();
}

void JSBGodot::_process(double delta)
{
}

Vector3 JSBGodot::_convert_location_to_godot(double latitude, double longitude, double altitude_meters)
{
	//Length in meters of 1° of latitude = multiply by 111.32 km
	//Length in meters of 1° of longitude = multiply by 40075 km * cos( latitude ) / 360
	return (Vector3(longitude * 40075000 * (cos(latitude) / 360), altitude_meters, latitude * 111139));
}

Vector3 JSBGodot::_convert_location_from_godot(Vector3 translation)
{
	double latitude = translation.z / 111139;
	double longitude = translation.x / (40075000 * (cos(latitude) / 360));
	return (Vector3(longitude, translation.y, latitude));
}

double JSBGodot::_scale_degrees(double degrees)
{
	double scaled_degrees;

	if(degrees < 0.0f)
	{
		scaled_degrees = std::fmod(degrees, -360.0f);
	}
	else
	{
		scaled_degrees = std::fmod(degrees, 360.0f);
	}

	if((scaled_degrees < -180.0f) || (scaled_degrees > 180.0f))
	{
		if(scaled_degrees > 180.0f)
		{
			scaled_degrees -= 360.0f;
		}
		else if(scaled_degrees < -180.0f)
		{
			scaled_degrees += 360.0f;
		}
	}

	return (scaled_degrees);
}
