/*****************************************************************************
*   File name: jsbgodot.hpp
*   File description: 
*****************************************************************************/
#ifndef JSBGODOT_HPP
#define JSBGODOT_HPP

#include <Godot.hpp>
#include <Sprite.hpp>
#include <Spatial.hpp>
#include "FGFDMExec.h"
#include <math.h>

namespace godot {

class JSBGodot : public Spatial {
	GODOT_CLASS(JSBGodot, Spatial)

private:
	JSBSim::FGFDMExec *FDMExec;
	bool do_scripted;

	double input_pitch = 0.0;
	double input_roll = 0.0;
	double input_rudder = 0.0;
	double input_throttle = 1.0;

	void copy_inputs_to_JSBSim();

	void copy_outputs_from_JSBSim();

	Vector3 start_pos;
	Vector3 start_rot;

public:
	static void _register_methods();

	JSBGodot();
	~JSBGodot();

	void _init(); // our initializer called by Godot

	void _process(double delta);

	void _input(const Ref<InputEvent> event);

	void _physics_process(const real_t delta);

	void initialise();

	Vector3 _convert_location_to_godot(double latitude, double longitude, double altitude_meters);

	Vector3 _convert_location_from_godot(Vector3 translation);

	double _scale_degrees(double degrees);
};

} // namespace godot

#endif /* JSBGODOT_HPP */