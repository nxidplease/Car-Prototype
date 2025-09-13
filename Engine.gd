extends Node

class_name CarEngine

signal update_rpm
signal gear_shift

# Power curve
# adjusts cure baseline y
export(int, -10, 1500) var a = 1470

#adjusts peak magnitude
export(int, -10, 600) var b = 564

# adjusts ends slope
export(float, 0, 10, 0.05) var c = 0.32

# adjusts peak x position
export(int, -10, 8000) var d = 6000

# adjusts ends slope
export(int, -10, 4000) var f = 1850


export(float, 0.001, 2.5, 0.05) var engine_break: float = 0.3

export(float, 0.001, 0.9, 0.005) var engine_friction: float = 0.7
# R, N, 1, 2, 3, 4, 5
export(Array, float) var gear_ratios = [-2.92, 0, 2.5, 1.61, 1.10, 0.81, 0.68];

export(float) var final_drive: float = 4.1

export(float) var shift_duration_sec: float = 0.5

var currentRpm: float = 1000

# Index into gear_ratios
var currentGear: int = 1

const NEUTRAL_GEAR = 1

var throttle: float = 0

var clutch: float = 1.0

var accelerating: bool = false

var nextGear: int = 0;
var prevGear: int = 0;

var time_since_shift: float = 0

var isShifting: bool = false

func _ready():
	pass

func _get_engine_torque_by_rpm(rpm):
	return (a - b) * exp(-pow(c*(rpm-d) / f, 2)) + b
	
func get_engine_torque():
	return _get_engine_torque_by_rpm(currentRpm)

func get_torque_at_wheels():
	return _get_engine_torque_by_rpm(currentRpm) * gear_ratios[currentGear] * final_drive * clutch
	
func get_rpm_at_wheels():
	
	var denom = (gear_ratios[currentGear] * final_drive * clutch)
	
	if denom == 0:
		return 0
	
	return currentRpm / denom
	
func gear_up():
	if currentGear + 1 < gear_ratios.size():
		isShifting = true
		nextGear = currentGear + 1
		prevGear = currentGear
		currentGear = 1
		
func gear_down():
	if currentGear - 1 >= 0:
		isShifting = true
		nextGear = currentGear - 1
		prevGear = currentGear
		currentGear = 1
		
func match_rpm(new_gear: int):
	
	# When switch to/from Neutral do no rev matching
	if new_gear == NEUTRAL_GEAR || prevGear == NEUTRAL_GEAR:
		return
	
	currentRpm *= (gear_ratios[new_gear] / gear_ratios[prevGear])
		
func adjust_throttle(dt):
	if accelerating:
		throttle = min(throttle + 0.5 * dt, 1)
	else:
		throttle = max(throttle - 0.5 * dt, 0)
		
func update_engine(rolling_rpm: float, dt: float):
	
	if time_since_shift >= shift_duration_sec:
		time_since_shift = 0
		isShifting = false
		match_rpm(nextGear)
		currentGear = nextGear
		emit_signal("gear_shift", currentGear)
		
	if isShifting:
		time_since_shift += dt
	
#	if currentGear == 0:
#		print("Help!")

	var torque_factor = 0 if isShifting else throttle
	
	var rolling_eng_rpm = rolling_rpm * final_drive * gear_ratios[currentGear]
	var rpmChange = _get_engine_torque_by_rpm(currentRpm) * torque_factor - engine_friction * currentRpm
	
	# If not in N gear sync engine to wheels
	if currentGear != 1:
#		Logger.info("Engine rpm: %.3f rpm according to wheels: %.3f diff: %.3f" %[currentRpm, rolling_eng_rpm, rolling_eng_rpm - currentRpm])
		rpmChange += engine_break * (clutch * rolling_eng_rpm - currentRpm)
	
	currentRpm = clamp(currentRpm + rpmChange * dt, 1000, 8500)
	emit_signal("update_rpm", currentRpm)
