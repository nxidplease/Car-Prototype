extends RigidBody

class_name Car

signal update_offset

export(Curve) var engineForceCurve: Curve

export(int) var maxSpeed

export(int) var maxEngineForce

export(float) var rr_coefficient = 0.01

export(float) var brake_coefficient = 1

export(float) var steering_speed_deg = 20

export(float) var max_steering_angle_deg = 65

export(float) var max_steer_force = 30.0

onready var wheels = [$FR, $FL, $RR, $RL]

onready var steering_speed_rad = deg2rad(steering_speed_deg)
onready var max_steering_angle_rad = deg2rad(max_steering_angle_deg)

var engineForce = [Vector3.ZERO, Vector3.ZERO, Vector3.ZERO, Vector3.ZERO]
var rrForce = [Vector3.ZERO, Vector3.ZERO, Vector3.ZERO, Vector3.ZERO]
var brakeForce = [Vector3.ZERO, Vector3.ZERO, Vector3.ZERO, Vector3.ZERO]
var accelerating: bool = false
var braking: bool = false
var steer_left: bool = false
var steer_right: bool = false
	
func applyWheelForces(wheel: Wheel, state: PhysicsDirectBodyState, totalForce: Vector3):
	# actually this is the base of the ray case, the point at which the spring connects to the car body
#	wheel.force_raycast_update()
	var wheelLocation = state.transform.basis.xform(wheel.get_wheel_body_space_location())
	
#	state.apply_impulse(wheelLocation, totalForce * state.step)
	state.add_force(totalForce, wheelLocation)
	
	
func _integrate_forces(state: PhysicsDirectBodyState):
	_steer_wheels(state)
	var wheelForces = []
	wheelForces.resize(4)
	
	for i in range(4):
		wheelForces[i] = calcTotalWheelForces(wheels[i], state)
	
	for i in range(4):
		applyWheelForces(wheels[i], state, wheelForces[i])
	
#	print(state.angular_velocity)

func _steer_wheels(state: PhysicsDirectBodyState):
	if steer_right:
		_rotate_front_wheels(state, -steering_speed_rad)
	
	elif steer_left:
		_rotate_front_wheels(state, steering_speed_rad)
	
func _rotate_front_wheels(state: PhysicsDirectBodyState, rotateSpeed: float):
	for i in range(2):
		var wheel = wheels[i]
		wheel.rotate(wheel.transform.basis.y, rotateSpeed * state.step)
	
func calcTotalWheelForces(wheel: Wheel, state: PhysicsDirectBodyState):
	var totalForce = calcRRForce(wheel, state) + \
		calcBrakingForce(wheel, state) + \
		wheel.getSpringForce() + \
		calcSteerForce(wheel, state)
#		calcEngineForce(wheel, state) + \
	
#	RWD
	match wheel.name:
		"RR", "RL":
			totalForce += calcEngineForce(wheel, state)
	
	return totalForce
	
func calcSteerForce(wheel: Wheel, state: PhysicsDirectBodyState):
	
	if !wheel.is_colliding():
		return Vector3.ZERO
	
	var ws_wheel_location = to_global(wheel.get_wheel_body_space_location())
	var velocity_at_wheel = get_point_velocity(ws_wheel_location)
	var ground_vel_at_wheel = velocity_at_wheel - velocity_at_wheel.project(Vector3.UP)
	
#	print(ground_vel_at_wheel)
	
#	var tire_forward_vel = ground_vel_at_wheel.project(wheel.transform.basis.z)
	var tire_side_vel = ground_vel_at_wheel.project(transform.basis.xform(wheel.transform.basis.x))
	
	var tire_mass = mass * 0.05
	
	var steer_force: Vector3 = -tire_side_vel * tire_mass / state.step
	
	match wheel.name:
		"RR", "RL":
			steer_force *= 0.6
	
	steer_force = steer_force.limit_length(max_steer_force)
	
	print(steer_force.length())
	
	return steer_force
	
func calcEngineForce(wheel: Wheel, state: PhysicsDirectBodyState):
	var forwardGroundDir = wheel.getProjectedOnGround(transform.basis.xform(wheel.transform.basis.z))
	var forwardGroundSpeed = wheel.getProjectedOnGround(state.linear_velocity)
	var currSpeed = forwardGroundSpeed.length()
	var engForce = Vector3.ZERO
	
	if accelerating && currSpeed < maxSpeed:
		var normalizedSpeed = currSpeed / maxSpeed
		var acceleration = engineForceCurve.interpolate(normalizedSpeed) * maxEngineForce * state.step
		engForce = forwardGroundDir * acceleration / (mass * 4)
		
	updateWheelForceDisplay(wheel.name, engineForce, engForce)
			
	return engForce
		
func calcRRForce(wheel: Wheel, state: PhysicsDirectBodyState):
	var forwardGroundSpeed = wheel.getProjectedOnGround(state.linear_velocity)
	var currSpeed = forwardGroundSpeed.length()
		
	if currSpeed > 0 && wheel.is_colliding():
		return -rr_coefficient * forwardGroundSpeed / state.step * (mass / 4) 
	else:
		return Vector3.ZERO
		
func calcBrakingForce(wheel: Wheel, state: PhysicsDirectBodyState):
	var forwardGroundSpeed = wheel.getProjectedOnGround(state.linear_velocity)
	var currSpeed = forwardGroundSpeed.length()
	var brakeForce = Vector3.ZERO
		
	if currSpeed > 0 && braking:
		brakeForce = - brake_coefficient * forwardGroundSpeed * (mass / 4)
	
	updateWheelForceDisplay(wheel.name, self.brakeForce, brakeForce)
		
	return brakeForce
	
## point must be in world space since car_body.transformation.origin is in world space
func get_point_velocity (point :Vector3)->Vector3:
	return linear_velocity + angular_velocity.cross(point - transform.origin)	
		
func updateWheelForceDisplay(wheelName: String, forceArr: Array, value: Vector3):
	
	match wheelName:
		"FR":
			forceArr[0] = value
		"RR":
			forceArr[1] = value
		"RL":
			forceArr[2] = value
		"FL":
			forceArr[3] = value
			

	

func _on_RayCast_update_offset(offset, force_mag, spring_force, dampening_force):
	emit_signal("update_offset", "FR", offset, force_mag, spring_force, dampening_force)


func _on_RayCast2_update_offset(offset, force_mag, spring_force, dampening_force):
	emit_signal("update_offset", "RR", offset, force_mag, spring_force, dampening_force)


func _on_RayCast3_update_offset(offset, force_mag, spring_force, dampening_force):
	emit_signal("update_offset", "RL", offset, force_mag, spring_force, dampening_force)


func _on_RayCast4_update_offset(offset, force_mag, spring_force, dampening_force):
	emit_signal("update_offset", "FL", offset, force_mag, spring_force, dampening_force)
