extends RigidBody

class_name Car

signal update_offset

export(Curve) var engineForceCurve: Curve

export(Curve) var front_grip_curve: Curve

export(Curve) var rear_grip_curve: Curve

export(int) var maxSpeed

export(int) var maxEngineForce

export(float) var rr_coefficient = 0.01

export(float) var brake_coefficient = 1

export(float) var steering_speed_deg = 20

export(float) var max_steering_angle_deg = 65

export(float) var max_steer_force = 30.0

export(float) var self_aligning_coefficient = 0.06

onready var wheels = [$FR, $FL, $RR, $RL]

onready var steering_speed_rad = deg2rad(steering_speed_deg)
onready var max_steering_angle_rad = deg2rad(max_steering_angle_deg)

var engineForce = [Vector3.ZERO, Vector3.ZERO, Vector3.ZERO, Vector3.ZERO]
var rrForce = [Vector3.ZERO, Vector3.ZERO, Vector3.ZERO, Vector3.ZERO]
var brakeForce = [Vector3.ZERO, Vector3.ZERO, Vector3.ZERO, Vector3.ZERO]
var wheelSteerAngle = [0, 0]
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
	var rotationAngle = rotateSpeed * state.step
	for i in range(2):
		var wheel = wheels[i]
		var new_rot = wheel.rotation.y + rotationAngle
		
		if new_rot > max_steering_angle_rad:
			rotationAngle -= new_rot - max_steering_angle_rad
		elif new_rot < -max_steering_angle_rad:
			rotationAngle -= new_rot + max_steering_angle_rad
		
		wheel.rotate_object_local(wheel.transform.basis.y, rotationAngle)
		update_wheel_steering_angle(wheel.name, rad2deg(wheel.rotation.y))
	
func calcTotalWheelForces(wheel: Wheel, state: PhysicsDirectBodyState):
	var totalForce = calcRRForce(wheel, state) + \
		calcBrakingForce(wheel, state) + \
		wheel.getSpringForce()

	var steerForce = calcSteerForce(wheel, state)
	
	totalForce += steerForce
#	totalForce += calcEngineForce(wheel, state)
	
#	RWD
	match wheel.name:
		"RR", "RL":
			totalForce += calcEngineForce(wheel, state)
#		"FL", "FR":
#			applySelfAligningForce(wheel, state, steerForce)
	
	return totalForce
	
func calcSteerForce(wheel: Wheel, state: PhysicsDirectBodyState):
	
	if !wheel.is_colliding():
		return Vector3.ZERO
	
	var ws_wheel_location = to_global(wheel.get_wheel_body_space_location())
	var velocity_at_wheel = get_point_velocity(ws_wheel_location)
	var ground_vel_at_wheel = velocity_at_wheel - velocity_at_wheel.project(Vector3.UP)
	
	if ground_vel_at_wheel.length() <= 0:
		return Vector3.ZERO
	
#	print(ground_vel_at_wheel)
	
#	var tire_forward_vel = ground_vel_at_wheel.project(wheel.transform.basis.z)
	var tire_side_vel = ground_vel_at_wheel.project(transform.basis.xform(wheel.transform.basis.x))
	
	var side_to_total_vel_ratio = tire_side_vel.length() / ground_vel_at_wheel.length()
	
	print("Side slip ration: %f" % side_to_total_vel_ratio)
	
	var grip_factor
	
	match wheel.name:
		"RR", "RL":
			grip_factor = rear_grip_curve.interpolate(side_to_total_vel_ratio)
		"FR", "FL":
			grip_factor = front_grip_curve.interpolate(side_to_total_vel_ratio)
	
#	grip_factor = 1
	
	var steer_force: Vector3 = -tire_side_vel * wheel.tyre_mass * grip_factor / state.step
	
	steer_force = steer_force.limit_length(max_steer_force)
	
	return steer_force
	
func applySelfAligningForce(wheel: Wheel, state: PhysicsDirectBodyState, steer_force: Vector3):
	var ws_wheel_side_dir = transform.basis.xform(wheel.transform.basis.x)
	var steer_force_scalar = steer_force.dot(ws_wheel_side_dir)
	
	if wheel.rotation.y != 0:
		var rot_accel = -self_aligning_coefficient * steer_force_scalar / wheel.calc_moment_of_inertia()
		wheel.rotate_object_local(wheel.transform.basis.y, rot_accel * state.step)
	
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
		
#	match wheel.name:
#		"FR", "FL":
#			brakeForce *= 0.3
#		"RR", "RL":
#			brakeForce *= 0.7
	
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
			
func update_wheel_steering_angle(wheelName: String, angle: float):
	
	match wheelName:
		"FL":
			wheelSteerAngle[0] = angle
		"FR":
			wheelSteerAngle[1] = angle
			
			

	

func _on_RayCast_update_offset(offset, force_mag, spring_force, dampening_force):
	emit_signal("update_offset", "FR", offset, force_mag, spring_force, dampening_force)


func _on_RayCast2_update_offset(offset, force_mag, spring_force, dampening_force):
	emit_signal("update_offset", "RR", offset, force_mag, spring_force, dampening_force)


func _on_RayCast3_update_offset(offset, force_mag, spring_force, dampening_force):
	emit_signal("update_offset", "RL", offset, force_mag, spring_force, dampening_force)


func _on_RayCast4_update_offset(offset, force_mag, spring_force, dampening_force):
	emit_signal("update_offset", "FL", offset, force_mag, spring_force, dampening_force)
