extends RigidBody

class_name Car

signal update_offset

signal update_slip_ratio

signal update_angular_vel

export(Curve) var engineForceCurve: Curve

export(Curve) var front_grip_curve: Curve

export(Curve) var rear_grip_curve: Curve

export(NodePath) var carEngineNode: NodePath

export(int) var maxSpeed

export(int) var maxEngineForce

export(float) var tractionForceMag = 50.0

export(float) var rr_coefficient = 0.001

export(float) var brake_coefficient = 1.0

export(float) var steering_speed_deg = 20.0

export(float) var max_steering_angle_deg = 65.0

export(float) var max_steer_force = 30.0

# Calculated from 6 degree caster angle and 0.03 pneumatic trail
export(float) var self_aligning_coefficient = 0.06

enum Drivetrain {
	FWD,
	RWD,
	AWD
}

const drivenWheels = {
	Drivetrain.AWD: ["FR", "FL", "RR", "RL"],
	Drivetrain.RWD: ["RR", "RL"],
	Drivetrain.FWD: ["FR", "FL"],
}

export(Drivetrain) var drivetrain = Drivetrain.RWD
export(float) var min_vel_for_slip = 1.0

const long_pacejka = {
	"b": 0.8,
	"d": 6.0,
	"c": 3.0,
	"e": 1.0
}

const lat_pacejka = {
	"b": 10,
	"c": 1.9,
	"d": 1.0,
	"e": 0.97
}

onready var wheels = [$FR, $FL, $RR, $RL]
const wheelArrIndex = {
	"FR": 0,
	"FL": 1,
	"RR": 2,
	"RL": 3
}

onready var steering_speed_rad = deg2rad(steering_speed_deg)
onready var max_steering_angle_rad = deg2rad(max_steering_angle_deg)
onready var carEngine: CarEngine = get_node(carEngineNode) as CarEngine

var engineForce = [Vector3.ZERO, Vector3.ZERO, Vector3.ZERO, Vector3.ZERO]
var rrForce = [Vector3.ZERO, Vector3.ZERO, Vector3.ZERO, Vector3.ZERO]
var brakeForce = [Vector3.ZERO, Vector3.ZERO, Vector3.ZERO, Vector3.ZERO]
var totalForce = [Vector3.ZERO, Vector3.ZERO, Vector3.ZERO, Vector3.ZERO]
var gripFactors: Array = [0.0, 0.0, 0.0, 0.0]
var sideSlipRatio = [0, 0, 0, 0]
var wheelSteerAngle = [0, 0]
var accelerating: bool = false
var braking: bool = false
var steer_left: bool = false
var steer_right: bool = false
var is_jumping: bool = false
var brake_input: float = 0.0

func pacejka(x: float, b: float, c: float, d: float, e: float) -> float:
	var bx = b*x
	return d * sin(c * atan(bx - e*(bx - atan(bx))))
	
func applyWheelForces(wheel: Wheel, state: PhysicsDirectBodyState, totalForce: Vector3):
	# actually this is the base of the ray case, the point at which the spring connects to the car body
#	wheel.force_raycast_update()
	var wheelLocation = state.transform.basis.xform(wheel.get_wheel_body_space_location())
	
#	if (totalForce.z != 0.0):
#		print('Hi')
	
#	state.apply_impulse(wheelLocation, totalForce * state.step)
	state.add_force(totalForce, wheelLocation)
	
func reset_steering():
	for i in range(2):
		wheels[i].rotation.y = 0
	
func _get_avg_angular_vel_of_driven_wheels():
	var avgVel = 0
	
	for wheelName in drivenWheels[drivetrain]:
		var wheel:Wheel = wheels[wheelArrIndex[wheelName]]
		avgVel += wheel.get_rolling_rpm()
		
	return avgVel / float(drivenWheels[drivetrain].size())

func _integrate_forces(state: PhysicsDirectBodyState):
	_steer_wheels(state)
	var wheelForces = []
	wheelForces.resize(4)
	
	var grounded = true
	carEngine.adjust_throttle(state.step)
	carEngine.update_engine(_get_avg_angular_vel_of_driven_wheels(), state.step)
	
	for i in range(4):
		wheelForces[i] = calcTotalWheelForces(wheels[i], state)
		grounded = grounded && wheels[i].is_colliding()
		totalForce[i] = wheelForces[i]
		
	emit_signal("update_angular_vel", wheels)
	
	var frontRight = totalForce[0]
	var frontLeft = totalForce[1]
	var rearRight = totalForce[2]
	var rearLeft = totalForce[3]
		
	
	update_gizmos($FR, frontRight, wheels[0], 100)
	update_gizmos($FL, frontLeft, wheels[1], 100)
	update_gizmos($RR, rearRight, wheels[2], 100)
	update_gizmos($RL, rearLeft, wheels[3], 100)
	
#	update_gizmos($FR, get_point_velocity($FR.global_position), wheels[0], 1.0)
#	update_gizmos($FL, get_point_velocity($FL.global_position), wheels[1], 1.0)
#	update_gizmos($RR, get_point_velocity($RR.global_position), wheels[2], 1.0)
#	update_gizmos($RL, get_point_velocity($RL.global_position), wheels[3], 1.0)

	
	
	for i in range(4):
		applyWheelForces(wheels[i], state, wheelForces[i])
		
	if is_jumping:
		state.apply_central_impulse(Vector3.UP * 50)
		is_jumping = false
		
	
	
#	if grounded:
#		add_central_force((transform.basis.z - transform.basis.z.project(Vector3.UP)) * 2500 / mass)
		
#	print(linear_velocity.y)

func update_gizmos(wheelRayCast: Node, totalForce: Vector3, wheel: Spatial, scaleFactor: float = 25.0):
	
	if wheelRayCast.get_child_count() < 1:
		return
	
	var xScale = totalForce.dot(wheel.global_transform.basis.x)
	var yScale = totalForce.dot(wheel.global_transform.basis.y)
	var zScale = totalForce.dot(wheel.global_transform.basis.z)
	
	var xGizmo = wheelRayCast.get_node("Tyre/Gizmos/X_Anchor");
	var yGizmo = wheelRayCast.get_node("Tyre/Gizmos/Y_Anchor");
	var zGizmo = wheelRayCast.get_node("Tyre/Gizmos/Z_Anchor");
	
	xGizmo.scale.x = xScale / scaleFactor
	yGizmo.scale.x = yScale / scaleFactor
	zGizmo.scale.x = zScale / scaleFactor

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
	wheel.force_raycast_update()
	wheel.force_update_transform()
	var springForce = wheel.getSpringForce()
	var totalForce = Vector3.ZERO
	totalForce += springForce
	var mu = 1.0 # on Dry asphalt 0.9-1.2 according to chat GPT
	var max_trac_force = springForce.length() * mu

	var steerForce = calcSteerForce(wheel, state, max_trac_force)
	
	var tracForce: Vector3 = Vector3.ZERO
	var forwardGroundDir = wheel.getProjectedOnGround(transform.basis.xform(wheel.transform.basis.z))
	var slip_ratio: float = 0.0
	
	if drivenWheels[drivetrain].has(wheel.name):
		var tracForceMag = carEngine.get_torque_at_wheels() / wheel.tyre_radius
#
#		if(abs(tracForceMag) <= max_trac_force ||  (linear_velocity - linear_velocity.project(Vector3.UP)).length() < min_vel_for_slip):
#			tracForce = forwardGroundDir * clamp(tracForceMag, -max_trac_force, max_trac_force)
#
##			if wheel.name == 'RR':
##				print(tracForceMag)
#		else:
#			slip_ratio = wheel.calc_slip_ratio()
#			# Need to blend between undriven and driven force, specifically when clutch is not 100%
#			tracForce = pacejka(slip_ratio, long_pacejka.b, long_pacejka.c, springForce.length(), long_pacejka.e) * forwardGroundDir

		var forceAndSlip = wheel.getDrivenForce(tracForceMag, max_trac_force)
		tracForce = forwardGroundDir * forceAndSlip["long_force"]
		slip_ratio = forceAndSlip["slip_ratio"]
			
			
			
	else:
		var forceAndSlip = wheel.getUndrivenForce(max_trac_force)
		tracForce = forwardGroundDir * forceAndSlip["long_force"]
		slip_ratio = forceAndSlip["slip_ratio"]
#		pass
		
	var wheelIndex = wheelArrIndex[wheel.name]
	emit_signal("update_slip_ratio", wheelIndex, slip_ratio)
	
	var tyreForces: Vector3 = tracForce + steerForce
	tyreForces.limit_length(max_trac_force)
	
#	totalForce += tracForce
#	totalForce += steerForce

	steerForce = tyreForces.project(wheel.global_transform.basis.x)
	tracForce = tyreForces.project(wheel.global_transform.basis.z)

	totalForce += tyreForces
	
	
#	wheel.prev_forces.tracForce = tracForce
#	wheel.prev_forces.steerForce = steerForce
#	wheel.prev_forces.springForce = springForce
	
#	if totalForce.z != 0:
#		print("Why!?")

	match wheel.name:
		"FL", "FR":
#			applySelfAligningForce(wheel, state, steerForce)
			update_wheel_steering_angle(wheel.name, rad2deg(wheel.rotation.y))
			
	#if drivenWheels[drivetrain].has(wheel.name):
		#totalForce += calcEngineForce(wheel, state)
		
	if drivenWheels[drivetrain].has(wheel.name):
		wheel.updateAngualrVel(brake_input, state.step, carEngine.get_torque_at_wheels(), carEngine.get_rpm_at_wheels())
	else:
		wheel.updateAngularVelNonDriven(brake_input, state.step, tracForce.dot(forwardGroundDir))
	
	return totalForce
	
func calcSteerForce(wheel: Wheel, state: PhysicsDirectBodyState, max_steer_force: float):
	
	if !wheel.is_colliding():
		return Vector3.ZERO
	
	var ws_wheel_location = to_global(wheel.get_wheel_body_space_location())
	var velocity_at_wheel = get_point_velocity(ws_wheel_location)
	var ground_vel_at_wheel = velocity_at_wheel - velocity_at_wheel.project(Vector3.UP)
	
	if ground_vel_at_wheel.length() <= 0:
#	if velocity_at_wheel.length() <= 0:
		return Vector3.ZERO
	
#	var tire_forward_vel = ground_vel_at_wheel.project(wheel.transform.basis.z)
#	var tire_side_vel = ground_vel_at_wheel.project(wheel.global_transform.basis.x)
	# Removing any y component resulting from body roll
#	var pure_side_dir = wheel.global_transform.basis.x - wheel.global_transform.basis.x.project(Vector3.UP)
	var pure_side_dir = wheel.global_transform.basis.x
	var steering_vel = ground_vel_at_wheel.project(pure_side_dir)
	
	var steering_vel_scalar = ground_vel_at_wheel.dot(pure_side_dir)
	
	var side_to_total_vel_ratio = steering_vel.length() / ground_vel_at_wheel.length()
	
#	if wheel.name == 'FR':
#		print('Steer Vel: %3.2f' % steering_vel_scalar)
	
	
	var grip_factor
	
	match wheel.name:
		"RR", "RL":
			grip_factor = rear_grip_curve.interpolate(side_to_total_vel_ratio)
		"FR", "FL":
			grip_factor = front_grip_curve.interpolate(side_to_total_vel_ratio)
	
	sideSlipRatio[wheelArrIndex[wheel.name]] = side_to_total_vel_ratio
#	grip_factor = 1.0
	
	gripFactors[wheelArrIndex[wheel.name]] = grip_factor
	
	var steer_force: Vector3 = -steering_vel * mass/4.0 * grip_factor / state.step
	
	var size_before: float = steer_force.length()
	
#	steer_force = steer_force.limit_length(max_steer_force)
	
	var size_after = steer_force.length()
	
#	if wheel.name == "FR" && size_after > 0:
#		print('Before: %3.0f, After %3.0f, Truncated: %2.0f%%' % [size_before, size_after, (1 - size_after / size_before) * 100.0])
	
	return steer_force
	
func applySelfAligningForce(wheel: Wheel, state: PhysicsDirectBodyState, steer_force: Vector3):
	# Try using the wheel velocity instead of offset from center
	var ws_wheel_side_dir = transform.basis.xform(wheel.transform.basis.x)
	var steer_force_scalar = steer_force.dot(ws_wheel_side_dir)
	
#	if wheel.rotation.y != 0:
	var rot_accel = -self_aligning_coefficient * steer_force_scalar / wheel.calc_moment_of_inertia()
	var rot_change = rot_accel * pow(state.step, 2)
	var curr_rot = wheel.rotation.y
	rot_change = clamp(rot_change, -max_steering_angle_rad + curr_rot, max_steering_angle_rad - curr_rot)
		
	wheel.rotate_object_local(wheel.transform.basis.y, rot_change)
	
func calcEngineForce(wheel: Wheel, state: PhysicsDirectBodyState):
	var forwardGroundDir = wheel.getProjectedOnGround(transform.basis.xform(wheel.transform.basis.z))
	var forwardGroundSpeed = wheel.getProjectedOnGround(state.linear_velocity)
	var currSpeed = forwardGroundSpeed.length()
	var engForce = Vector3.ZERO
	
	if accelerating && currSpeed < maxSpeed:
		var normalizedSpeed = currSpeed / maxSpeed
		var acceleration = engineForceCurve.interpolate(normalizedSpeed) * maxEngineForce
		engForce = forwardGroundDir * acceleration / (mass * 4)
	
	updateWheelForceDisplay(wheel.name, engineForce, engForce)
			
	return engForce
		
func calcRRForce(wheel: Wheel, state: PhysicsDirectBodyState):
	var forwardGroundSpeed = wheel.getProjectedOnGround(state.linear_velocity)
	var currSpeed = forwardGroundSpeed.length()
	var retVal
		
	if currSpeed > 0 && wheel.is_colliding():
		retVal = -rr_coefficient * forwardGroundSpeed / state.step * (mass / 4) 
	else:
		retVal = Vector3.ZERO
	
	#updateWheelForceDisplay(wheel.name, self.rrForce, retVal)
	
	return retVal
		
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
	
func adjust_braking():
	if braking:
		brake_input = min(brake_input + 0.1, 1)
	else:
		brake_input = max(brake_input - 0.05, 0)
		
#	carEngine.clutch = 1 - brake_input
	
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
			
func reset():
	global_transform.origin = Vector3(0, 2.305, -463.153)
	carEngine.currentGear = 1
	carEngine.currentRpm = 1000
	linear_velocity = Vector3.ZERO
	angular_velocity = Vector3.ZERO
	rotation = Vector3.ZERO
	
	for wheel in wheels:
		wheel.angular_vel = 0.0
	
	
	

func _on_RayCast_update_offset(offset, distance, topPos, colPos):
	emit_signal("update_offset", "FR", offset, distance, topPos, colPos)


func _on_RayCast2_update_offset(offset, distance, topPos, colPos):
	emit_signal("update_offset", "RR", offset, distance, topPos, colPos)


func _on_RayCast3_update_offset(offset, distance, topPos, colPos):
	emit_signal("update_offset", "RL", offset, distance, topPos, colPos)


func _on_RayCast4_update_offset(offset, distance, topPos, colPos):
	emit_signal("update_offset", "FL", offset, distance, topPos, colPos)
