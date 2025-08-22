extends RayCast

export(float) var spring_strength = 2.0
export var default_dist_from_groud = 0.25
export var damping_strength = 0.5
export(NodePath) var car_body_path
export var tyre_radius = 0.2

var curr_offset = 0
var spring_force = 0
var dampening_force = 0
var angular_vel = 0 # radians per sec
var prev_forces: Dictionary = {
	tracForce = Vector3.ZERO,
	steerForce = Vector3.ZERO,
	springForce = Vector3.ZERO
}

signal update_offset

onready var car_body: Spatial = get_node(car_body_path)

# Virtual tire mass
onready var tyre_mass: float = 18.8
#onready var tyre_mass: float = car_body.mass / 4

class_name Wheel

func _ready():
	cast_to = Vector3.DOWN * (default_dist_from_groud + tyre_radius)

func calc_spring_force_for_wheel(collisionPoint: Vector3):
	var localCollisionPoint = car_body.to_local(collisionPoint)
	var distance = translation.distance_to(localCollisionPoint)
	curr_offset = default_dist_from_groud - distance
	
#	print("%f, %f" % [get_point_velocity(localCollisionPoint).length(), transform.basis.y.dot(get_point_velocity(localCollisionPoint))])
	
	var point_vel = car_body.get_point_velocity(collisionPoint)
#	print(point_vel)
	var velocity_to_offset = transform.basis.y.dot(point_vel)
#	var velocity_to_offset = get_collision_normal().dot(point_vel)
	
	dampening_force = -velocity_to_offset * damping_strength
	spring_force = spring_strength * curr_offset
	
	if curr_offset < 0:
		return 0
		
	var force_mag = spring_force + dampening_force
	#var force_mag = spring_force
	
	emit_signal("update_offset", curr_offset, distance, translation, localCollisionPoint)
	
	return force_mag
#	return spring_force
	
func get_spring_force_at_wheel(collisionPoint: Vector3):
	var force_mag = calc_spring_force_for_wheel(collisionPoint)
	
	# Consider using collision normal instead of local UP vector
#	return force_mag * global_transform.basis.y
	return force_mag * get_collision_normal()
	
func getSpringForce():
	if is_colliding():
		var collisionPoint = get_collision_point()
#		print(to_local(collisionPoint))
#		$Tyre.translation = to_local(collisionPoint) + transform.basis.y*tyre_radius
		return get_spring_force_at_wheel(collisionPoint)
	
	else:
#		$Tyre.translation = -transform.basis.y * default_dist_from_groud
		return Vector3.ZERO
		
func updateAngualrVel(brake_input: float, dt: float, engine_torque: float, expected_rpm: float):
#	if (expected_rpm > 0):
#		print('WOW')
	var expected_angular_vel = (expected_rpm / 60) * 2 * PI;
	var drive_change = (engine_torque / 500) * (expected_angular_vel - angular_vel)
#	var drive_change = 5 * (expected_angular_vel - angular_vel)
#	var drive_change = engine_torque * (expected_angular_vel - angular_vel)
	_updateAngularVel(drive_change, brake_input, dt);
	
func updateAngularVelNonDriven(brake_input: float, dt: float, long_force: float):
	var long_torque = -long_force * tyre_radius
	var long_drive = long_torque / calc_moment_of_inertia()
	
#	print('Slip drive change: %.2f' % long_drive)
	
	_updateAngularVel(long_drive, brake_input, dt)
	
func getDrivenForce(tracForceMag: float, max_trac_force: float) -> Dictionary:
	var slip_ratio: float = 0.0
	var long_force: float = 0.0
	var velocity_in_rolling_dir = get_velocity_in_rolling_dir()
		
	if(abs(tracForceMag) <= max_trac_force ||  velocity_in_rolling_dir.length() < 1.0):
		long_force = clamp(tracForceMag, -max_trac_force, max_trac_force)
	else:
		var long_pacejka = car_body.long_pacejka
		slip_ratio = calc_slip_ratio()
		long_force = car_body.pacejka(slip_ratio, long_pacejka.b, long_pacejka.c, max_trac_force, long_pacejka.e)
		
	return {
		"long_force": long_force,
		"slip_ratio": slip_ratio
	}
	
func getUndrivenForce(max_friction: float) -> Dictionary:
	var long_force: float
	var slip_ratio: float = 0.0
	var velocity_in_rolling_dir = get_velocity_in_rolling_dir()
	
	if velocity_in_rolling_dir.length() < 1.0:
		var desired = velocity_in_rolling_dir.dot(global_transform.basis.z)
		var actual = angular_vel * tyre_radius
		# This is the force that will act on the car, so it should try to sync the car's speed to the wheel's speed
		var desired_force = (actual - desired) * 0.5
		
#		if name == 'FR':
#			print('Car: %.2f Wheel: %.2f Diff(Wheel - Car): %.2f' % [desired, actual, actual - desired])
		
		# THe problem is herreeeeeee when decellerating
		long_force = desired_force
	else:		
		slip_ratio = calc_slip_ratio()
		var long_pacejka = car_body.long_pacejka
		long_force = car_body.pacejka(slip_ratio, long_pacejka.b, long_pacejka.c, max_friction, long_pacejka.e)
	
	long_force = clamp(long_force, -max_friction, max_friction)
		
	return {
		"long_force": long_force,
		"slip_ratio": slip_ratio
	}
	
	
func _updateAngularVel(drive_change: float, brake_input: float, dt: float):
	var angularVelChange = drive_change \
		- sign(angular_vel) * car_body.brake_coefficient * brake_input \
		- car_body.rr_coefficient * angular_vel
#		- car_body.brake_coefficient * brake_input \
	angular_vel += angularVelChange / calc_moment_of_inertia() * dt
	
#	if name == 'RR':
#		print("%.0f %.3f %.3f" % [drive_change, angularVelChange * dt, angular_vel])
		
func get_wheel_body_space_location() -> Vector3:
	return transform.xform($Tyre.translation)
	
func _process(_delta):
	#force_raycast_update()
	#force_update_transform()
	if is_colliding():
		$Tyre.translation = to_local(get_collision_point()) + transform.basis.y*tyre_radius
	else:
		$Tyre.translation = -transform.basis.y * default_dist_from_groud
	
		
func getProjectedOnGround(direction: Vector3):
	if is_colliding():
		var collision_normal = get_collision_normal()
		return direction - direction.project(collision_normal)
	else:
		return Vector3.ZERO


## (m * r^2) / 2
func calc_moment_of_inertia() -> float:
#	var cylinder_mesh := $Tyre.mesh as CylinderMesh
#	return (pow(cylinder_mesh.height, 2) + 3 * pow(tyre_radius, 2)) * tyre_mass / 12
	return (tyre_mass * pow(tyre_radius, 2)) / 2
	
func get_velocity_in_rolling_dir() -> Vector3:
	var global_velocity_at_wheel = car_body.get_point_velocity(to_global(translation))
	return global_velocity_at_wheel.project(global_transform.basis.z)
	
	
func get_rolling_rpm() -> float:
	var velocity_in_rolling_dir = get_velocity_in_rolling_dir()
	
	return (velocity_in_rolling_dir.dot(global_transform.basis.z) / (2 * PI * tyre_radius)) * 60
#	return velocity_in_rolling_dir.dot(global_transform.basis.z) / 2 * PI * tyre_radius

func calc_slip_ratio() -> float:
#	var rolling_speed = (rolling_rpm / 60) * 2 * PI * tyre_radius
	var velocity_in_rolling_dir: Vector3 = get_velocity_in_rolling_dir();
	var contact_vel: float = angular_vel * tyre_radius
	var rolling_vel: float = velocity_in_rolling_dir.dot(global_transform.basis.z)
	
	# Stabilizer to avoid infinite slip ratio
#	var c: float = 1.0;
	
#	if name == 'FR':
#		print("%.2f  %.2f" % [contact_vel, rolling_vel])
	
	var slipRatio = 0.0
	
	if is_colliding():
		slipRatio = contact_vel / abs(rolling_vel) - sign(rolling_vel)
	
	return slipRatio
