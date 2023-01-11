extends RayCast

export(float) var spring_strength = 2
export var default_dist_from_groud = 0.25
export var damping_strength = 0.5
export(NodePath) var car_body_path
export var tyre_radius = 0.2

var curr_offset = 0
var spring_force = 0
var dampening_force = 0

signal update_offset

onready var car_body: RigidBody = get_node(car_body_path)

func calc_spring_force_for_wheel(collisionPoint: Vector3):
	var localCollisionPoint = car_body.to_local(collisionPoint)
	$Tyre.translation = to_local(collisionPoint) + transform.basis.y*tyre_radius
	var distance = translation.distance_to(localCollisionPoint)
	curr_offset = default_dist_from_groud - distance
	
	var velocity_to_offset_func = global_transform.basis.y.dot(get_point_velocity(localCollisionPoint))
	
	dampening_force = -velocity_to_offset_func * damping_strength
	spring_force = spring_strength * curr_offset
	
	if curr_offset < 0:
		return 0
		
	
	return spring_force + dampening_force
	
func get_point_velocity (point :Vector3)->Vector3:
	return car_body.linear_velocity + car_body.angular_velocity.cross(point - car_body.transform.origin)	
	
func get_spring_force_at_wheel(collisionPoint: Vector3):
	var force_mag = calc_spring_force_for_wheel(collisionPoint)
	
	
	emit_signal("update_offset", curr_offset, force_mag, spring_force, dampening_force)
	
	# Consider using collision normal instead of local UP vector
#	return force_mag * global_transform.basis.y
	return force_mag * get_collision_normal()
	
func getSpringForce():
	if is_colliding():
		return get_spring_force_at_wheel(get_collision_point())
	
	else:
		return Vector3.ZERO
