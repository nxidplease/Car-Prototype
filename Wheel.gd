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

# Virtual tire mass
onready var tyre_mass: float = car_body.mass / 4

class_name Wheel

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
	
	return spring_force + dampening_force
#	return spring_force
	
func get_spring_force_at_wheel(collisionPoint: Vector3):
	var force_mag = calc_spring_force_for_wheel(collisionPoint)
	
	
	emit_signal("update_offset", curr_offset, force_mag, spring_force, dampening_force)
	
	# Consider using collision normal instead of local UP vector
	return force_mag * global_transform.basis.y
#	return force_mag * get_collision_normal()
	
func getSpringForce():
	if is_colliding():
		var collisionPoint = get_collision_point()
#		print(to_local(collisionPoint))
#		$Tyre.translation = to_local(collisionPoint) + transform.basis.y*tyre_radius
		return get_spring_force_at_wheel(collisionPoint)
	
	else:
#		$Tyre.translation = -transform.basis.y * default_dist_from_groud
		return Vector3.ZERO
		
func get_wheel_body_space_location() -> Vector3:
	return transform * $Tyre.translation
	
func _process(_delta):
	force_raycast_update()
	force_update_transform()
	if is_colliding():
		$Tyre.translation = to_local(get_collision_point()) + transform.basis.y*tyre_radius
	else:
		$Tyre.translation = -transform.basis.y * default_dist_from_groud
	
		
func getProjectedOnGround(direction: Vector3):
	if is_colliding():
		var collision_normal = get_collision_normal()
		return direction - direction.dot(collision_normal) * collision_normal
	else:
		return Vector3.ZERO


## m(h^2 + 3r^2)/12
func calc_moment_of_inertia() -> float:
	var cylinder_mesh := $Tyre.mesh as CylinderMesh
	return (pow(cylinder_mesh.height, 2) + 3 * pow(tyre_radius, 2)) * tyre_mass / 12
