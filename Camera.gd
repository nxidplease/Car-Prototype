extends Camera
export(NodePath) var car_path
export(float) var up_offset = 1.5
export(float) var back_offset = 3.5
export(float, 0.2, 2.0, 0.1) var look_ahead_dist = 0.5
export(float, 0.1, 50.0, 0.1) var spring_strength = 1.0
export(float, 10.0, 100.0, 5.0) var dampening = 70.0
#export(float) var max_distance = 4
#export(float) var min_distance = 2
#export var height = 1.5

onready var car: RigidBody = get_node(car_path)

var look_back: bool = false
var last_pos: Vector3

func _ready():
	global_position = _get_target_origin()
	last_pos = global_position
	if !current:
		set_process(false)

#func cool_camera():
#	var target = car.get_global_transform().origin
#	var pos = get_global_transform().origin
#
#	var from_target = pos - target
#
#	# Check ranges.
#	if from_target.length() < min_distance:
#		from_target = from_target.normalized() * min_distance
#	elif from_target.length() > max_distance:
#		from_target = from_target.normalized() * max_distance
#
#	from_target.y = height
#
#	pos = target + from_target
#
#	look_at_from_position(pos, target, Vector3.UP)
	

func _process(delta):

	var velocity_to_ground_proj = car.linear_velocity - car.linear_velocity.project(Vector3.UP)

#	print(velocity_to_ground_proj.length())
	# Look ahead of the car
	var camera_look_target = car.global_position + car.global_transform.basis.z * look_ahead_dist

	if velocity_to_ground_proj.length() > 1.5:
		camera_look_target = car.global_position + velocity_to_ground_proj.normalized() * look_ahead_dist
			
	
	var new_target_origin = _get_target_origin()
	
	var accel_dir = new_target_origin - global_position
	
	var last_vel = (global_position - last_pos) / delta
	
	var spring_accel = accel_dir * spring_strength * delta
	

#	var car_forward_edge = car.translation + car.transform.basis.z
	look_at_from_position(global_position + spring_accel, camera_look_target, Vector3.UP)
	
	last_pos = global_position
	
func _get_target_origin():
#	project car facing onto ground and normalize(important if car is tilting back/forward
	var camera_back_dir = (car.transform.basis.z - car.transform.basis.z.project(Vector3.UP)).normalized()

	var velocity_to_ground_proj = car.linear_velocity - car.linear_velocity.project(Vector3.UP)

	if velocity_to_ground_proj.length() > 1.5:
		camera_back_dir = velocity_to_ground_proj.normalized()

		if look_back:
			camera_back_dir *= -1
			
	
	return car.translation - camera_back_dir * back_offset + Vector3.UP * up_offset
	
