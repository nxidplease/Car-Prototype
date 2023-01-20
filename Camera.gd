extends Camera
export(NodePath) var car_path
export(float) var up_offset = 1.5
export(float) var back_offset = 3.5

onready var car: RigidBody = get_node(car_path)

func _ready():
	if !current:
		set_process(false)

func _process(_delta):
#	project car facing onto ground and normalize(important if car is tilting back/forward
	var camera_back_dir = (car.transform.basis.z - car.transform.basis.z.dot(Vector3.UP) * Vector3.UP).normalized()
	
	var velocity_to_ground_proj = car.linear_velocity - car.linear_velocity.dot(Vector3.UP) * Vector3.UP

	if velocity_to_ground_proj.length() > 0.5:
		camera_back_dir = velocity_to_ground_proj.normalized()
	
#	var car_forward_edge = car.translation + car.transform.basis.z
	look_at_from_position(car.translation - camera_back_dir * back_offset + Vector3.UP * up_offset, car.translation, Vector3.UP)
