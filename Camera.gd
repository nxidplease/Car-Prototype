extends Camera

const max_speed = 3500

func _process(delta):
	var car_pos = get_node("../Car").translation
	look_at(car_pos, Vector3.UP)
	
	var dist_to_car = car_pos.distance_to(translation)
	var speed = 0
	
	if dist_to_car > 500:
		speed = max_speed
	elif dist_to_car <= 500 and dist_to_car > 5:
		speed = lerp(0, max_speed, (dist_to_car - 5) / 495)
	
	translation -= global_transform.basis.z * delta * speed
