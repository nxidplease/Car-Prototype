extends Node

const offset_str = "%s Offset: %f Force: %f Spring: %f Damp: %f"

var prev_time = 0

func _on_Car_update_offset(wheel, offset, force_mag, spring_force, dampening_force):
	match wheel:
		"FR":
			$UI/FR_offset.text = offset_str % ["FR", offset, force_mag, spring_force, dampening_force]
			
			var curr_msec = OS.get_ticks_msec()
			if curr_msec - prev_time >= 200:
				$UI/FR_Spring_Force.add_point(Vector2(OS.get_ticks_msec() / 200 * 1024 / 30, 600 - spring_force / 5 * 20))
				print($UI/FR_Spring_Force.get_point_count())
				prev_time = curr_msec
		"RR":
			$UI/RR_offset.text = offset_str % ["RR", offset, force_mag, spring_force, dampening_force]
		"RL":
			$UI/RL_offset.text = offset_str % ["RL", offset, force_mag, spring_force, dampening_force]
		"FL":
			$UI/FL_offset.text = offset_str % ["FL", offset, force_mag, spring_force, dampening_force]
			
func _physics_process(_delta):
	$UI/Speed.text = "Speed: %f" % $Car.linear_velocity.length()
	$UI/Braking.text = "Braking: %s" % $Car.braking
	$UI/Engine.text = "Engine: %s" % $Car.engineForce
	$Car.accelerating = Input.is_action_pressed("ui_up")
	$Car.braking = Input.is_action_pressed("ui_down")

func _unhandled_input(event):
	if event.is_action_pressed("ui_accept"):
		var ball = preload("res://Ball.tscn").instance()
		ball.translation = $Car.translation
		ball.translation += Vector3.UP * 2.5
		ball.translation += $Car.global_transform.basis.z * 0.75
		ball.translation += $Car.global_transform.basis.x * 0.5
		add_child(ball)
	
