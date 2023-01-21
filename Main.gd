extends Node

const offset_str = "%s Offset: %f Force: %f Spring: %f Damp: %f"

var prev_RR_time = 0
var prev_FR_time = 0
var prev_self_align_time = 0

func _on_Car_update_offset(wheel, offset, force_mag, spring_force, dampening_force):
	match wheel:
		"FR":
			$UI/FR_offset.text = offset_str % ["FR", offset, force_mag, spring_force, dampening_force]
			
#			print("Front: %f" % force_mag)
			
			var curr_msec = OS.get_ticks_msec()
			if curr_msec - prev_FR_time >= 200:
				$UI/FR_Spring_Force.add_point(Vector2(curr_msec / 200.0 * 1024 / 30, 600 - spring_force / 5 * 20))
#				print($UI/FR_Spring_Force.position.x)
				prev_FR_time = curr_msec
		"RR":
			$UI/RR_offset.text = offset_str % ["RR", offset, force_mag, spring_force, dampening_force]
			
#			print("Rear: %f\n" % force_mag)
			
			var curr_msec = OS.get_ticks_msec()
			if curr_msec - prev_RR_time >= 200:
				$UI/RR_Spring_Force.add_point(Vector2(curr_msec / 200.0 * 1024 / 30, 600 - spring_force / 5 * 20))
#				print($UI/FR_Spring_Force.position.x)
				prev_RR_time = curr_msec
		"RL":
			$UI/RL_offset.text = offset_str % ["RL", offset, force_mag, spring_force, dampening_force]
		"FL":
			$UI/FL_offset.text = offset_str % ["FL", offset, force_mag, spring_force, dampening_force]
			
func _physics_process(_delta):
	$UI/Speed.text = "Speed: %f" % $Car.linear_velocity.length()
	$UI/Braking.text = "Braking: %s" % $Car.braking
	$UI/Engine.text = "Engine: %s %s %s %s" % $Car.engineForce
	$UI/Brake.text = "Brake: %s %s %s %s" % $Car.brakeForce
	$UI/RR_force.text = "RR force: %s %s %s %s" % $Car.rrForce
	$UI/SideSlip.text = "Side slip(FR, FL, RR, RL): %.2f, %.2f, %.2f, %.2f" % $Car.sideSlipRatio
	$"UI/Steering(L\\R)".text = "Steering(L\\R): %f, %f" % $Car.wheelSteerAngle
	$UI/Heading.text = "Heading: %f" % rad2deg($Car.global_rotation.y)
	_control_car()
	
	var curr_msec = OS.get_ticks_msec()
#	if curr_msec - prev_self_align_time >= 200:
#		$UI/Self_Aligining.add_point(Vector2(curr_msec / 200.0 * 1024 / 30, $Car/FL.rotation_degrees.y * 10 + 270))
##				print($UI/FR_Spring_Force.position.x)
#		prev_self_align_time = curr_msec
	
	$UI/FR_Spring_Force.position.x = -curr_msec / 1000.0 * 1024 / 30
	$UI/RR_Spring_Force.position.x = -curr_msec / 1000.0 * 1024 / 30
#	$UI/Self_Aligining.position.x = -curr_msec / 1000.0 * 1024 / 30
	
func _control_car():
	$Car.accelerating = Input.is_action_pressed("ui_up")
	$Car.braking = Input.is_action_pressed("ui_down")
	$Car.steer_left = Input.is_action_pressed("ui_left")
	$Car.steer_right = Input.is_action_pressed("ui_right")
	
	if Input.is_action_just_pressed("reset_rotation"):
		$Car.reset_steering()
	

func _unhandled_input(event):
	if event.is_action_pressed("ui_accept"):
		var ball = preload("res://Ball.tscn").instance()
		ball.translation = $Car.translation
		ball.translation += Vector3.UP * 2.5
		ball.translation += $Car.global_transform.basis.z * 0.75
		ball.translation += $Car.global_transform.basis.x * 0.5
		add_child(ball)
	
