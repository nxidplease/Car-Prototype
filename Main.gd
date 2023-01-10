extends Node

const offset_str = "%s Offset: %f Force: %f Spring: %f Damp: %f"


func _on_Car_update_offset(wheel, offset, force_mag, spring_force, dampening_force):
	match wheel:
		"FR":
			$UI/FR_offset.text = offset_str % ["FR", offset, force_mag, spring_force, dampening_force]
		"RR":
			$UI/RR_offset.text = offset_str % ["RR", offset, force_mag, spring_force, dampening_force]
		"RL":
			$UI/RL_offset.text = offset_str % ["RL", offset, force_mag, spring_force, dampening_force]
		"FL":
			$UI/FL_offset.text = offset_str % ["FL", offset, force_mag, spring_force, dampening_force]

func _unhandled_input(event):
	if event.is_action_pressed("ui_accept"):
		var ball = preload("res://Ball.tscn").instance()
		ball.translation = $Car.translation
		ball.translation += Vector3.UP * 2.5
		ball.translation += $Car.global_transform.basis.z * 0.75
		ball.translation += $Car.global_transform.basis.x * 0.5
		add_child(ball)
	
