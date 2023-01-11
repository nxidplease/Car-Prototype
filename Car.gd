extends RigidBody

signal update_offset

func applySpringImpulseOnWheel(wheel: RayCast, state):
	state.add_force(wheel.getSpringForce(), state.transform.basis.xform(wheel.translation))
#	state.add_force(wheel.getSpringForce(), wheel.translation)
	
func _integrate_forces(state):
	applySpringImpulseOnWheel($FR, state)
	applySpringImpulseOnWheel($RR, state)
	applySpringImpulseOnWheel($RL, state)
	applySpringImpulseOnWheel($FL, state)


func _on_RayCast_update_offset(offset, force_mag, spring_force, dampening_force):
	emit_signal("update_offset", "FR", offset, force_mag, spring_force, dampening_force)


func _on_RayCast2_update_offset(offset, force_mag, spring_force, dampening_force):
	emit_signal("update_offset", "RR", offset, force_mag, spring_force, dampening_force)


func _on_RayCast3_update_offset(offset, force_mag, spring_force, dampening_force):
	emit_signal("update_offset", "RL", offset, force_mag, spring_force, dampening_force)


func _on_RayCast4_update_offset(offset, force_mag, spring_force, dampening_force):
	emit_signal("update_offset", "FL", offset, force_mag, spring_force, dampening_force)
