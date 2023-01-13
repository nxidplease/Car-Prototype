extends RigidBody

signal update_offset

export(Curve) var engineForceCurve: Curve

export(int) var maxSpeed

export(int) var maxEngineForce

export(float) var rr_coefficient = 0.01

export(float) var brake_coefficient = 1

var engineForce = Vector3.ZERO
var rrForce = Vector3.ZERO
var brakeForce = Vector3.ZERO
var accelerating: bool = false
var braking: bool = false
	
func applyWheelForces(wheel: Wheel, state: PhysicsDirectBodyState):
	var transformedWheelPos = state.transform.basis.xform(wheel.translation)
	state.add_force(wheel.getSpringForce(), transformedWheelPos)
	state.add_force(calcEngineForce(wheel, state), transformedWheelPos)
	state.add_force(calcRRForce(wheel, state), transformedWheelPos)
	state.add_force(brakeForce, transformedWheelPos)
	
func _integrate_forces(state: PhysicsDirectBodyState):
	applyWheelForces($FR, state)
	applyWheelForces($RR, state)
	applyWheelForces($RL, state)
	applyWheelForces($FL, state)
	var forwardLinearVel = state.transform.basis.z.dot(state.linear_velocity) * state.transform.basis.z
	var groundVel
	var currSpeed = forwardLinearVel.length()
		
	if currSpeed > 0:
		rrForce = -rr_coefficient * forwardLinearVel / state.step * (mass / 4) 
		
		if braking:
			brakeForce = - brake_coefficient * forwardLinearVel / state.step * (mass / 4)
			
	if !braking:
		brakeForce = Vector3.ZERO
	
func calcEngineForce(wheel: Wheel, state: PhysicsDirectBodyState):
	var forwardGroundDir = wheel.getProjectedOnGround(state.transform.basis.z)
	var forwardLinearVel = state.transform.basis.z.dot(state.linear_velocity) * state.transform.basis.z
	var forwardGroundSpeed = wheel.getProjectedOnGround(forwardLinearVel)
	var currSpeed = forwardGroundSpeed.length()
	
	if accelerating && currSpeed < maxSpeed:
		var normalizedSpeed = currSpeed / maxSpeed
		var acceleration = engineForceCurve.interpolate(normalizedSpeed) * maxEngineForce * state.step
		return forwardGroundDir * acceleration / (mass * 4)
			
	else:
		return Vector3.ZERO
		
func calcRRForce(wheel: Wheel, state: PhysicsDirectBodyState):
	var forwardGroundDir = wheel.getProjectedOnGround(state.transform.basis.z)
	var forwardLinearVel = state.transform.basis.z.dot(state.linear_velocity) * state.transform.basis.z
	var forwardGroundSpeed = wheel.getProjectedOnGround(forwardLinearVel)
	var currSpeed = forwardGroundSpeed.length()
		
	if currSpeed > 0 && wheel.is_colliding():
		return -rr_coefficient * forwardGroundSpeed / state.step * (mass / 4) 
	else:
		return Vector3.ZERO
		
func calcBrakingForce(wheel: Wheel, state: PhysicsDirectBodyState):
	var forwardGroundDir = wheel.getProjectedOnGround(state.transform.basis.z)
	var forwardLinearVel = state.transform.basis.z.dot(state.linear_velocity) * state.transform.basis.z
	var forwardGroundSpeed = wheel.getProjectedOnGround(forwardLinearVel)
	var currSpeed = forwardGroundSpeed.length()
		
	if currSpeed > 0 && braking:
		brakeForce = - brake_coefficient * forwardGroundSpeed / state.step * (mass / 4)
			
	elif !braking:
		brakeForce = Vector3.ZERO

func _on_RayCast_update_offset(offset, force_mag, spring_force, dampening_force):
	emit_signal("update_offset", "FR", offset, force_mag, spring_force, dampening_force)


func _on_RayCast2_update_offset(offset, force_mag, spring_force, dampening_force):
	emit_signal("update_offset", "RR", offset, force_mag, spring_force, dampening_force)


func _on_RayCast3_update_offset(offset, force_mag, spring_force, dampening_force):
	emit_signal("update_offset", "RL", offset, force_mag, spring_force, dampening_force)


func _on_RayCast4_update_offset(offset, force_mag, spring_force, dampening_force):
	emit_signal("update_offset", "FL", offset, force_mag, spring_force, dampening_force)
