extends Spatial

# This script should be attached to a wheel mesh which is a direct child of a RayCast with a Wheel.gd Script

var parentWheel: Wheel

# Called when the node enters the scene tree for the first time.
func _ready():
	parentWheel = get_parent().get_parent()


# Called every frame. 'delta' is the elapsed time since the previous frame.
func _process(delta):
	rotate_x(parentWheel.angular_vel * delta)
