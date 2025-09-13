extends Node

const offset_str = "%s Offset: %.2f Distance: %.2f Top: %s Collision: %.2f %.2f"
const rpm_str = "RPM: %.0f"

var prev_RR_time = 0
var prev_FR_time = 0
var prev_self_align_time = 0

var slipRatios = [0.0, 0.0, 0.0, 0.0]

onready var cameras: Array = [$Car/CameraGimbal/Camera, $FollowCamera]
var camera_index = 1

func _ready():
	$Car.carEngine.connect("update_rpm", self, "_on_rpm_update")
	$Car.carEngine.connect("gear_shift", self, "_on_gear_shift")
	Logger.add_appender(FileAppender.new("user://logs/log.txt"))
	Logger.set_logger_format(Logger.LOG_FORMAT_MORE)

func _on_rpm_update(rpm: int):
	$UI/RPM.text = rpm_str % rpm
	
func _on_gear_shift(new_gear_index: int):
		$UI/Gear.text = "Gear: %d" % ($Car.carEngine.currentGear - 1)
	

func _on_Car_update_offset(wheel, offset, distance, topPos, colPos):
	match wheel:
		"FR":
			$UI/FR_offset.text = offset_str % ["FR", offset, distance, topPos, colPos.y, colPos.z]
			
#			print("Front: %f" % force_mag)
			
			var curr_msec = OS.get_ticks_msec()
			if curr_msec - prev_FR_time >= 200:
				#$UI/FR_Spring_Force.add_point(Vector2(curr_msec / 200.0 * 1024 / 30, 600 - force_mag / 5 * 20))
#				print($UI/FR_Spring_Force.position.x)
				prev_FR_time = curr_msec
		"RR":
			$UI/RR_offset.text = offset_str % ["RR", offset, distance, topPos, colPos.y, colPos.z]
			
#			print("Rear: %f\n" % force_mag)
			
			var curr_msec = OS.get_ticks_msec()
			if curr_msec - prev_RR_time >= 200:
				#$UI/RR_Spring_Force.add_point(Vector2(curr_msec / 200.0 * 1024 / 30, 600 - force_mag / 5 * 20))
#				print($UI/FR_Spring_Force.position.x)
				prev_RR_time = curr_msec
		"RL":
			$UI/RL_offset.text = offset_str % ["RL", offset, distance, topPos, colPos.y, colPos.z]
		"FL":
			$UI/FL_offset.text = offset_str % ["FL", offset, distance, topPos, colPos.y, colPos.z]
			
func _physics_process(_delta):
	$UI/Speed.text = "Speed: %7.5f" % ($Car.linear_velocity.length() * 3.6)
	$UI/Braking.text = "Braking: %s" % $Car.braking
	$UI/Engine.text = "Engine: %s %s %s %s" % $Car.engineForce
	$UI/Brake.text = "Brake: %s %s %s %s" % $Car.brakeForce
	$UI/RR_force.text = "RR force: %s %s %s %s" % $Car.rrForce
	$UI/SideSlip.text = "Side slip(FR, FL, RR, RL): %4.2f, %4.2f, %4.2f, %4.2f" % $Car.sideSlipRatio
	$"UI/Steering(L\\R)".text = "Steering(L\\R): %.2f, %.2f" % $Car.wheelSteerAngle
	$UI/Heading.text = "Heading: %.0f" % rad2deg($Car.global_rotation.y)
	$UI/GripFactor.text = "Grip factor: %3.2f %3.2f %3.2f %3.2f" % $Car.gripFactors
	
	var frontBackGripDiff = getFrontBackGripDiff()
	
	$UI/FrontBackGripDiff.text = "(F - R) Grip Diff(L, R): %3.2f, %3.2f" % [frontBackGripDiff.left, frontBackGripDiff.right]
	_control_car()
#	$UI/Self_Aligining.position.x = -curr_msec / 1000.0 * 1024 / 30

func getFrontBackGripDiff() -> Dictionary:
	var left = $Car.gripFactors[1] - $Car.gripFactors[3] 
	var right = $Car.gripFactors[0] - $Car.gripFactors[2]
	
	return {
		left = left,
		right = right
	}

func _control_car():
	$Car.accelerating = Input.is_action_pressed("ui_up")
	$Car.carEngine.accelerating = Input.is_action_pressed("ui_up")
	$Car.braking = Input.is_action_pressed("ui_down")
	$Car.steer_left = Input.is_action_pressed("ui_left")
	$Car.steer_right = Input.is_action_pressed("ui_right")
	
	if Input.is_action_just_pressed("reset_rotation"):
		$Car.reset_steering()
	
	if Input.is_action_just_pressed("jump"):
		$Car.is_jumping = true;
		
	if Input.is_action_just_pressed("gear_up"):
		$Car.carEngine.gear_up()
		
	if Input.is_action_just_pressed("gear_down"):
		$Car.carEngine.gear_down()
		
	($Car as Car).adjust_braking()
	
	if Input.is_action_just_pressed("reset"):
		$Car.reset()
		$UI/Gear.text = "Gear: %d" % ($Car.carEngine.currentGear - 1)
		
	if Input.is_action_just_pressed("look_right"):
		$Car/CameraGimbal.rotation.y = 0
		
	if Input.is_action_just_pressed("look_left"):
		$Car/CameraGimbal.rotation.y = -PI
		
	if Input.is_action_just_pressed("cycle_camera"):
		camera_index = (camera_index + 1) % cameras.size()
		(cameras[camera_index] as Camera).current = true
	
	if Input.is_action_just_pressed("look_back"):
		$FollowCamera.look_back = true
		
	if Input.is_action_just_released("look_back"):
		$FollowCamera.look_back = false

func _unhandled_input(event):
	if event.is_action_pressed("drop_ball"):
		var ball = preload("res://Ball.tscn").instance()
		ball.translation = $Car.translation
		ball.translation += Vector3.UP * 2.5
		ball.translation += $Car.global_transform.basis.z * 0.75
		ball.translation += $Car.global_transform.basis.x * 0.5
		add_child(ball)
	


func _on_Car_update_slip_ratio(wheelIndex: int, slipRatio: float):
	slipRatios[wheelIndex] = slipRatio
	$UI/SlipRatio.text = "Slip ratio: %3.2f %3.2f %3.2f %3.2f" % slipRatios


func _on_Car_update_angular_vel(wheels):
	$UI/AngualrVel.text = "Angular Vel: %3.2f %3.2f %3.2f %3.2f" % [wheels[0].angular_vel, wheels[1].angular_vel, wheels[2].angular_vel, wheels[3].angular_vel]
