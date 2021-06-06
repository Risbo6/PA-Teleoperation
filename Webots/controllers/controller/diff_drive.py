#https://www.instructables.com/Joystick-to-Differential-Drive-Python/
import math 
def joystickToDiff(x, y, minJoystick, maxJoystick, minSpeed, maxSpeed):	# If x and y are 0, then there is not much to calculate...
	if x == 0 and y == 0:
		return (0, 0)
	z = math.sqrt(x * x + y * y)
	rad = math.acos(math.fabs(x) / z)
	angle = rad * 180 / math.pi

	tcoeff = -1 + (angle / 90) * 2
	turn = tcoeff * math.fabs(math.fabs(y) - math.fabs(x))
	turn = round(turn * 100, 0) / 100

	mov = max(math.fabs(y), math.fabs(x))

	if (x >= 0 and y >= 0) or (x < 0 and y < 0):
		rawLeft = mov
		rawRight = turn
	else:
		rawRight = mov
		rawLeft = turn

	if y < 0:
		rawLeft = 0 - rawLeft
		rawRight = 0 - rawRight

	rightOut = map(rawRight, minJoystick, maxJoystick, minSpeed, maxSpeed)
	leftOut = map(rawLeft, minJoystick, maxJoystick, minSpeed, maxSpeed)

	return (rightOut, leftOut)

def map(v, in_min, in_max, out_min, out_max):
	if v < in_min:
		v = in_min

	if v > in_max:
		v = in_max
	return (v - in_min) * (out_max - out_min) // (in_max - in_min) + out_min




