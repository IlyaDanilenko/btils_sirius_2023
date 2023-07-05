p = script.readAll("param.txt")[0];
param = p[0].split(",")
for (var i = 0; i < param.length; i++){
	param[i] = Number(param[i])
}
brick.gyroscope().setCalibrationValues(param);
script.wait(300);
function getYaw(){
	yaw = brick.gyroscope().read()[6]/1000
	deltaYaw = yaw - yawOld 
	yawOld = yaw
	sgn = yawOld == 0 ? 0 : yawOld/Math.abs(yawOld)
	n -= sgn*Math.floor(Math.abs(deltaYaw/320))
	direction = yaw + n * 360 + bias
	direction += 0.01
    brick.display().addLabel(String(direction), 1, 1);
}

tim = script.timer(100)
tim.timeout.connect(getYaw)