var __interpretation_started_timestamp__;
var pi = 3.141592653589793;

forw = brick.sensor(A3).read
back = brick.sensor(A4).read
rght = brick.sensor(A5).read
left = brick.sensor(A6).read

e3 = brick.encoder(E3)
e4 = brick.encoder(E4)

r = brick.sensor(A1).read
f = brick.sensor(A2).read

m1 = brick.motor(M1).setPower
m2 = brick.motor(M2).setPower
m3 = brick.motor(M3).setPower
m4 = brick.motor(M4).setPower

yawOld = 0
n = 0
direction = 0
dir = 0

kVol = 1+(1-brick.battery().readVoltage()/12.5)*6
k = 1 // 1 коэф. выравнивания
kp = 1  // 1
v = 41 * kVol // 55
vr = 20 * kVol // 35
bias = 0

function sign(a){
	if (a > 0){
		return 1;
	} else if (a < 0){
		return -1;
	} else { 
		return 0;
	}
}

function getYaw(){
	yaw = brick.gyroscope().read()[6]/1000
	deltaYaw = yaw - yawOld 
	yawOld = yaw
	sgn = yawOld == 0 ? 0 : yawOld/Math.abs(yawOld)
	n -= sgn*Math.floor(Math.abs(deltaYaw/320))
	direction = yaw + n * 360 + bias
	direction += 0.01
//	print(direction)
}

function yawPrint(){
	print(direction)
}

print("--------------------")
p = script.readAll("param.txt")
param = p[0].split(",")
for (var i = 0; i < param.length; i++){
	param[i] = Number(param[i])
}


print(param)
print("vol=", 1+(1-brick.battery().readVoltage()/12.5))

brick.gyroscope().setCalibrationValues(param)
script.wait(300)

print(v,"  ",vr)

tim = script.timer(100)
tim.timeout.connect(getYaw)

tPrint = script.timer(300)
tPrint.timeout.connect(yawPrint)

angle = 89

e3.reset()
e4.reset()
script.wait(100)

flag = false

brick.display().addLabel("dir=" + dir,10,10,30)
brick.display().redraw()

while (!brick.keys().wasPressed(KeysEnum.Esc)){
	
	if(brick.keys().wasPressed(KeysEnum.Up)){
		dir = 0
		bias += dir - direction
		e3.reset()
		e4.reset()
		brick.display().addLabel("dir=" + dir,10,10,30)
		brick.display().redraw()
		script.wait(100)
	}
	
	if(r() < 20){
		m1(100)
	} else {
		m1(0)
	}
	
	if(f() < 12){
		m2(100)
		if (flag == true){
			brick.motor(M3).brake(200)
			brick.motor(M4).brake(200)
			script.wait(300)
			flag = false
		}
	} else {
		m2(0)
		if (flag == false){
			
			flag = true
		}
	}
	
	if(forw()>50){
		err = dir - direction
		errEn = e3.read() - e4.read()
		u = 0 * errEn + (k+0.8) *err
		if (r() < 30){
			u = -2.5 * (12 - r())
		}
		m3(v + u)
		m4(v - u)
//		print("run")
	} else if(back() > 50) {
//		err = dir - direction
//		errEn = e3.read() - e4.read()
//		u = -k * errEn
		m3(-v-20)
		m4(-v-20)
		print("back")
		
//		bias += dir - direction
	} else if(rght() > 50) {
		script.wait(600)
		print("rotate R")
		dir = direction + angle
		brick.display().addLabel("dir=" + dir,10,10,30)
		brick.display().redraw()
//		e3.reset()
//		en = 310
//		while(Math.abs(e3.read()) < en){
//			m3(vr)
//			m4(-vr)
//		}
		while(Math.abs(dir-direction)>2){
			err = dir - direction
			u = kp * err
			if (Math.abs(u) > vr){
				m3(u)
				m4(-u)
			} else {
				m3(vr*sign(u))
				m4(-vr*sign(u))
			}
			script.wait(1)
		}
		brick.motor(M3).brake(100)
		brick.motor(M4).brake(100)
		e3.reset()
		e4.reset()
		script.wait(120)
		print("dir=",dir)
	} else if(left() > 50) {
		script.wait(600)
		dir = direction - angle 
		print("rotate L")
		brick.display().addLabel("dir=" + dir,10,10,30)
		brick.display().redraw()
		while(Math.abs(dir-direction)>2){
			err = dir - direction
			u = kp * err
			if (Math.abs(u) > vr){
				m3(u)
				m4(-u)
			} else {
				m3(vr*sign(u))
				m4(-vr*sign(u))
			}
			script.wait(1)
		}
		brick.motor(M3).brake(100)
		brick.motor(M4).brake(100)
		e3.reset()
		e4.reset()		
		script.wait(120)
		print("dir=",dir)
	} else {
		m3(0)
		m4(0)
	}
	script.wait(1)
}