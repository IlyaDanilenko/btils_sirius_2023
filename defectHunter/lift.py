mailbox.connect("192.168.77.1", 8889)

liftMotor = brick.motor(M1).setPower
liftEnc = brick.encoder(E1)

while True:
    if mailbox.hasMessages():
        msg = mailbox.receive()
        if msg == "get box":
            liftEnc.reset()
            while liftEnc.read() < 360 * 24.9:
                liftMotor(100)
                script.wait(10)
            liftMotor(0)
            script.wait(300)
            mailbox.send(1, "box in")
            msg2 = mailbox.receive()
            oldEnc = liftEnc.read()
            if msg2 == "lift up":
                while liftEnc.read() - oldEnc < 270:
                    liftMotor(100)
                    script.wait(10)
            liftMotor(0)
            script.wait(300)
            mailbox.send(1, "box out")
            msg3 = mailbox.receive()
            if msg3 == "lower down":
                while liftEnc.read() > 20:
                    liftMotor(-60)
                    script.wait(10)
                mailbox.send(1, "ready")
            liftMotor(0)
            script.wait(300)
            break
    script.wait(100)
