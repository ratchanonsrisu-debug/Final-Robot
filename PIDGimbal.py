import time
from robomaster import robot, vision, blaster


# --------------------------------------------------
# คลาสสำหรับคำนวณ PID Controller
# --------------------------------------------------
class PIDController:
    def __init__(self, kp, ki, kd, limits=(-100, 100)):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.min_limit, self.max_limit = limits

        self.last_error = 0.0
        self.integral = 0.0
        self.last_time = time.time()

    def compute(self, error):
        now = time.time()
        dt = now - self.last_time
        if dt <= 0:
            dt = 0.01  # ป้องกันการหารด้วยศูนย์

        # 1. Proportional term
        p_term = self.kp * error

        # 2. Integral term (สะสมค่า Error)
        self.integral += error * dt
        i_term = self.ki * self.integral

        # 3. Derivative term (อัตราการเปลี่ยนแปลงของ Error)
        derivative = (error - self.last_error) / dt
        d_term = self.kd * derivative

        # รวมค่าความเร็ว
        output = p_term + i_term + d_term

        # จำกัดความเร็วสูงสุด/ต่ำสุด (Output Clamping)
        output = max(self.min_limit, min(self.max_limit, output))

        # บันทึกค่าไว้ใช้ในรอบถัดไป
        self.last_error = error
        self.last_time = now

        return output

    def reset(self):
        self.last_error = 0.0
        self.integral = 0.0
        self.last_time = time.time()


# --------------------------------------------------
# ตั้งค่าระบบ PID สำหรับ Yaw และ Pitch
# --------------------------------------------------
# ปรับแต่งค่า Kp, Ki, Kd ตามความเหมาะสมของหุ่นยนต์
pid_yaw = PIDController(kp=80.0, ki=0.0, kd=0.0, limits=(-180, 180))
pid_pitch = PIDController(kp=80.0, ki=0.0, kd=0.0, limits=(-120, 120))

target_locked = False
ep_gimbal = None
ep_blaster = None


def on_detect_marker(marker_info):
    global target_locked, ep_gimbal, ep_blaster, pid_yaw, pid_pitch

    if target_locked:
        return

    for marker in marker_info:
        x, y, w, h, info = marker
        print("[DEBUG] เห็น marker: info={0} x={1:.2f} y={2:.2f}".format(info, x, y))

        # เล็กล็อกเป้าหมายป้ายหมายเลข "1"
        if info == "1":
            # คำนวณ Error สัมพัทธ์กับจุดกึ่งกลางจอ (0.5)
            err_x = x - 0.5
            err_y = 0.5 - y

            # คำนวณความเร็วด้วย PID
            yaw_speed = pid_yaw.compute(err_x)
            pitch_speed = pid_pitch.compute(err_y)

            # เช็กเงื่อนไข ล็อกเป้าเมื่อ Error ต่ำกว่า 3% (|Error| < 0.03)
            if abs(err_x) < 0.03 and abs(err_y) < 0.03:
                ep_gimbal.drive_speed(pitch_speed=0, yaw_speed=0)
                print(">> ล็อกเป้าสำเร็จด้วย PID! -> ยิง!")

                target_locked = True
                ep_blaster.fire(fire_type=blaster.IR_FIRE, times=1)

                # รีเซ็ตค่าสะสมของ PID เมื่อยิงเสร็จ
                pid_yaw.reset()
                pid_pitch.reset()
            else:
                # สั่ง Gimbal เคลื่อนที่ตามความเร็ว PID
                ep_gimbal.drive_speed(pitch_speed=pitch_speed, yaw_speed=yaw_speed)
            break


def main():
    global ep_gimbal, ep_blaster

    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")

    ep_gimbal = ep_robot.gimbal
    ep_blaster = ep_robot.blaster
    ep_vision = ep_robot.vision
    ep_camera = ep_robot.camera

    print("เปิดใช้งานกล้อง และเซ็ต Gimbal...")
    ep_camera.start_video_stream(display=False)
    ep_gimbal.recenter().wait_for_completed()
    time.sleep(1)

    print("เริ่มค้นหา Vision Marker หมายเลข 1 ด้วย PID Auto-Aim...")
    ep_vision.sub_detect_info(name=vision.MARKER, callback=on_detect_marker)

    global target_locked
    for _ in range(30):
        if target_locked:
            time.sleep(2)
            print(">> พร้อมค้นหาเป้าหมายถัดไป...")
            target_locked = False
        time.sleep(0.1)

    # คืนทรัพยากร
    ep_vision.unsub_detect_info(name=vision.MARKER)
    ep_camera.stop_video_stream()
    ep_robot.close()


if __name__ == "__main__":
    main()
