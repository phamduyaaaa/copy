# copy
```bash
import math
import time

# ================= ROBOT MODEL =================

class Robot:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.angle = 0.0
        self.speed = 0.0
        self.turn_rate = 0.0

    def forward(self, v):
        self.speed = v
        self.turn_rate = 0.0

    def steer(self, v, w):
        self.speed = v
        self.turn_rate = w

    def update(self):
        self.angle += self.turn_rate
        self.x += math.sin(self.angle) * self.speed * 0.01
        self.y += math.cos(self.angle) * self.speed * 0.01

# ================= VIRTUAL LINE =================

class VirtualLine:
    def get_y(self, x):
        return math.sin(x)  # line cong

# ================= LOOKAHEAD LINE FOLLOWING =================

class LineFollower:
    def __init__(self):
        self.robot = Robot()
        self.line = VirtualLine()

        # lookahead distance (quan trọng nhất)
        self.lookahead = 0.6

        # PID đơn giản
        self.kp = 1.2
        self.kd = 0.3
        self.prev_error = 0.0

        self.v = 35  # tốc độ cố định, chạy chậm

    def step(self):
        # 1. Dự đoán điểm phía trước robot (bỏ qua vùng mù)
        lx = self.robot.x + math.sin(self.robot.angle) * self.lookahead
        ly = self.line.get_y(lx)

        # 2. Vector từ robot tới điểm lookahead
        dx = lx - self.robot.x
        dy = ly - self.robot.y

        # 3. Góc mong muốn
        desired_angle = math.atan2(dx, dy)

        # 4. Heading error
        error = desired_angle - self.robot.angle
        error = math.atan2(math.sin(error), math.cos(error))  # normalize

        # 5. PD control
        d_error = error - self.prev_error
        w = self.kp * error + self.kd * d_error
        w = max(min(w, 0.15), -0.15)  # clamp

        self.prev_error = error

        # 6. Điều khiển robot
        self.robot.steer(self.v, w)
        self.robot.update()

        print(
            f"POS=({self.robot.x:.2f},{self.robot.y:.2f}) "
            f"ANGLE={self.robot.angle:.2f} "
            f"ERR={error:.3f}"
        )

# ================= MAIN =================

if __name__ == "__main__":
    follower = LineFollower()

    while True:
        follower.step()
        time.sleep(0.05)

```
