import math

def get_float(prompt, default=None):
    while True:
        if default is not None:
            value = input(f"{prompt} (default: {default}): ")
        else:
            value = input(prompt)

        if value == "":
            if default is not None:
                value = default
            else:
                raise ValueError("A value is required")

        try:
            value = float(value)
            return value
        except ValueError:
            print("Please enter a number")

# PID controller parameters
Kp = get_float("Enter Kp:", 0.8)
Ki = get_float("Enter Ki:", 0)
Kd = get_float("Enter Kd:", 0.05)
max_steering_angle = get_float("Enter max steering angle:", 30)
max_velocity = get_float("Enter max velocity:", 15)

# Initial state and target
target_x = get_float("Enter target x:", 20)
target_y = get_float("Enter target y:", 20)
current_x = get_float("Enter current x:", 0)
current_y = get_float("Enter current y:", 0)
heading = get_float("Enter heading:", 0)
velocity = get_float("Enter velocity:", 2)

def control_steering(target_x, target_y,
                     current_x, current_y,
                     heading, velocity,
                     dt, wheel_base,
                     acceleration, deceleration,
                     steering_rate,
                     steps, Kp=0.8, Ki=0, Kd=0.05,
                     max_steering_angle=30,
                     max_velocity=15):
    if dt <= 0:
        raise ValueError("dt must be positive")

    def add_result(current_x, current_y,
                   step, heading, steering_angle,
                   error, velocity):
        return {"x": current_x, "y": current_y,
                "step": step + 1,
                "heading": math.degrees(heading),
                "steering_angle": math.degrees(steering_angle),
                "error": math.degrees(error),
                "velocity": velocity}

    result = []
    steering_rate = math.radians(steering_rate)
    max_steering_angle = math.radians(max_steering_angle)
    accumulated_error = 0
    previous_error = 0
    steering_angle = 0
    error = 0

    # Calculate distance and update velocity
    for step in range(steps):
        old_velocity = velocity
        dx = target_x - current_x
        dy = target_y - current_y
        distance = math.sqrt(dx * dx + dy * dy)
        braking_distance = velocity ** 2 / (2 * deceleration)
        if distance > braking_distance:
            if velocity + acceleration * dt >= max_velocity:
                velocity = max_velocity
            else:
                velocity += acceleration * dt
        else:
            if velocity - deceleration * dt > 0:
                velocity -= deceleration * dt
            else:
                velocity = 0
        actual_acceleration = (velocity - old_velocity) / dt
        step_distance = old_velocity * dt + 0.5 * actual_acceleration * dt ** 2

        # Check if target is reached
        if distance <= step_distance:
            current_x = target_x
            current_y = target_y
            entry = add_result(current_x, current_y,
                               step, heading, steering_angle,
                               error, velocity)
            result.append(entry)
            print(f"The object reached the target in {step + 1} steps.")
            break

        # Calculate steering error and PID control
        target_heading = math.atan2(dy, dx)
        error = target_heading - heading
        error = math.atan2(math.sin(error), math.cos(error))
        accumulated_error += error * dt

        p = Kp * error
        i = Ki * accumulated_error
        d = Kd * (error - previous_error) / dt

        previous_error = error
        pid = p + i + d

        # Apply steering constraints
        if pid > max_steering_angle:
            desired_steering_angle = max_steering_angle
        elif pid < -max_steering_angle:
            desired_steering_angle = -max_steering_angle
        else:
            desired_steering_angle = pid

        if steering_angle < desired_steering_angle:
            if steering_angle + steering_rate * dt > desired_steering_angle:
                steering_angle = desired_steering_angle
            else:
                steering_angle += steering_rate * dt
        elif steering_angle > desired_steering_angle:
            if steering_angle - steering_rate * dt < desired_steering_angle:
                steering_angle = desired_steering_angle
            else:
                steering_angle -= steering_rate * dt

        # Update vehicle state
        average_velocity = (velocity + old_velocity) / 2
        angular_velocity = average_velocity / wheel_base * math.tan(steering_angle)
        heading += angular_velocity * dt
        velocity_x = average_velocity * math.cos(heading)
        velocity_y = average_velocity * math.sin(heading)
        current_x += velocity_x * dt
        current_y += velocity_y * dt

        entry = add_result(current_x, current_y,
                           step, heading, steering_angle,
                           error, velocity)
        result.append(entry)
    return result

result1 = control_steering(target_x, target_y,
                           current_x, current_y,
                           heading, velocity,
                           0.1, 1,
                           2, 6,
                           30, 10000,
                           Kp, Ki, Kd,
                           max_steering_angle, max_velocity)

for x in result1:
    print(x)
