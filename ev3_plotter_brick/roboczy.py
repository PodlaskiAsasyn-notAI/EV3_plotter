import turtle
import math

def move_to(x, y):
    a = 100
    b = 100

    turtle.penup()
    turtle.home()
    turtle.pendown()

    r = math.hypot(x, y)
    if r > (a + b) or r < abs(a - b):
        return None

    cos_angle = (x**2 + y**2 - a**2 - b**2) / (2*a*b)
    if cos_angle > 1:
        cos_angle = 1
    elif cos_angle < -1:
        cos_angle = -1

    beta = -math.acos( cos_angle )
    alpha = math.atan2(y, x) + math.atan2( (b*math.sin(beta)), (a + b * math.cos(beta)) )

    beta = math.degrees(beta)
    alpha = math.degrees(alpha)


    turtle.lt(alpha)
    turtle.fd(a)
    turtle.rt(beta)
    turtle.fd(b)



def main():
    turtle.speed(0)
    for x in range(0, 200, 20):
        for y in range(0, 200, 20):
            move_to(x, y)
            turtle.dot(5, "red")
    # while True:
        # pass

if __name__ == "__main__":
    main()
