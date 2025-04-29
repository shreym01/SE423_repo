
import image, sensor, ustruct, pyb, time, display, math

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QQVGA) # use QQVGA 160columns*120rows
sensor.set_auto_gain(False,gain_db=11.4801) # must be turned off for color tracking
# sensor.set_auto_whitebal(False,[60.2071, 60.5557, 67.1094]) # set rgb_gain does not work for the new firmware
sensor.set_auto_whitebal(False) # must be turned off for color tracking
lcd = display.SPIDisplay()

uart = pyb.UART(3)
uart.init(115200, bits=8, parity=None)
threshold1 = (27, 85, 21, 64, 11, 60) # change to a color threshold range
threshold2 = (30, 75, 2, 40, -35, -11) # change to a color threshold range
# Packets to Send
blob_packet = '<fff'

# Setup RED LED for easier debugging
red_led   = pyb.LED(1)
green_led = pyb.LED(2)
blue_led  = pyb.LED(3)
clock = time.clock()                # Create a clock object to track the FPS.
framecount = 0
toggle = 0
offcount = 0
tagfound = 0

f_x = (2.8 / 3.984) * 160  # find_apriltags defaults to this if not set
f_y = (2.8 / 2.952) * 120  # find_apriltags defaults to this if not set
c_x = 160 * 0.5  # find_apriltags defaults to this if not set (the image.w * 0.5)
c_y = 120 * 0.5  # find_apriltags defaults to this if not set (the image.h * 0.5)


def degrees(radians):
    return (180 * radians) / math.pi


while True:
    clock.tick()                    # Update the FPS clock.
    if framecount % 2 == 0:
        if toggle == 0:
            blue_led.on()
            toggle = 1
            offcount = 0
        else:
            blue_led.off()
            offcount = offcount + 1
            if offcount == 20:
                toggle = 0
                offcount = 0
    framecount = framecount + 1
    img = sensor.snapshot()

    blobs1 = img.find_blobs([threshold1], roi=(0,60,160,60), pixels_threshold=5, area_threshold=20)
    blobs2 = img.find_blobs([threshold2], roi=(0,60,160,60), pixels_threshold=5, area_threshold=20)

    if blobs1:
        blob1_sort = sorted(blobs1, key = lambda b: b.pixels(), reverse=True)
        blob1_largest = blob1_sort[:3]
        blobs1_found = len(blob1_largest)

        msg = "**".encode()
        uart.write(msg)
        for i in range(3):
            if i < blobs1_found:
                b = blob1_largest[i]
                a = float(b.area())
                x_cnt = float(b.cx())
                y_cnt = float(b.cy())
                img.draw_rectangle(b[0:4]) # rect on x,y,w,h
                img.draw_cross(b.cx(), b.cy())
            else:
                a = 0.0
                x_cnt = 0.0
                y_cnt = 0.0

            # Send the blob area and centroids over UART
            b = ustruct.pack(blob_packet, a, x_cnt, y_cnt)
            uart.write(b)
    else:  # nothing found
        msg = "**".encode()
        uart.write(msg)
        b = ustruct.pack(blob_packet, 0.0, 0.0, 0.0)
        uart.write(b)
        b = ustruct.pack(blob_packet, 0.0, 0.0, 0.0)
        uart.write(b)
        b = ustruct.pack(blob_packet, 0.0, 0.0, 0.0)
        uart.write(b)

    if blobs2:
        blob2_sort = sorted(blobs2, key = lambda b: b.pixels(), reverse=True)
        blob2_largest = blob2_sort[:3]
        blobs2_found = len(blob2_largest)

        msg = "*!".encode()
        uart.write(msg)
        for i in range(3):
            if i < blobs2_found:
                b = blob2_largest[i]
                a = float(b.area())
                x_cnt = float(b.cx())
                y_cnt = float(b.cy())
                img.draw_rectangle(b[0:4]) # rect on x,y,w,h
                img.draw_cross(b.cx(), b.cy())
            else:
                a = 0.0
                x_cnt = 0.0
                y_cnt = 0.0

            # Send the blob area and centroids over UART
            b = ustruct.pack(blob_packet, a, x_cnt, y_cnt)
            uart.write(b)
    else:  # nothing found
        msg = "*!".encode()
        uart.write(msg)
        b = ustruct.pack(blob_packet, 0.0, 0.0, 0.0)
        uart.write(b)
        b = ustruct.pack(blob_packet, 0.0, 0.0, 0.0)
        uart.write(b)
        b = ustruct.pack(blob_packet, 0.0, 0.0, 0.0)
        uart.write(b)
    tagfound = 0
    for tag in img.find_apriltags(
        fx=f_x, fy=f_y, cx=c_x, cy=c_y
    ):  # defaults to TAG36H11
        tagfound = 1
        img.draw_rectangle(tag.rect(), color=(255, 0, 0))
        img.draw_cross(tag.cx(), tag.cy(), color=(0, 255, 0))
        print_args = (
            tag.id(),
            tag.x_translation(),
            tag.y_translation(),
            tag.z_translation(),
            degrees(tag.x_rotation()),
            degrees(tag.y_rotation()),
            degrees(tag.z_rotation()),
        )
        # Translation units are unknown. Rotation units are in degrees.
        print("id: %d Tx: %f, Ty %f, Tz %f, Rx %f, Ry %f, Rz %f" % print_args)
        if tag.id() <= 2.0:   #if two or more tags are found the send will send again and robot will receive both in the same variables write over the first one  If two tags are needed, more code is needed.
            msg = "*$".encode()
            uart.write(msg)
            b = ustruct.pack(blob_packet, tag.x_translation(), tag.y_translation(), tag.z_translation())
            uart.write(b)
            b = ustruct.pack(blob_packet, degrees(tag.x_rotation()), degrees(tag.y_rotation()), degrees(tag.z_rotation()))
            uart.write(b)
            b = ustruct.pack(blob_packet, tag.id(), 0.0, 0.0)
            uart.write(b)
    if tagfound == 0:
        msg = "*$".encode()
        uart.write(msg)
        b = ustruct.pack(blob_packet, 0.0, 0.0, 0.0)
        uart.write(b)
        b = ustruct.pack(blob_packet, 0.0, 0.0, 0.0)
        uart.write(b)
        b = ustruct.pack(blob_packet, -1.0, 0.0, 0.0)  #send no tag found
        uart.write(b)

    #roi is left, top, width, height
    #lcd.write(img, roi=(96,80,128,160)) # display the image to lcd only middle 128 cols by 160 rows.
    lcd.write(img, roi=(16,0,128,160))
    print(clock.fps())              # Note: OpenMV Cam runs about half as fast when connected
