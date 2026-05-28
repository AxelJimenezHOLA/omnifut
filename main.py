import sensor
import image
import time
import display
from pyb import UART
from machine import Pin

# ── COLOR THRESHOLDS ───────────────────────────────────────────────────────────
THRESHOLD = {
    "red": (0, 100, 40, 80, 15, 75),
    "blue": (0, 100, 5, 55, -100, -35),
    "yellow": (0, 100, -15, 20, 40, 90)
}

# ── CAMERA SETUP ───────────────────────────────────────────────────────────────
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)  # QVGA 320×240 | CIF - 352x288
sensor.skip_frames(time=2000)
sensor.set_auto_whitebal(False)
sensor.set_auto_gain(False)

clock = time.clock()

# ── LCD SETUP ──────────────────────────────────────────────────────────────────
lcd = display.SPIDisplay(vflip=True, hmirror=True, bgr=True, width=80, height=160)
lcd.bus_write(0x21)

lcd_x = 26
lcd_y = 1
lcd.bus_write(cmd=0x2A, args=bytearray([0, lcd_x & 0xFF, 0, lcd_x + 80 - 1]))
lcd.bus_write(cmd=0x2B, args=bytearray([0, lcd_y & 0xFF, 0, lcd_y + 160 - 1]))
lcd.bus_write(0x2C)
lcd.bus_write(cmd=0xC0, args=bytearray([0xAB, 0x0B]))
lcd.bus_write(cmd=0xC1, args=bytearray([0xC5]))
lcd.bus_write(cmd=0xC2, args=bytearray([0x0D, 0x00]))
lcd.bus_write(cmd=0xC4, args=bytearray([0x8D, 0xEE]))
lcd.bus_write(cmd=0xC5, args=bytearray([0x0F]))
lcd.bus_write(cmd=0xE0, args=bytearray([0x07, 0x0E, 0x08, 0x07, 0x10, 0x07, 0x02,
                                        0x07, 0x09, 0x0F, 0x25, 0x36, 0x00, 0x08, 0x04, 0x10]))
lcd.bus_write(cmd=0xE1, args=bytearray([0x0A, 0x0D, 0x08, 0x07, 0x0F, 0x07, 0x02,
                                        0x07, 0x09, 0x0F, 0x25, 0x35, 0x00, 0x09, 0x04, 0x10]))
lcd.backlight(0)

# ── K1 BUTTON SETUP ────────────────────────────────────────────────────────────
K1 = {
    "pin": Pin("C13", Pin.IN, Pin.PULL_DOWN),
    "last": 0,
    "now": 0
}

# ── COLORS ─────────────────────────────────────────────────────────────────────
def rgb565(r, g, b): return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3)

color565 = {
    "red": rgb565(255, 50, 50),
    "blue": rgb565(50, 50, 255),
    "yellow": rgb565(255, 255, 50),
    "black": rgb565(0, 0, 0)
}

SECONDARY_COLORS = [
    {"name": "BLUE",   "threshold": THRESHOLD["blue"],   "color": color565["blue"]},
    {"name": "YELLOW", "threshold": THRESHOLD["yellow"], "color": color565["yellow"]}
]

color_idx = 0

# ── FRAME ──────────────────────────────────────────────────────────────────────
WIDTH = sensor.width()
HEIGHT = sensor.height()
FULL_ROI = (0, 0, WIDTH, HEIGHT)
CX = WIDTH // 2
CY = HEIGHT // 2
MIN_AREA = 400

# ── UART ───────────────────────────────────────────────────────────────────────
uart = UART(3, 115200, timeout_char=1000)


def send_uart(r_found, r_cx, r_cy, s_found, s_cx, s_cy, s_id):
    data = bytearray([
        0xAA, 0xBB,                       # HEADER
        1 if r_found else 0,              # R_FOUND
        (r_cx >> 8) & 0xFF, r_cx & 0xFF,  # R_CX (HI & LO)
        (r_cy >> 8) & 0xFF, r_cy & 0xFF,  # R_CY (HI & LO)
        s_id,                             # S_ID (0=BLUE, 1=YELLOW)
        1 if s_found else 0,              # S_FOUND
        (s_cx >> 8) & 0xFF, s_cx & 0xFF,  # S_CX (HI & LO)
        (s_cy >> 8) & 0xFF, s_cy & 0xFF,  # S_CY (HI & LO)
    ])
    data.append(sum(data) & 0xFF)
    uart.write(data)

# ── LCD OVERLAY ───────────────────────────────────────────────────────────────
lcd_every = 3
frame_n = 0

def draw_overlay(img, r_found, r_cx, r_cy, s_found, s_cx, s_cy, s_color):
    SCALE = 2
    HEIGHT_CHAR = 8 * SCALE
    BORDER = 4
    ALTO_FRANJA = (HEIGHT_CHAR * 2) + (BORDER * 3)
    Y_FRANJA = HEIGHT - ALTO_FRANJA

    # Franja de fondo en la parte inferior
    img.draw_rectangle(0, Y_FRANJA, WIDTH, ALTO_FRANJA, color=color565["black"], fill=True)

    # Coordenadas del rojo
    r_txt = f"X:{r_cx:3d} Y:{r_cy:3d}" if r_found else "X:--- Y:---"
    # Coordenadas del secundario
    s_txt = f"X:{s_cx:3d} Y:{s_cy:3d}" if s_found else "X:--- Y:---"

    # Una sola línea con ambos en sus colores
    img.draw_string(BORDER, Y_FRANJA + BORDER, r_txt, color=color565["red"], scale=SCALE)
    img.draw_string(BORDER, Y_FRANJA + BORDER + HEIGHT_CHAR + BORDER, s_txt, color=s_color, scale=SCALE)

# ── MAIN LOOP ──────────────────────────────────────────────────────────────────
while True:
    clock.tick()

    # ── Leer botón K1 con detección de flanco ────────────────────────────────
    K1["now"] = K1["pin"].value()
    if K1["now"] == 1 and K1["last"] == 0:
        # Flanco de subida: cambiar color secundario
        color_idx = 1 - color_idx
        sec = SECONDARY_COLORS[color_idx]
        print(f"Color secundario: {sec['name']}")
        time.sleep_ms(50)   # antirrebote mínimo
    K1["last"] = K1["now"]

    # ── Color secundario activo ───────────────────────────────────────────────
    sec = SECONDARY_COLORS[color_idx]
    sec_name = sec["name"]
    sec_color = sec["color"]
    sec_thr = sec["threshold"]

    # ── Captura y detección (dos thresholds en una sola llamada) ─────────────
    img = sensor.snapshot()
    # ── find_blobs evalúa dos thresholds en una sola pasada por el frame
    # blob.code()==1 → rojo (primer threshold)
    # blob.code()==2 → secundario activo (segundo threshold)
    blobs = img.find_blobs([THRESHOLD["red"], sec_thr], roi=FULL_ROI, merge=False)

    # Separar blobs por código
    red_blobs = [b for b in blobs if b.code() == 1 and b.w() * b.h() >= MIN_AREA]
    sec_blobs = [b for b in blobs if b.code() == 2 and b.w() * b.h() >= MIN_AREA]

    # Tomar el blob más grande de cada grupo
    # ── Se conserva solo el blob de mayor área por color
    # Descarta reflejos y detecciones múltiples del mismo objeto
    red_blob = max(red_blobs, key=lambda b: b.w() * b.h()) if red_blobs else None
    sec_blob = max(sec_blobs, key=lambda b: b.w() * b.h()) if sec_blobs else None

    # ── Datos del objeto rojo ─────────────────────────────────────────────────
    if red_blob:
        r_cx = red_blob.cx()
        r_cy = red_blob.cy()
        img.draw_rectangle(red_blob.rect(), color=color565["red"], thickness=4)
        img.draw_cross(r_cx, r_cy, color=color565["red"], size=10)
        img.draw_string(red_blob.x(), max(red_blob.y() - 12, 0),
                        "RED", color=color565["red"], scale=2)
        r_found = True
    else:
        r_cx, r_cy = 0, 0
        r_found = False

    # ── Datos del objeto secundario ───────────────────────────────────────────
    if sec_blob:
        s_cx = sec_blob.cx()
        s_cy = sec_blob.cy()
        img.draw_rectangle(sec_blob.rect(), color=sec_color, thickness=4)
        img.draw_cross(s_cx, s_cy, color=sec_color, size=10)
        img.draw_string(sec_blob.x(), max(sec_blob.y() - 12, 0),
                        sec_name, color=sec_color, scale=2)
        s_found = True
    else:
        s_cx, s_cy = 0, 0
        s_found = False

    # ── Consola ───────────────────────────────────────────────────────────────
    r_str = f"X={r_cx} Y={r_cy}" if r_found else "NOT FOUND"
    s_str = f"X={s_cx} Y={s_cy}" if s_found else "NOT FOUND"
    print(f"RED:{r_str} | {sec_name}:{s_str} | fps={clock.fps():.1f}")

    # ── UART ──────────────────────────────────────────────────────────────────
    send_uart(r_found, r_cx, r_cy, s_found, s_cx, s_cy, color_idx)

    # ── LCD ───────────────────────────────────────────────────────────────────
    frame_n += 1
    if frame_n >= lcd_every:
        frame_n = 0
        # ── draw_overlay modifica img en memoria antes de lcd.write
        # Una sola escritura SPI por actualización = menos parpadeo
        draw_overlay(img, r_found, r_cx, r_cy, s_found, s_cx, s_cy, sec_color)
        lcd.write(img, hint=image.CENTER | image.SCALE_ASPECT_KEEP | image.ROTATE_90)
