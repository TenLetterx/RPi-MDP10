import json
import logging
import os
import time

import cv2
import numpy as np
import requests

try:
    from picamera2 import Picamera2
except ImportError:
    raise ImportError("picamera2 is required but not installed. Run: sudo apt install -y python3-picamera2")

# Legacy PiCamera is not supported on Raspberry Pi OS Bullseye/Bookworm,
# so we disable it by default.
PiCamera = None

logger = logging.getLogger(__name__)
con_file = "PiLCConfig9.txt"
home_files = []
home_files.append(str(os.getenv("USER")))
config_file = "/home/" + "/pi" + "/" + con_file

extns = ["jpg", "png", "bmp", "rgb", "yuv420", "raw"]

shutters = [
    -2000,-1600,-1250,-1000,-800,-640,-500, 
    -400,-320,-288,-250,-240,-200,-160,-144,
    -125,-120,-100,-96,-80,-60,-50,-48,-40,
    -30,-25,-20,-15,-13,-10,-8,-6,-5,
    -4,-3,0.4,0.5,0.6,0.8,1,1.1,1.2,2,3,4,5,6,
    7,8,9,10,11,15,20,25,30,40,50,60,75,
    100,112,120,150,200,220,230,239,435
]

meters = ["centre", "spot", "average"]
awbs = ["off", "auto", "incandescent", "tungsten", "fluorescent", "indoor", "daylight", "cloudy"]
denoises = ["off", "cdn_off", "cdn_fast", "cdn_hq"]
    
def calculate_brightness(image_path: str) -> int:
    """
    calculate brightness based on average pixel brightness (ranges between 0 and 255)
    """
    image = cv2.imread(image_path)
    grey_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    avg_brightness = np.mean(grey_image)
    logger.info(f"The average brightness is {avg_brightness}")
    return avg_brightness


def adjust_camera_settings(avg_brightness: int) -> tuple[int, int, int]:
    """
    offsets speed,gain,brightness based on avg_brightness
    """
    if avg_brightness < 60:
        return 10, 5, 20  # Low light: increase shutter speed and gain
    elif avg_brightness > 200:
        return 10, -5, -20  # Bright light: decrease shutter speed and gain
    else:
        return 0, 0, 0  # Normal light


def snap_using_libcamera(
    obstacle_id: str,
    signal: str,
    filename: str,
    filename_send: str,
    url: str,
    auto_callibrate: bool,
) -> str:
    """
    RPi snaps an image and calls the API for image-rec.
    The image is saved in /home/pi/cam
    The response is then forwarded back to the android
    """

    config = []
    with open(config_file, "r") as file:
        line = file.readline()
        while line:
            config.append(line.strip())
            line = file.readline()
        config = list(map(int, config))
    
    mode = config[0]
    speed = config[1]
    gain = config[2]
    brightness = config[3]
    contrast = config[4]
    red = config[6]
    blue = config[7]
    ev = config[8]
    extn = config[15]
    saturation = config[19]
    meter = config[20]
    awb = config[21]
    sharpness = config[22]
    denoise = config[23]
    quality = config[24]
    
    if auto_callibrate == True:
        image_callibration_filepath = f"/home/pi/cam/sample_ambient_light.jpg"
        callibrate_camera_cmd = "libcamera-still -e " + extns[extn] + " -n -t 500 -o " + image_callibration_filepath
        os.system(callibrate_camera_cmd)
        dspeed, dgain, dbrightness = adjust_camera_settings(calculate_brightness(image_callibration_filepath))
        speed = speed + dspeed
        gain = gain + dgain
        brightness = brightness + dbrightness

    retry_count = 0

    while True:
        retry_count += 1

        shutter = shutters[speed]
        if shutter < 0:
            shutter = abs(1 / shutter)
        sspeed = int(shutter * 1000000)
        if (shutter * 1000000) - int(shutter * 1000000) > 0.5:
            sspeed += 1

        rpistr = "libcamera-still -e " + extns[extn] + " -n -t 500 -o " + filename
        rpistr += " --brightness " + str(brightness / 100) + " --contrast " + str(contrast / 100)
        rpistr += " --shutter " + str(sspeed)
        if ev != 0:
            rpistr += " --ev " + str(ev)
        if sspeed > 1000000 and mode == 0:
            rpistr += " --gain " + str(gain) + " --immediate "
        else:
            rpistr += " --gain " + str(gain)
            if awb == 0:
                rpistr += " --awbgains " + str(red / 10) + "," + str(blue / 10)
            else:
                rpistr += " --awb " + awbs[awb]
        rpistr += " --metering " + meters[meter]
        rpistr += " --saturation " + str(saturation / 10)
        rpistr += " --sharpness " + str(sharpness / 10)
        rpistr += " --quality " + str(quality)
        rpistr += " --denoise " + denoises[denoise]
        rpistr += " --metadata - --metadata-format txt >> PiLibtext.txt"
        os.system(rpistr)
        
        response = requests.post(url, files={"file": (filename_send, open(filename, "rb"))})
        
        if response.status_code != 200:
            logger.error("Error from image-rec API.")
            raise OSError("API Error")

        results = json.loads(response.content)
        return results

#def snap_using_picamera(
    #filename: str,
    #filename_send: str,
    #url: str,
#) -> str:
    #my_file = open(filename, "wb")
    #time.sleep(1)
    #with PiCamera() as camera:
        #camera.resolution = (640, 480)
        #camera.start_preview()
        #camera.capture(my_file)
        #camera.stop_preview()

    #my_file.close()
    #response = requests.post(url, files={"file": (filename_send, open(filename, "rb"))})
    
    #if response.status_code != 200:
        #logger.error("Error from image-rec API.")
        #raise OSError("API Error")

    #results = json.loads(response.content)
    #logger.debug(results)
    #return results

def snap_using_picamera2(
    filename: str,
    filename_send: str,
    url: str,
    # picam2,
) -> str:

    retry_count = 0
    max_retries = 3
    timeout_seconds = 20

    while retry_count < max_retries:
        retry_count += 1

        picam2 = Picamera2()
        config = picam2.create_still_configuration()
        picam2.configure(config)
        picam2.start()
        picam2.capture_file(filename)
        picam2.close()
        # picam2.start_and_capture_file(filename)
        logger.info("Image captured. Sending to image-rec API...")

        try:
            # Only append /image if not already present
            if not url.endswith("/image"):
                url = url.rstrip("/") + "/image"

            with open(filename, "rb") as f:
                response = requests.post(
                    url,
                    files={"file": (filename_send, f)},
                    timeout=timeout_seconds,
                )

            if response.status_code != 200:
                logger.error(f"Error from image-rec API. Status: {response.status_code}")
                raise OSError("API Error")

            results = json.loads(response.content)
            return results

        except Exception as e:
            logger.warning(f"Request failed ({e}). Retrying {retry_count}/{max_retries}...")
            if retry_count == max_retries:
                raise OSError("API request timed out after multiple attempts")
