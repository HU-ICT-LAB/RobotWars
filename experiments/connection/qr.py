import time
from robomaster import conn
from MyQR import myqr
from PIL import Image
import matplotlib.pyplot as plt
import sys


# Run this script in the console with the wifi and its password or
# add the wifi and its password in the config of the qr
ssid = sys.argv[1]
password = sys.argv[2]

if __name__ == '__main__':
    helper = conn.ConnectionHelper()
    info = helper.build_qrcode_string(ssid=ssid, password=password)
    myqr.run(words=info, save_name=f"{ssid}.png")
    time.sleep(1)
    img = Image.open(f"{ssid}.png")
    plt.imshow(img)
    plt.show()
    if helper.wait_for_connection():
        print("Connected!")
    else:
        print("Connect failed!")
