import subprocess
import time

ret = subprocess.Popen("hostname -I", shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
ret.wait()

if ret.stdout.read() == "\n":
    exit()

while True:
    print "wifi OK!"
    time.sleep(10)
