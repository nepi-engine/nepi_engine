import time
import datetime
import pytz

tzd = 'America/New_York'

tzo = pytz.timezone(tzd)
print(tzo)

dt_now = datetime.datetime.now(tzo).strftime("%m/%d/%Y,%H:%M:%S")
print(dt_now)

time_sec = time.time()
dt = datetime.datetime.fromtimestamp(time_sec,tzo)
print(dt)
