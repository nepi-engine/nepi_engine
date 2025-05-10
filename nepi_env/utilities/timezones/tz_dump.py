import yaml

cvs = [
  {
    "timezone": "UTC",
    "offset_str": "UTC-00",
    "offset_num": -0
  },
  {
    "timezone": "America/New_York",
    "offset_str": "UTC-05",
    "offset_num": -5
  },
  {
    "timezone": "America/Chicago",
    "offset_str": "UTC-06",
    "offset_num": -6
  },
  {
    "timezone": "America/Denver",
    "offset_str": "UTC-07",
    "offset_num": -7
  },
  {
    "timezone": "America/Phoenix",
    "offset_str": "UTC-07",
    "offset_num": -7
  },
  {
    "timezone": "America/Los Angeles",
    "offset_str": "UTC-08",
    "offset_num": -8
  },
  {
    "timezone": "America/Anchorage",
    "offset_str": "UTC-09",
    "offset_num": -9
  },
  {
    "timezone": "Pacific/Honolulu",
    "offset_str": "UTC-10",
    "offset_num": -10
  },
  {
    "timezone": "Africa/Johannesburg",
    "offset_str": "UTC+02",
    "offset_num": +2
  },
  {
    "timezone": "America/Mexico City",
    "offset_str": "UTC-06",
    "offset_num": -6
  },
  {
    "timezone": "Africa/Monrousing",
    "offset_str": "UTC+00",
    "offset_num": +0
  },
  {
    "timezone": "Asia/Tokyo",
    "offset_str": "UTC+09",
    "offset_num": +9
  },
  {
    "timezone": "America/Jamaica",
    "offset_str": "UTC-05",
    "offset_num": -5
  },
  {
    "timezone": "Europe/Rome",
    "offset_str": "UTC+01",
    "offset_num": +1
  },
  {
    "timezone": "Asia/Hong Kong",
    "offset_str": "UTC+08",
    "offset_num": +8
  },
  {
    "timezone": "Pacific/Guam",
    "offset_str": "UTC+10",
    "offset_num": +10
  },
  {
    "timezone": "Europe/Athens",
    "offset_str": "UTC+02",
    "offset_num": +2
  },
  {
    "timezone": "Europe/London",
    "offset_str": "UTC+00",
    "offset_num": +0
  },
  {
    "timezone": "Europe/Paris",
    "offset_str": "UTC+01",
    "offset_num": +1
  },
  {
    "timezone": "Europe/Madrid",
    "offset_str": "UTC+01",
    "offset_num": +1
  },
  {
    "timezone": "Africa/Cairo",
    "offset_str": "UTC+02",
    "offset_num": +2
  },
  {
    "timezone": "Europe/Copenhagen",
    "offset_str": "UTC+01",
    "offset_num": +1
  },
  {
    "timezone": "Europe/Berlin",
    "offset_str": "UTC+01",
    "offset_num": +1
  },
  {
    "timezone": "Europe/Prague",
    "offset_str": "UTC+01",
    "offset_num": +1
  },
  {
    "timezone": "America/Vancouver",
    "offset_str": "UTC-08",
    "offset_num": -8
  },
  {
    "timezone": "America/Edmonton",
    "offset_str": "UTC-07",
    "offset_num": -7
  },
  {
    "timezone": "America/Toronto",
    "offset_str": "UTC-05",
    "offset_num": -5
  },
  {
    "timezone": "America/Montreal",
    "offset_str": "UTC-05",
    "offset_num": -5
  },
  {
    "timezone": "America/Sao Paulo",
    "offset_str": "UTC-03",
    "offset_num": -3
  },
  {
    "timezone": "Europe/Brussels",
    "offset_str": "UTC+01",
    "offset_num": +1
  },
  {
    "timezone": "Australia/Perth",
    "offset_str": "UTC+08",
    "offset_num": +8
  },
  {
    "timezone": "Australia/Sydney",
    "offset_str": "UTC+10",
    "offset_num": +10
  },
  {
    "timezone": "Asia/Seoul",
    "offset_str": "UTC+09",
    "offset_num": +9
  },
  {
    "timezone": "Africa/Lagos",
    "offset_str": "UTC+01",
    "offset_num": +1
  },
  {
    "timezone": "Europe/Warsaw",
    "offset_str": "UTC+01",
    "offset_num": +1
  },
  {
    "timezone": "America/Puerto Rico",
    "offset_str": "UTC-04",
    "offset_num": -4
  },
  {
    "timezone": "Europe/Moscow",
    "offset_str": "UTC+04",
    "offset_num": +4
  },
  {
    "timezone": "Asia/Manila",
    "offset_str": "UTC+08",
    "offset_num": +8
  },
  {
    "timezone": "Atlantic/Reykjavik",
    "offset_str": "UTC+00",
    "offset_num": +0
  },
  {
    "timezone": "Asia/Jerusalem",
    "offset_str": "UTC+02",
    "offset_num": +2
  }
]

tz_dict = dict()
for i in range(len(cvs)):
  entry=cvs[i]
  key = entry["timezone"]
  offset = int(entry['offset_num'])
  tz_dict[key]=offset
print(tz_dict)
with open('data.yaml', 'w') as file:
  yaml.dump(tz_dict, file, sort_keys=False, default_flow_style=False)
