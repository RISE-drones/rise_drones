import argparse
import json
import math

parser = argparse.ArgumentParser()
parser.add_argument('--rotatedeg', default=0)
parser.add_argument('--file', default='MissionXY.json')
args = parser.parse_args()
rot_deg = int(args.rotatedeg)
mission_file = args.file

rot_rad = rot_deg/180*math.pi
with open(mission_file,'r', encoding='utf-8') as infile:
  mission = json.load(infile)
  print(mission)

rot_mission = {}
for wp in mission:
  print(wp)
  print(mission[wp]["x"])
  rot_mission.update({wp:{}})
  x = mission[wp]["x"]
  y = mission[wp]["y"]
  down = mission[wp]["z"]
  heading = mission[wp]["heading"]

  if heading != -1:
    heading += rot_deg
    if  heading < 0:
      heading += 360
    if 360 < heading:
      heading -=360

  try:
    speed = mission[wp]["speed"]
  except:
    speed = False
  north = round(x * math.cos(rot_rad) - y * math.sin(rot_rad),1)
  east = round(x * math.sin(rot_rad) + y * math.cos(rot_rad),1)

  rot_mission[wp].update({"north": north})
  rot_mission[wp].update({"east": east})
  rot_mission[wp].update({"down": down})
  rot_mission[wp].update({"heading": heading})
  if speed:
    rot_mission[wp].update({"speed": speed})

print(rot_mission)

with open('Mission.json','w',encoding='utf-8') as outfile:
  outfile.write(json.dumps(rot_mission, indent=4))
