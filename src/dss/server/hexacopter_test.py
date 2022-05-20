from dss.server.hexacopter import Waypoint


def test_get_3D_distance():
  stockholm = Waypoint(59.334591, 18.063240, 19)
  norrkoping = Waypoint(58.588455, 16.188313, 12)
  (_, _, d_alt, _, _, bearing) = norrkoping.get_3D_distance_to(stockholm)
  assert d_alt == 7
  assert 52 <= bearing <= 53
