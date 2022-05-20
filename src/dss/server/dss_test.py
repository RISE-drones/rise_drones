import dss.server


def test_heading_valid():
  assert dss.server.Server.heading_valid(None, 'course')
  assert dss.server.Server.heading_valid(None, 'poi')
  assert dss.server.Server.heading_valid(None, 20)
  assert not dss.server.Server.heading_valid(None, 'absolute')
  assert not dss.server.Server.heading_valid(None, -10)
  assert not dss.server.Server.heading_valid(None, 20.4)
  assert not dss.server.Server.heading_valid(None, '20')
