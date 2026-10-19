"""scene.Renderer 몸 층 재사용(body_key) 확인 — 캐시를 쓴 렌더가 안 쓴 렌더와 같아야 한다 (로봇이 돌아도).
실행: .venv/bin/python test_scene.py"""
import numpy as np

import maze
import scene

r = scene.Renderer(np.asarray(maze.wall_boxes(maze.load()), float))
tri = np.array([[[0.25, -0.1, 0.0], [0.25, 0.1, 0.0], [0.25, 0.0, 0.2]]])  # 앞 0.25m 세운 삼각형
cam = scene.transform(scene.rpy_matrix(-np.pi / 2 - 0.4, 0, -np.pi / 2), (0.0, 0.0, 0.3))  # 앞·아래를 보는 카메라
for yaw in (0.0, 1.0):  # yaw 0에서 만든 몸 층을 yaw 1에서 재사용
    twb = scene.transform(scene.rpy_matrix(0, 0, yaw), (0.3, 0.2, 0))
    boxes = [(twb @ scene.transform(np.eye(3), (0.15, 0, 0.05)), 0.1, 0.2, 0.1, (40, 40, 200))]
    meshes = [(twb, tri, (200, 40, 40))]
    b0, d0 = r.render(twb @ cam, [], boxes, meshes)
    b1, d1 = r.render(twb @ cam, [], boxes, meshes, body_key=('cam', False, ()))
    assert not np.array_equal(d0, r.render(twb @ cam, [])[1]), yaw  # 몸이 실제로 화면에 보임
    assert np.array_equal(d0, d1), yaw
    assert np.abs(b0.astype(int) - b1).max() <= 1, yaw
print('ok')
