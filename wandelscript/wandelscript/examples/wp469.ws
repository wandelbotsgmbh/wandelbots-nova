home = (400, -100, 300, 0, pi, 0)

velocity(2000)
move via p2p() to home
move via p2p() to home :: (0, 0, 100, 0, 0, 0)
move via p2p() to home :: (0, 0, -100, 0, 0, 0)
with velocity(100):
    move via arc(home :: (0, 100, 200, 0, 0, 0)) to home :: (0, 0, 400, 0, 0, 0) with velocity(100)
