from human import Human

W, H = 20, 12
free = lambda x, y: 0 <= x < W and 0 <= y < H and not (x == 10 and y != 5)  # wall at x=10, gap at y=5

def test_stays_in_free_cells():
    h = Human((2, 2), free, W, H, seed=1)
    for _ in range(300):
        h.step()
        assert free(*h.pos)

def test_velocity_and_prediction_straight():
    h = Human((2, 2), free, W, H, seed=1)
    h.pos = (3, 2); h.history.clear(); h.history.extend([(2, 2), (3, 2)])
    assert h.velocity() == (1, 0)
    p = h.predicted_cells(4)
    assert p[0] == {(3, 2)} and p[1] == {(4, 2)} and p[4] == {(7, 2)}

def test_prediction_stops_at_wall():
    h = Human((8, 2), free, W, H, seed=1)
    h.pos = (9, 2); h.history.clear(); h.history.extend([(8, 2), (9, 2)])
    p = h.predicted_cells(5)
    assert p[5] == {(9, 2)}          # wall at x=10, y=2

def test_half_speed_prediction():
    h = Human((2, 2), free, W, H, speed=2, seed=1)
    h.pos = (3, 2); h.history.clear(); h.history.extend([(2, 2), (3, 2), (3, 2)])
    assert h.velocity() == (1, 0)    # last real move, not (0,0)
    p = h.predicted_cells(4)
    assert p[1] == {(3, 2)} and p[2] == {(4, 2)} and p[4] == {(5, 2)}

def test_buffer():
    h = Human((5, 5), free, W, H, seed=1)
    assert len(h.predicted_cells(0, buffer=1)[0]) == 5

if __name__ == "__main__":
    for n, f in list(globals().items()):
        if n.startswith("test_"):
            f(); print("ok", n)
    print("all passed")
