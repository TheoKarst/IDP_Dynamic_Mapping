import random
from mapping.geometry_maps.bool_axis import BoolAxis

axis = BoolAxis(False)
  
assert axis.get_value(100) is False
assert axis.get_value(-100) is False
assert len(axis.splits) == 0
print("TEST: Init OK")

axis.set_value(-50, 50, False)

assert axis.get_value(-100) is False
assert axis.get_value(100) is False
assert axis.get_value(0) is False
assert len(axis.splits) == 0
print("TEST: Set default OK")

axis.set_value(10, 10, True)
assert len(axis.splits) == 0
print("TEST: Zero range OK")

axis.set_value(-50, 50, True)

assert axis.get_value(-100) is False
assert axis.get_value(100) is False
assert axis.get_value(0) is True
assert len(axis.splits) == 2
print("TEST: Set opposite OK")

axis.set_value(-100, 100, False)

assert len(axis.splits) == 0
assert axis.get_value(0) is False
print("TEST: Remove range OK")

axis.set_value(-50, 0, True)
axis.set_value(0, 50, True)

assert len(axis.splits) == 2
assert axis.get_value(0) is True
assert axis.get_value(-25) is True
assert axis.get_value(-100) is False
assert axis.get_value(100) is False
assert axis.get_value(25) is True
print("TEST: Ranges contact OK")

axis.set_value(0, 100, False)

assert len(axis.splits) == 2
assert axis.get_value(-100) is False
assert axis.get_value(-25) is True
assert axis.get_value(25) is False
assert axis.get_value(75) is False
print("TEST: Ranges overlap OK")

axis.set_value(10, 20, True)

assert len(axis.splits) == 4
assert axis.get_value(15) is True
print("TEST: Ranges included OK")

for i in range(1000):
    min = 200 * random.random() - 100
    max = 200 * random.random() - 100
    axis.set_value(min, max, random.random() < 0.5)

    assert len(axis.splits) % 2 == 0

print("TEST: Size modulo 2 OK")
