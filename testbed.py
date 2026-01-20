import numpy as np
from modelspace.ModelSpacePy import CartesianVector3

np.random.seed(42)
thing = CartesianVector3([1.0,2.0,3.0])
for j in range(10):
    b = np.random.normal(0,500,(3,1))
    for i in range(3):
        thing.set(i, thing.get(i) + b[i][0])
    print(b)
print(thing.get(0), thing.get(1), thing.get(2))  