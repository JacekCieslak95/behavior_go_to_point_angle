# BehaviorGoToPointAngle #
Zachowanie umożliwia dronowi poruszanie się pod zadanym kątem do zadanego punktu. Nie jest wykorzystywany path planning. Na ten moment możliwe poruszanie tylko w płaszczyźnie XY.
### Przyjmowane argumrnty: ###
 coordinates=[x,y,z] lub relative_coordinates=[x,y,z]
 speed=a - prędkość osiągana przez drona. Nie podając argumentu domyślnie speed=5
 angle=b - kąt pod jakim dron porusza się do punktu, w stopniach. Domyślnie 0 stopni.
