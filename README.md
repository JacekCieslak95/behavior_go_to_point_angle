# Behavior GoToPointAngle

Read in [English]

Paczka tworzy behavior używany przez [Aerostack] (oprogramowanie grupy [Vision4UAV])
Zachowanie umożliwia dronowi poruszanie się pod zadanym kątem do zadanego punktu. Nie jest wykorzystywany path planning.
### Instalacja ###
1. Pliki niniejszego repozytorium należy umieścić w folderze 
    `~/workspace/ros/aerostack_catkin_ws/src/`
    tak, aby tworzyły poniższe drzewo:
    
        ~/workspace/ros/aerostack_catkin_ws/src/
            -behavior_go_to_point_angle
    		    -CMakeLists.txt
                -package.xml
                -launch
                    -behavior_go_to_point_angle.launch
    			-src
                    -include
                        -behavior_go_to_point_angle.h
                    -source
                        -behavior_go_to_point_angle.cpp
                        -behavior_go_to_point_angle_main.cpp

2. Przeprowadzić kompilację catkin `~/workspace/ros/aerostack_catkin_ws/$ catkin_make`
3. Edytować plik `simulated_quadrotor_basic.sh` - W skrypcie uruchamiającym należy dokleić na końcu poniższe linie:
    
	    `#----------------------------------------------` \
	    `# Behavior GoToPointAngle                                   ` \
	    `#----------------------------------------------` \
	    --tab --title "Behavior GoToPointAngle" --command "bash -c \"
	    roslaunch behavior_go_to_point_angle behavior_go_to_point_angle.launch --wait \
    		drone_id_namespace:=drone$NUMID_DRONE \
    		drone_id:=$NUMID_DRONE \
    		my_stack_directory:=${AEROSTACK_STACK};
    	exec bash\"" \
    
4. Edytować plik `behavior_catalog.yaml`. Plik znajduje się w lokalizacji: `~/workspace/ros/aerostack_catkin_ws/src/aerostack_stack/configs/droneX` 
    W sekcji `behavior_descriptors` należy dokleić poniższe linie:
#### UWAGA! Należy to wkleić do folderu `configs/droneX` każdego drona, którego chcemy uruchamiać z danym zachowaniem.
#### Np. Używając tego w dronach 1 i 2 poniższy fragment należy dokleić do `behavior_catalog.yaml` w folderach `configs/drone1` oraz `configs/drone2`
	    
		
          - behavior: GO_TO_POINT_ANGLE
            timeout: 30
            incompatible_lists: [motion_behaviors]
            capabilities: [SETPOINT_BASED_FLIGHT_CONTROL, PATH_PLANNING]
            arguments:
              - argument: COORDINATES
                allowed_values: [-100,100]
                dimensions: 3
              - argument: RELATIVE_COORDINATES
                allowed_values: [-100,100]
                dimensions: 3
              - argument: ANGLE
                allowed_values: [-360,360]
              - argument: SPEED
                allowed_values: [0,30]
              - argument: AVOID_DRONE_ID
                allowed_values: [0,10]
		    
				
##### UWAGA! Wcięcia powinny być realizowane przez spacje, nie tabulatory!

### Przyjmowane argumenty ###
Behavior przyjmuje argumenty:
    
    coordinates=[x,y,z]
    
lub
    
    relative_coordinates=[x,y,z]
    
Jest to punkt na który dron będzie skierowany.
    
    angle=x
    
Jest to kąt (w stopniach) pod jakim dron będzie skierowany względem punktu. Domyślnie 0 stopni.
    
    speed=x
    
Jest to prędkość osiągana przez drona. Nie podając argumentu domyślnie speed=5

    avoid_drone_ID=x

Jest to numer drona, z którym kolizjii dron będzie unikał. Uwaga, przy niepodaniu numeru - dron nie unika kolizji.

Przykład wywołania:
`result = api.executeBehavior('GO_TO_POINT_ANGLE', coordinates=[6.5, 6.5, 3], speed=5, angle=30, avoid_drone_id=2)`

[//]: # (These are reference links used in the body of this note and get stripped out when the markdown processor does its job. There is no need to format nicely because it shouldn't be seen. Thanks SO - http://stackoverflow.com/questions/4823468/store-comments-in-markdown-syntax)
   [Polish]: <https://github.com/JacekCieslak95//behavior_go_to_point_angle/blob/master/README.md>
   [English]: <https://github.com/JacekCieslak95//behavior_go_to_point_angle/blob/master/README_en.md>
   [Aerostack]: <https://github.com/Vision4UAV>
   [Vision4UAV]: <https://github.com/Vision4UAV/Aerostack>

