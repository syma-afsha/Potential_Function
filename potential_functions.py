from matplotlib import pyplot as plt
from PIL import Image
import numpy as np
import sys

#Taking inputs from the terminal
if len(sys.argv) > 2:
    path_to_grid_map_image = str(sys.argv[1])
    start_x = int(sys.argv[2])
    start_y = int(sys.argv[3])
    goal_x = int(sys.argv[4])
    goal_y = int(sys.argv[5])
    repulsQ = float(sys.argv[6])


def chooseImage(path):
    image = Image.open(path).convert('L')
    grid_map = np.array(image.getdata()).reshape(image.size[0], image.size[1])/255

    # Binarize the image
    grid_map[grid_map > 0.5]  = 1
    grid_map[grid_map <= 0.5] = 0

    # Invert colors to make 0 -> free and 1 -> occupied
    grid_map = (grid_map *-1 +1)

    return grid_map

image_map=chooseImage(path_to_grid_map_image)
start = [start_x, start_y]
goal  = [goal_x, goal_y]

# plot original grid map
fig = plt.figure(figsize=(30,10))
fig.add_subplot(3,4,1)
plt.imshow(image_map)
plt.title('Original Grid Map')
plt.colorbar()

#8 point connectivity with wavefront planner
def waveFrontPlanner(map, counter):
    rows = len(map)
    cols = len(map[0])
    for i in range(rows-1):
        for j in range(cols-1):
            if map[i, j] == counter + 1:
                if map[i-1, j-1] == 0:
                    map[i-1, j-1] = counter+2
                if map[i-1, j] == 0:
                    map[i-1, j] = counter+2
                if map[i-1, j+1] == 0 :
                    map[i-1, j+1] = counter+2
                if map[i, j-1] == 0 :
                    map[i, j-1] = counter+2
                if map[i, j+1] == 0 :
                    map[i, j+1] = counter+2
                if map[i+1, j-1] == 0:
                    map[i+1, j-1] = counter+2
                if map[i+1, j] == 0 :
                    map[i+1, j] = counter+2
                if map[i+1, j+1] == 0:
                    map[i+1, j+1] = counter+2
    return map

#Attraction Function Map
attraction_function_map = chooseImage(path_to_grid_map_image)
rows=len(attraction_function_map)
cols=len(attraction_function_map[0])
attraction_function_map[goal[0], goal[1]] = 2.0
attraction_counter = 1
for i in range(rows):
    for j in range(cols):
        while attraction_function_map[i,j] == 0:
            waveFrontPlanner(attraction_function_map, attraction_counter)
            attraction_counter = attraction_counter + 1

#normalize attraction map
def normalizer(map, counter):
    rows = len(map)
    cols = len(map[0])
    max_val = float(np.amax(map))
    min_val = float(np.amin(map))
    for i in range(rows):
        for j in range (cols):
            if map[i,j] != counter:
                map[i,j] = float(map[i,j] - min_val)
                map[i,j] = float(map[i,j] / max_val)
    return map

normalize_counter = 1
normalized_attraction_map = normalizer(attraction_function_map, normalize_counter)


# plot attraction function
fig.add_subplot(3,4,2)
plt.imshow(normalized_attraction_map)
plt.title('Attraction Function')
plt.colorbar()
#
#
# resulting function
def resultingPath(map, start_x , start_y, goal_x, goal_y):
    result =[start_x, start_y]
    map[goal_x, goal_y] = 0
    while map[start_x, start_y]!= 0:
        resultCounter = 0
        map[start_x, start_y] = 1
        neighbour= np.zeros([1,8])
        for i in [-1,0,1]:
            for j in [-1,0,1]:
                if i == 0 and j == 0:
                    pass
                else:
                    neighbour[0, resultCounter]= map[ start_x +i, start_y+j]
                    resultCounter = resultCounter +1

        min_index = np.argmin(neighbour)
        if min_index == 0:
            start_x = start_x -1
            start_y = start_y -1
        elif min_index == 1:
            start_x = start_x -1
            start_y = start_y
        elif min_index == 2:
            start_x = start_x -1
            start_y = start_y +1
        elif min_index == 3:
            start_x = start_x
            start_y = start_y -1
        elif min_index == 4:
            start_x = start_x
            start_y = start_y+1
        elif min_index == 5:
            start_x = start_x +1
            start_y = start_y -1
        elif min_index == 6:
            start_x = start_x +1
            start_y = start_y
        elif min_index == 7:
            start_x = start_x +1
            start_y = start_y +1
        result.append((start_x,start_y))

    return map, result

#path finding
first_path_map = np.copy(normalized_attraction_map)
first_path_map, way_attraction = resultingPath(first_path_map, start[0], start[1], goal[0], goal[1])

fig.add_subplot(3,4,3)
plt.imshow(first_path_map)
plt.title('Path on attraction function')
plt.scatter(start_y,start_x, 50, c="b", marker="+", label = 'Start')
plt.scatter(goal_y, goal_x, 50, c="r", marker="o", label = 'Goal')
plt.legend(loc='best')
plt.colorbar()




#8 point connectivity with brushFire
def brushFire(map, counter):
    rows = len(map)
    cols = len(map[0])
    for i in range(rows):
        for j in range(cols):
            if map[i,j] == counter and i < rows - 1 and j < cols - 1: #considering obstacles
                if map[i-1, j-1] == 0:
                    map[i-1, j-1] = counter + 1
                if map[i-1, j] == 0:
                    map[i-1, i] = counter + 1
                if map[i-1, j+1] == 0 :
                    map[i-1, j+1] = counter + 1
                if map[i, j-1] == 0 :
                    map[i, j-1] = counter + 1
                if map[i, j+1] == 0 :
                    map[i, j+1] = counter + 1
                if map[i+1, j-1] == 0:
                    map[i+1, j-1] = counter + 1
                if map[i+1, j] == 0 :
                    map[i+1, j] = counter + 1
                if map[i+1, j+1] == 0:
                    map[i+1, j+1] = counter + 1
            if map[i, j] == counter and i < rows - 1 and j >= cols - 1:
                if map[i, j-1] == 0:
                    map[i, j-1] = counter + 1
            if map[i, j] == counter and i >= rows - 1 and j < cols - 1:
                if map[i-1, j] == 0:
                    map[i-1, j] = counter + 1

    return map

#Brushfire Map
brushfire_map = chooseImage(path_to_grid_map_image)
brush_counter = 1
for i in range(rows):
    for j in range(cols):
        while brushfire_map[i,j] == 0:
            brushFire(brushfire_map, brush_counter)
            brush_counter = brush_counter + 1

fig.add_subplot(3,4,4)
plt.imshow(brushfire_map)
plt.title('Brushfire')
plt.colorbar()

#Repulsion map
def repulsiveFunction(map, Q):
    C = 1
    rows = len(map)
    cols = len(map[0])
    for i in range(rows-1):
        for j in range(cols-1):
            if map[i, j] <= Q and map[i,j] != 1:
                map[i,j] = C * ((1/map[i,j]) - (1/Q))**2
            elif map[i,j] > Q:
                map[i,j] = 0

    return map

repulsion_map = np.copy(brushfire_map)
repulsion_map = repulsiveFunction(repulsion_map, repulsQ)
fig.add_subplot(3,4,5)
plt.imshow(repulsion_map)
plt.title('Repulsive function')
plt.colorbar()


#Normalised Potential Map
def normalizer1(map):
    rows = len(map)
    cols = len(map[0])
    max_val = float(np.amax(map))
    min_val = float(np.amin(map))
    for i in range(rows):
        for j in range (cols):
            map[i,j] = float(map[i,j] - min_val)
            map[i,j] = float(map[i,j] / max_val)
    return map

# Potential function map
potential_map = attraction_function_map + repulsion_map
normalized_potential_map = np.copy(potential_map)
normalized_potential_map = normalizer1(normalized_potential_map)
fig.add_subplot(3,4,6)
plt.imshow(normalized_potential_map)
plt.title('Potential function')
plt.colorbar()



#Optimal Path keeping distance from obstacles
result_Map = np.copy(normalized_potential_map)
result_Map, way = resultingPath(result_Map, start[0], start[1], goal[0], goal[1])
print('Path = ', way)
fig.add_subplot(3,4,7)
plt.imshow(result_Map)
plt.title('Resulting Path')
plt.scatter(start_y,start_x, 50, c="b", marker="+", label = 'Start')
plt.scatter(goal_y, goal_x, 50, c="r", marker="o", label = 'Goal')
plt.legend(loc='best')
plt.show()
