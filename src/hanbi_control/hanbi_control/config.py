
params = {
    "MAP" : {
        "RESOLUTION": 0.1,
        "OCCUPANCY_UP": 0.02,
        "OCCUPANCY_DOWN": 0.01,
        "CENTER": (-50, -50),
        "SIZE": (50, 50),
        "FILENAME": 'test.png',
        "MAPVIS_RESIZE_SCALE": 2.0,
        "PATH" : 'C:\\dev\\ros2_smart_home\\src\\hanbi_control\\map\\map1005.txt'
    },
    "PATH" :{
        "PATH" : 'C:\\dev\\ros2_smart_home\\src\\hanbi_control\\path\\path.txt'
    }
}
params["MAP"]["origin"] = [params["MAP"]["CENTER"][0]-(params["MAP"]["SIZE"][0]/2), params["MAP"]["CENTER"][1]-(params["MAP"]["SIZE"][1]/2)]

#MAP_DATA 생성
MAP_DATA = []
f = open(params["MAP"]["PATH"], "r")
while True:
    line = list(map(int, f.readline().split()))
    if not line: break
    MAP_DATA.append(line)

#PATH_DATA 생성

def pose2grid(pose):
    [x, y] = pose
    return  [int((x - params["MAP"]["origin"][0])/params["MAP"]["RESOLUTION"]),int((y - params["MAP"]["origin"][1])/params["MAP"]["RESOLUTION"])]

    pass
def grid2pose(grid):
    [x,y] = grid
    resol = params["MAP"]["RESOLUTION"]
    origin = params["MAP"]["origin"]
    return [float(origin[0] - x * resol), float((origin[1] - y * resol))]

