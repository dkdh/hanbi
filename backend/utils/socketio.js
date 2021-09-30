const fs = require("fs")
const path = require("path")
const params = require("../config")
const fd = fs.openSync(path + 'map.txt', 'w+')
var outF = fs.createWriteStream(null, { flags: 'w', fd });


info = {
  map: [],
  robot: [] // y, x
}
//* 테스트 데이터 주입
info.map = fs.readFileSync(path.resolve(__dirname, "../", "assets", "map.txt"), "utf-8")
info.map = info.map.split("\n")
for (let i = 0; i < info.map.length; i++) {
  info.map[i] = info.map[i].split(' ')
}
info.robot = [50, 20]

module.exports.createSocket = function (http_server) {
  const io = require("socket.io")(http_server, {
    cors: {
      origin: params["origin"],
      methods: ["GET", "POST"],
      transports: ["websocket", "polling"],
      credentials: true,
    },
    allowEIO3: true,
    allowEI04: true,
  });


  const roomName = 'team'
  io.on("connection", async (socket) => {
    //소켓 서버 생성
    socket.join(roomName)
    console.log("Connect from Client: ");

    socket.on("Map2Server", async (data) => {
      //로직 1. get Map data from ROS
      console.log("get map from ROS")
      info.map = data
    });
    socket.on("Robot2Server", async (data) => {
      //로직 1. get Map data from ROS
      console.log("get robot from ROS")
      info.robot = data
    });

    socket.on("Map2Web", async (data) => {
      // console.log("Map2Web")
      //! socket.to(roomName).emit('Map2Web', mapData);
      socket.emit('Map2Web', info.map);
    })
    socket.on("Robot2Web", async (data) => {
      // console.log("Map2Web")
      //! socket.to(roomName).emit('Robot2Web', mapData);
      socket.emit('Robot2Web', info.robot);
    })
  });
};