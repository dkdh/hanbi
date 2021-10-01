const fs = require("fs")
const path = require("path")
const params = require("../config")
const fd = fs.openSync(path + 'map.txt', 'w+')
var outF = fs.createWriteStream(null, { flags: 'w', fd });


info = {
  map: [],
  robot: [], // y, x
  log: []
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
    //(issue) socket.to(roomName) 하면 ROS 요청을 수신 못함

    socket.join(roomName)
    console.log("Connect from Client: ");

    //로직 1. 맵 이벤트
    socket.on("Map2Server", async (data) => {
      //로직 1-1. get Map data from ROS
      console.log("get map from ROS", data)
      info.map = data
    });
    socket.on("Map2Web", async (data) => {
      console.log("emit Map to Web")

      // socket.to(roomName).emit('Map2Web', mapData);
      socket.emit('Map2Web', info.map);
    })

    //로직 2. 로봇 이벤트
    socket.on("Robot2Server", async (data) => {
      console.log("get robot from ROS")
      info.robot = data
    });
    socket.on("Robot2Web", async (data) => {
      console.log("emit robot to Web")

      // socket.to(roomName).emit('Robot2Web', mapData);
      socket.emit('Robot2Web', info.robot);
    })

    //로직 3. 로그 이벤트
    socket.on("Log2Server", async (data) => {
      console.log("get Log from ROS")

      //todo 로직 3-1. get Log history and save DB
      //로직 3-1. 테스트 케이스 
      info.log.push({ timestamp: new Date(), content: "from nodejs" }, { timestamp: new Date(), content: "from nodejs2" }, { timestamp: new Date(), content: "from nodejs3" })
    });
    socket.on("Log2Web", async (data) => {
      //todo 
      //! socket.to(roomName).emit('Robot2Web', mapData);
      socket.emit('Robot2Web', info.log);
    })
  });
};