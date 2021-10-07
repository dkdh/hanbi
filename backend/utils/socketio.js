const fs = require("fs")
const path = require("path")
const params = require("../config")
const fd = fs.openSync(path + 'map.txt', 'w+')
var outF = fs.createWriteStream(null, { flags: 'w', fd });
const { Mutex } = require('async-mutex');
const mutex = new Mutex()

// db
const db = require("./db.js");
const Record = require("./models/Record.js");
const Lost = require("./models/Lost.js");

const moment = require('moment')
require('moment-timezone')
moment.tz.setDefault("Asia/Seoul")

var info = {
  mapq: [],
  map: {
    queue: [],
    data: [],
    dSizeY: 500,
    dSizeX: 500
  },
  robot: {
    battery: 100,
    velocity: 0,
    pos: [0, 0],
    mode: 0
  },
  log: [{ timestamp: new Date(), content: "from nodejs" }, { timestamp: new Date(), content: "from nodejs2" }, { timestamp: new Date(), content: "from nodejs3" }],
  environment: {
    weather: "Cloudy",
    temperature: "30"
  }
}
//* map
for (let y = 0; y < info.map.dSizeY; y++) {
  info.map.data.push([])
  for (let x = 0; x < info.map.dSizeX; x++) {
    info.map.data[y].push(-1)
  }
}

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

    socket.on("SocketNode2Server", (data) => {
      const { robot, environment } = data

      info.robot = robot
      info.environment = environment

    })

    //로직 1. 맵 이벤트
    socket.on("Map2Server", async (data) => {
      //로직 1-1. get Map data from ROS
      // const release = await mutex.acquire()
      // console.log("get map from ROS", data)
      for (let i = 0; i < data.length; i++) {
        const [y, x, next_value] = data[i]
        if (info.map.data[y][x] != next_value) {
          info.map.data[y][x] = next_value
          info.map.queue.push([y, x, next_value])
        }
      }
      console.log(info.map.queue.length)
      // release()
    });
    socket.on("Map2Web", async () => {
      // const release = await mutex.acquire()
      // console.log("emit Map to Web")
      idx = info.map.queue.length
      limit = 10000
      if (idx > limit) idx = limit
      socket.emit('Map2Web', info.map.queue.splice(0, idx));
      // release()
    })

    //로직 1. 로봇 이벤트
    socket.on("Robot2Web", async (data) => {
      // console.log("emit robot to Web", info.robot)
      socket.emit('Robot2Web', info.robot);
    })

    //로직 2. 로그 이벤트
    socket.on("History2Server", async (data) => {
      console.log("get Log from ROS")
      const { content } = data
      if (!content) return
      info.log.push({ timestamp: new Date(), content })
    });

    socket.on("History2Web", async (data) => {
      console.log("emit log to web")
      socket.emit('History2Web', info.log);
    })

    //로직 4. 환경 이벤트
    socket.on("Environment2Web", async (data) => {
      // console.log("emit Environment to web")
      socket.emit('Environment2Web', info.environment);
    })

    // 실시간 화면 요청
  socket.on('request_stream_from_vue', () => {
    socket.to(roomName).emit('request_stream_from_node');
    console.log("요청")
  });
  
  // 실시간 화면 응답
  socket.on('stream_from_python', (message) => {
    socket.to(roomName).emit('stream_from_nodejs', message);
    console.log("응답")
  });

  // 블랙박스 저장
  socket.on('uploadVideo', async (url) => {
    const date = moment().format('YYYY년 MM월 DD일 HH:mm:ss')
    try {
      const newRecord = await Record.create({
        fileUrl: url,
        createdAt: date
      });
    } catch (error) {
      console.log(error);
    };
  });

  });
};